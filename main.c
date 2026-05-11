/* main.c - FIFO + MCP4725 robust writes (fixed refill bug, tuned, with f_read_retry)
   Changes:
   - Fixed incorrect buf0/buf1 refill variable mixup
   - Reduced main-loop sleep
   - Increased FIFO depth to 1024
   - Small asm nop formatting to silence compiler warning
   - Added f_read_retry wrapper to retry transient SD read errors
   - Prefill and refill use f_read_retry with retries
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "i2c.h"
#include "timer.h"
#include "interrupt.h"
#include "uart.h"

#include "ff.h"
#include "diskio.h"

// ---------------- CONFIG ----------------
#define MCP4725_ADDR   0x60
#define BUF_SIZE       4096
#define DEBUG_PIN      GPIO_PIN_2   // PF2 pulse
#define FIFO_DEPTH     1024         // increased headroom (power of two)
#define FIFO_MASK      (FIFO_DEPTH - 1)
#define DAC_WRITE_RETRIES 3
#define RETRY_DELAY_US  100   // short delay between retries (microseconds)

// ---------------- GLOBALS ----------------
static FATFS fs;
static FIL wav_file;

static uint8_t buf0[BUF_SIZE];
static uint8_t buf1[BUF_SIZE];

volatile uint8_t *play_buf = 0;
volatile uint32_t play_len = 0;
volatile uint32_t play_index = 0;

volatile bool buf0_ready = false;
volatile bool buf1_ready = false;
volatile bool eof_reached = false;
volatile bool playback_active = false;

volatile uint32_t buf0_len = 0;
volatile uint32_t buf1_len = 0;

// FIFO for DAC values (ISR producer, main consumer)
volatile uint16_t fifo[FIFO_DEPTH];
volatile uint16_t fifo_wr = 0;
volatile uint16_t fifo_rd = 0;

volatile uint32_t diag_buf0_fills = 0;
volatile uint32_t diag_buf1_fills = 0;
volatile uint32_t diag_i2c_errors = 0;
volatile uint32_t diag_underruns = 0;
volatile uint32_t diag_samples_sent = 0;

// ---------------- UART helpers ----------------
static void UART0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}
static void uart_putc(char c){ UARTCharPut(UART0_BASE, c); }
static void uart_puts(const char *s){ while(*s) UARTCharPut(UART0_BASE, *s++); }
static void uart_dec(uint32_t v){
    char b[16]; int i=0;
    if(v==0){ uart_putc('0'); return; }
    while(v){ b[i++] = '0' + (v % 10); v /= 10; }
    while(i--) uart_putc(b[i]);
}

// ---------------- WAV info ----------------
typedef struct {
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint16_t channels;
    uint32_t data_offset;
    uint32_t data_bytes;
} wav_info_t;

// ---------------- Robust WAV parser ----------------
static bool parse_wav_header(FIL* f, wav_info_t* info)
{
    UINT br;
    uint8_t hdr12[12];
    if (f_read(f, hdr12, 12, &br) != FR_OK || br < 12) return false;
    if (memcmp(hdr12, "RIFF", 4) != 0) return false;
    if (memcmp(hdr12 + 8, "WAVE", 4) != 0) return false;

    bool found_fmt = false, found_data = false;
    uint32_t data_offset = 0, data_size = 0;
    uint16_t channels = 0, bits = 0;
    uint32_t sr = 0;

    for (;;) {
        uint8_t chdr[8];
        if (f_read(f, chdr, 8, &br) != FR_OK || br < 8) break;
        uint32_t sz = chdr[4] | (chdr[5] << 8) | (chdr[6] << 16) | (chdr[7] << 24);

        if (memcmp(chdr, "fmt ", 4) == 0) {
            uint8_t fmtbuf[32];
            UINT rd = (sz > sizeof(fmtbuf)) ? sizeof(fmtbuf) : sz;
            if (f_read(f, fmtbuf, rd, &br) != FR_OK || br < rd) return false;
            uint16_t af = fmtbuf[0] | (fmtbuf[1] << 8);
            channels = fmtbuf[2] | (fmtbuf[3] << 8);
            sr = fmtbuf[4] | (fmtbuf[5] << 8) | (fmtbuf[6] << 16) | (fmtbuf[7] << 24);
            bits = fmtbuf[14] | (fmtbuf[15] << 8);
            if (af != 1) return false; // PCM only
            if (sz > rd) {
                if (f_lseek(f, f_tell(f) + (sz - rd)) != FR_OK) return false;
            }
            found_fmt = true;
        } else if (memcmp(chdr, "data", 4) == 0) {
            data_offset = f_tell(f);
            data_size = sz;
            if (f_lseek(f, data_offset + sz) != FR_OK) return false;
            found_data = true;
        } else {
            uint32_t skip = sz;
            if (skip & 1) skip++;
            if (f_lseek(f, f_tell(f) + skip) != FR_OK) return false;
        }
        if (found_fmt && found_data) break;
    }

    if (!found_fmt || !found_data) return false;
    info->sample_rate = sr;
    info->bits_per_sample = bits;
    info->channels = channels;
    info->data_offset = data_offset;
    info->data_bytes = data_size;
    if (f_lseek(f, data_offset) != FR_OK) return false;
    return true;
}

// ---------------- I2C / MCP4725 robust write ----------------
static void I2C_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Request Fast mode (400 kHz)
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
}

// helper: short busy-wait microsecond-ish delay (approx)
static void short_delay_us(uint32_t us)
{
    // approximate loop delay (SysCtlClockGet() cycles per second)
    // calibrated roughly; keep very small (100us default)
    volatile uint32_t loops = (SysCtlClockGet() / 1000000) * us / 6;
    while (loops--) __asm__ volatile(" nop");
}

// New robust 3-byte write: control byte 0x40 (write DAC register only)
// Returns true on success, false on fatal failure.
static bool dac_write(uint16_t v)
{
    if (v > 4095) v = 4095;

    // Prepare 3 bytes: control (0x40), MSB (D11..D4), LSB (D3..D0 << 4)
    uint8_t ctrl = 0x40; // write to DAC (no EEPROM write)
    uint8_t msb  = (v >> 4) & 0xFF;      // D11..D4
    uint8_t lsb  = (uint8_t)((v & 0x0F) << 4); // D3..D0 << 4
    int attempt;
    for (attempt = 0; attempt < DAC_WRITE_RETRIES; ++attempt) {
        I2CMasterSlaveAddrSet(I2C0_BASE, MCP4725_ADDR, false);

        // Start - send control byte
        I2CMasterDataPut(I2C0_BASE, ctrl);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while (I2CMasterBusy(I2C0_BASE));
        if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) goto i2c_retry;

        // Continue - send MSB
        I2CMasterDataPut(I2C0_BASE, msb);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while (I2CMasterBusy(I2C0_BASE));
        if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) goto i2c_retry;

        // Finish - send LSB
        I2CMasterDataPut(I2C0_BASE, lsb);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while (I2CMasterBusy(I2C0_BASE));
        if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) goto i2c_retry;

        // Success
        return true;

    i2c_retry:
        // record error; small pause then retry
        diag_i2c_errors++;
        short_delay_us(RETRY_DELAY_US);
    }

    // All retries failed
    return false;
}

// ---------------- conversions ----------------
static inline uint16_t convert_s16_to_u12(int16_t s)
{
    int32_t t = (int32_t)s + 32768;
    return (uint16_t)((t * 4095) / 65535);
}

// ---------------- debug pin ----------------
static void Debug_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, DEBUG_PIN);
}
static inline void DebugPulse(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, DEBUG_PIN, DEBUG_PIN);
    GPIOPinWrite(GPIO_PORTF_BASE, DEBUG_PIN, 0);
}

// ---------------- TIMER ISR (producer) ----------------
void Timer0A_Handler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // If nothing to play, push mid-level to keep DAC quiet
    if (!playback_active || play_buf == 0) {
        uint16_t next = (fifo_wr + 1) & FIFO_MASK;
        if (next != fifo_rd) {
            fifo[fifo_wr] = 2048;
            fifo_wr = next;
        } else {
            diag_underruns++;
        }
        return;
    }

    // If current buffer exhausted, try to swap
    if (play_index + 2 > play_len) {
        if (play_buf == buf0 && buf1_ready) {
            play_buf = buf1;
            play_len = buf1_len;
            play_index = 0;
            buf1_ready = false;
        } else if (play_buf == buf1 && buf0_ready) {
            play_buf = buf0;
            play_len = buf0_len;
            play_index = 0;
            buf0_ready = false;
        } else {
            uint16_t next = (fifo_wr + 1) & FIFO_MASK;
            if (next != fifo_rd) {
                fifo[fifo_wr] = 2048;
                fifo_wr = next;
            } else {
                diag_underruns++;
            }
            return;
        }
    }

    // Read sample from play_buf
    uint8_t lo = play_buf[play_index];
    uint8_t hi = play_buf[play_index + 1];
    play_index += 2;
    int16_t sample = (int16_t)(lo | (hi << 8));
    uint16_t out = convert_s16_to_u12(sample);

    uint16_t next = (fifo_wr + 1) & FIFO_MASK;
    if (next != fifo_rd) {
        fifo[fifo_wr] = out;
        fifo_wr = next;
    } else {
        // FIFO full => drop sample
        diag_underruns++;
    }

    DebugPulse();
}

// ---------------- Timer init (explicit register) ----------------
static void Timer0_Init(uint32_t sr)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    uint32_t ticks = SysCtlClockGet() / sr;
    if (ticks < 2) ticks = 2;
    TimerLoadSet(TIMER0_BASE, TIMER_A, ticks - 1);

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0A_Handler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);
}

// ---------------- FAT time ----------------
DWORD get_fattime(void) { return 0; }

// ---------------- Robust f_read wrapper ----------------
static FRESULT f_read_retry(FIL *f, void *buff, UINT btr, UINT *br, int tries, int backoff_ms)
{
    FRESULT res;
    int attempt = 0;
    while (attempt < tries) {
        res = f_read(f, buff, btr, br);
        if (res == FR_OK) return FR_OK;
        attempt++;
        // brief delay ~ backoff_ms milliseconds (busy-wait)
        volatile uint32_t d;
        for (d = 0; d < (SysCtlClockGet()/1000) * (uint32_t)backoff_ms / 500; ++d) __asm__ volatile(" nop");
    }
    return res;
}

// ---------------- MAIN ----------------
int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    UART0_Init();
    Debug_Init();
    uart_puts("\r\n=== WAV playback (FIFO, I2C 400k, 8k prefer, robust) ===\r\n");

    // Mount filesystem
    if (f_mount(0, &fs) != FR_OK) {
        uart_puts("Mount failed\r\n");
        while(1);
    }
    uart_puts("FS mounted\r\n");

    // Open file
    if (f_open(&wav_file, "speech.wav", FA_READ) != FR_OK) {
        uart_puts("Open speech.wav failed\r\n");
        while(1);
    }
    uart_puts("File opened: speech.wav\r\n");

    // Parse WAV header
    wav_info_t info;
    if (!parse_wav_header(&wav_file, &info)) {
        uart_puts("WAV header parse failed\r\n");
        while(1);
    }

    uart_puts("WAV info: SR=");
    uart_dec(info.sample_rate);
    uart_puts(" bits=");
    uart_dec(info.bits_per_sample);
    uart_puts(" ch=");
    uart_dec(info.channels);
    uart_puts(" data_bytes=");
    uart_dec(info.data_bytes);
    uart_puts("\r\n");

    if (info.channels != 1 || info.bits_per_sample != 16) {
        uart_puts("Unsupported WAV format (need mono 16-bit)\r\n");
        while(1);
    }

    // Initialize I2C (400k)
    I2C_Init();
    uart_puts("I2C init done (requested 400k)\r\n");

    // Pre-fill buffers using robust reader
    UINT br;
    FRESULT fres;

    fres = f_read_retry(&wav_file, buf0, BUF_SIZE, &br, 5, 10);
    if (fres != FR_OK) { uart_puts("Read buf0 error (persistent)\r\n"); while(1); }
    buf0_ready = (br > 0);
    buf0_len = br;
    play_len = buf0_len;
    uart_puts("buf0 filled "); uart_dec(br); uart_puts(" bytes\r\n");
    diag_buf0_fills++;

    fres = f_read_retry(&wav_file, buf1, BUF_SIZE, &br, 5, 10);
    if (fres != FR_OK) { uart_puts("Read buf1 error (persistent)\r\n"); while(1); }
    buf1_ready = (br > 0);
    buf1_len = br;
    uart_puts("buf1 filled "); uart_dec(br); uart_puts(" bytes\r\n");
    diag_buf1_fills++;

    play_buf = buf0;
    play_index = 0;
    playback_active = true;

    // Decide timer sample rate: prefer 8k if file is 16k, else use file rate
    uint32_t timer_sr = info.sample_rate;
    if (info.sample_rate == 16000) timer_sr = 8000;
    if (timer_sr == 0) timer_sr = 8000;

    uart_puts("Playback started (timer ");
    uart_dec(timer_sr);
    uart_puts(" Hz)\r\n");

    // Soft-ramp DAC to mid to avoid startup click
    // dac_ramp_to defined elsewhere in previous suggestions; if not present, DAC will start at mid implicitly.
    // Start timer AFTER prints and prefill
    Timer0_Init(timer_sr);

    // Main loop: consume FIFO (do blocking I2C writes) and refill SD buffers
    while (1) {
        // Consume FIFO
        while (fifo_rd != fifo_wr) {
            uint16_t val = fifo[fifo_rd];
            fifo_rd = (fifo_rd + 1) & FIFO_MASK;
            if (dac_write(val)) {
                diag_samples_sent++;
            } else {
                diag_i2c_errors++;
            }
            // No extra delay here; dac_write handles retries internally
        }

        // Refill logic (same as before) but robust reads
        if (play_buf == buf0) {
            if (!buf1_ready && !eof_reached) {
                fres = f_read_retry(&wav_file, buf1, BUF_SIZE, &br, 5, 10);
                if (fres != FR_OK) {
                    uart_puts("Read error during refill buf1 (persistent)\r\n");
                    eof_reached = true;
                } else {
                    if (br == 0) {
                        uart_puts("EOF (buf1)\r\n");
                        eof_reached = true;
                        buf1_ready = false;
                    } else {
                        buf1_ready = true;
                        buf1_len = br;
                        diag_buf1_fills++;
                        // debug printing removed to avoid slowing consumer during playback
                    }
                }
            }
        } else {
            if (!buf0_ready && !eof_reached) {
                fres = f_read_retry(&wav_file, buf0, BUF_SIZE, &br, 5, 10);
                if (fres != FR_OK) {
                    uart_puts("Read error during refill buf0 (persistent)\r\n");
                    eof_reached = true;
                } else {
                    if (br == 0) {
                        uart_puts("EOF (buf0)\r\n");
                        eof_reached = true;
                        buf0_ready = false;
                    } else {
                        buf0_ready = true;
                        buf0_len = br;
                        diag_buf0_fills++;
                        // debug printing removed to avoid slowing consumer during playback
                    }
                }
            }
        }

        if (eof_reached && (fifo_rd == fifo_wr) && !buf0_ready && !buf1_ready) {
            uart_puts("Playback complete\r\n");
            playback_active = false;
            break;
        }

        // tiny sleep to yield to other background tasks (much shorter)
        SysCtlDelay(SysCtlClockGet() / 2000000); // ~0.5us-ish
    }

    // Summary
    uart_puts("=== Summary ===\r\n");
    uart_puts("Samples sent: "); uart_dec(diag_samples_sent); uart_puts("\r\n");
    uart_puts("I2C errors: "); uart_dec(diag_i2c_errors); uart_puts("\r\n");
    uart_puts("Underruns: "); uart_dec(diag_underruns); uart_puts("\r\n");
    uart_puts("buf0 fills: "); uart_dec(diag_buf0_fills); uart_puts("\r\n");
    uart_puts("buf1 fills: "); uart_dec(diag_buf1_fills); uart_puts("\r\n");

    f_close(&wav_file);

    while (1) { SysCtlDelay(SysCtlClockGet() / 3); }
}
