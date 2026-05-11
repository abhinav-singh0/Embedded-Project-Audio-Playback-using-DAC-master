#include "diskio.h"
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "sysctl.h"
#include "gpio.h"
#include "ssi.h"
#include "pin_map.h"

// SPI pins: PA2=SCK, PA5=MOSI, PA4=MISO
// CS pin: PA3 (GPIO)

#define CS_PORT GPIO_PORTA_BASE
#define CS_PIN  GPIO_PIN_3

static volatile DSTATUS Stat = STA_NOINIT;

// ----------------------- SPI helpers -----------------------
static void spi_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);

    GPIOPinTypeGPIOOutput(CS_PORT, CS_PIN);
    GPIOPinWrite(CS_PORT, CS_PIN, CS_PIN);

    // Initial clock speed: 400kHz or less. This is necessary for all cards.
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 400000, 8);

    SSIEnable(SSI0_BASE);
}

static uint8_t spi_txrx(uint8_t b)
{
    uint32_t r;
    SSIDataPut(SSI0_BASE, b);
    while(SSIBusy(SSI0_BASE));
    SSIDataGet(SSI0_BASE, &r);
    return (uint8_t)r;
}

static void cs_low(void)
{
    GPIOPinWrite(CS_PORT, CS_PIN, 0);
}

static void cs_high(void)
{
    GPIOPinWrite(CS_PORT, CS_PIN, CS_PIN);
    spi_txrx(0xFF); // Send clock pulse to meet SD timing requirement
}

// ----------------------- Send CMD (CORRECTED) -----------------------
static uint8_t send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t res;
    uint8_t crc = 0x01;
    int n;

    // Calculate CRC for CMD0 and CMD8
    if (cmd == 0) crc = 0x95;
    if (cmd == 8) crc = 0x87;

    // --- NEW: ENSURE CS IS HIGH BEFORE STARTING ---
    cs_high();
    spi_txrx(0xFF); // Send a dummy clock pulse

    // Select the card
    cs_low();

    // Command packet
    spi_txrx(0x40 | cmd);
    spi_txrx(arg >> 24);
    spi_txrx(arg >> 16);
    spi_txrx(arg >> 8);
    spi_txrx(arg);
    spi_txrx(crc);

    // Wait for response (R1 is single byte, starts with 0)
    for (n = 0; n < 255; n++) // Increased timeout
    {
        res = spi_txrx(0xFF);
        if (!(res & 0x80)) {
             // Response received!
             return res;
        }
    }

    // --- NEW: MUST DESELECT IF RESPONSE IS NOT FOUND ---
    cs_high();
    return res;
}

// ----------------------- Disk I/O API: Initialize (CORRECTED) -----------------------
DSTATUS disk_initialize(BYTE drv)
{
    uint8_t type = 0;
    uint8_t res;
    int i;

    if (drv != 0) return STA_NOINIT;

    // The SPI peripheral is initialized to a slow clock speed (400kHz) here.
    spi_init();

    // 1. Initial wake-up (Send 10 * 0xFF with CS high)
    for (i = 0; i < 10; i++) spi_txrx(0xFF);

    // 2. Send CMD0 (Go Idle State)
    if (send_cmd(0, 0) != 1) {
        // Red LED cause: Card failed to enter idle state (0x01)
        return STA_NOINIT;
    }
    cs_high(); // Deselect after successful CMD0

    // 3. Send CMD8 (Check Voltage) - Deselect happens inside send_cmd
    if (send_cmd(8, 0x1AA) == 1)
    {
        // Read 4 extra bytes for R7 response (ignored, but necessary)
        spi_txrx(0xFF); spi_txrx(0xFF); spi_txrx(0xFF); spi_txrx(0xFF);
        type = 1; // SDv2/SDHC compatible
    }
    cs_high(); // Deselect after successful CMD8

    // 4. ACMD41 loop (Initialization)
    for (i = 0; i < 200000; i++)
    {
        // CMD55 must precede ACMD41
        if (send_cmd(55, 0) > 1) continue; // If card is busy, try again.

        uint32_t arg = (type == 1) ? (1UL << 30) : 0; // HCS bit for SDHC
        res = send_cmd(41, arg);

        if (res == 0) {
            // Initialization Complete!

            // --- CRITICAL FIX: REDUCE DATA TRANSFER SPEED ---
            // Increase the speed to a faster rate for data transfer (e.g., 5MHz)
            // You must use the DriverLib function here:
            // SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
            //                    SSI_MODE_MASTER, 5000000, 8);

            Stat &= ~STA_NOINIT;
            cs_high();
            return Stat;
        }
    }

    // Fallback/Timeout Failure
    cs_high();
    return STA_NOINIT;
}

DSTATUS disk_status(BYTE drv)
{
    if (drv != 0) return STA_NOINIT;
    return Stat;
}

DRESULT disk_read(BYTE drv, BYTE* buff, DWORD sector, BYTE count)
{
    BYTE c;
    int i;
    int n;
    uint32_t timeout = 0;

    if (drv != 0 || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    for (c = 0; c < count; c++)
    {
        if (send_cmd(17, sector + c) != 0)
            return RES_ERROR;

        uint8_t token;
        // Wait for start block token (0xFE) with a generous timeout
        timeout = 0;
        for(n = 0; n < 500000; n++) { // Increased timeout for read token
            token = spi_txrx(0xFF);
            if (token != 0xFF) break;
        }

        if (token != 0xFE)
            return RES_ERROR;

        for (i = 0; i < 512; i++)
            buff[c * 512 + i] = spi_txrx(0xFF);

        spi_txrx(0xFF); spi_txrx(0xFF);
        cs_high();
    }

    return RES_OK;
}

DRESULT disk_write(BYTE drv, const BYTE* buff, DWORD sector, BYTE count)
{
    BYTE c;
    int i;
    uint32_t timeout;

    if (drv != 0 || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    for (c = 0; c < count; c++)
    {
        if (send_cmd(24, sector + c) != 0)
            return RES_ERROR;

        spi_txrx(0xFE); // Start block token

        for (i = 0; i < 512; i++)
            spi_txrx(buff[c * 512 + i]);

        spi_txrx(0xFF); // Write 16-bit CRC (dummy)
        spi_txrx(0xFF);

        uint8_t resp = spi_txrx(0xFF);
        if ((resp & 0x1F) != 0x05)
            return RES_ERROR;

        // --- CRITICAL FIX 2: ROBUST BUSY WAIT ---
        // Add robust software timeout for card internal programming.
        timeout = 0;
        while(spi_txrx(0xFF) != 0xFF) {
            if (timeout++ > 0x100000) return RES_ERROR; // Increased timeout for stability
        }

        spi_txrx(0xFF); // Trailing clock pulse

        cs_high();
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE drv, BYTE cmd, void* buff)
{
    if (drv != 0) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd)
    {
        case GET_SECTOR_SIZE: *(WORD*)buff = 512; return RES_OK;
        case GET_BLOCK_SIZE:  *(DWORD*)buff = 1;  return RES_OK;
        case CTRL_SYNC: return RES_OK;
    }
    return RES_PARERR;
}
