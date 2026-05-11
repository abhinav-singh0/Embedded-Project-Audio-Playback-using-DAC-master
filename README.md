# 🔊 Embedded WAV Audio Player using Tiva C LaunchPad

A standalone real-time embedded audio player built using the **Tiva C TM4C123 LaunchPad**, **MCP4725 12-bit DAC**, **MicroSD card**, and **LM386 audio amplifier**.

The system reads WAV audio files from a FAT32 MicroSD card, processes PCM samples in real time using timer-driven interrupts, and generates analog audio output through an external DAC and amplifier — all without using any external DSP or codec IC.

---

# 📌 Features

- Real-time WAV audio playback
- FAT32 MicroSD card support
- Double buffering for uninterrupted playback
- Timer-driven ISR audio scheduling
- FIFO buffering between ISR and DAC writes
- MCP4725 12-bit DAC interfacing via I2C
- LM386-based audio amplification
- Oscilloscope-verified analog waveform output
- Smooth playback at 8 kHz and 16 kHz

---

# 🧠 System Architecture

```text
SD Card → Tiva C MCU → MCP4725 DAC → LM386 Amplifier → Speaker
```

The Tiva C microcontroller handles:
- WAV file parsing
- Buffer management
- Sample timing
- FIFO synchronization
- DAC communication

---

# ⚙️ Hardware Components

| Component | Description |
|---|---|
| Tiva C TM4C123 LaunchPad | ARM Cortex-M4F MCU running at 80 MHz |
| MCP4725 DAC | 12-bit I2C DAC for analog waveform generation |
| MicroSD Card Module | FAT32 audio file storage |
| LM386 Amplifier | Audio amplification stage |
| 8Ω Speaker | Audio output device |

---

# 🔄 Working Principle

## 1. System Initialization

The MCU:
- Configures PLL and runs at 80 MHz
- Initializes:
  - GPIO
  - SPI
  - I2C
  - Timer0A

---

## 2. WAV File Loading

The system:
- Mounts FAT32 filesystem
- Opens `.wav` audio file
- Verifies:
  - RIFF header
  - WAVE signature
- Extracts:
  - Sample rate
  - Channels
  - Bits per sample
  - PCM data location

---

## 3. Double Buffering

Two 4 KB buffers are used:

```text
buf0 → active playback
buf1 → background refill
```

This ensures continuous playback without audio gaps.

---

## 4. Timer-Driven Playback

Timer0A generates interrupts at the exact audio sample rate.

Example:

```text
8 kHz audio → interrupt every 125 µs
```

Each interrupt triggers playback of one audio sample.

---

## 5. Interrupt Service Routine (ISR)

The ISR:
- Selects active buffer
- Extracts PCM sample
- Converts signed 16-bit PCM to unsigned 12-bit DAC value
- Pushes sample into FIFO

The ISR never accesses:
- SD card
- I2C bus

This prevents timing violations and playback glitches.

---

# 📦 FIFO Buffering

FIFO acts as a synchronization layer between:
- Real-time ISR sample generation
- Variable-time I2C DAC writes

This prevents:
- Audio underruns
- Timing jitter
- Playback instability

---

# 🎵 DAC and Audio Output

The MCP4725 DAC:
- Converts digital samples to analog voltage
- Outputs 0–3.3V waveform

The LM386 amplifier:
- Amplifies DAC signal
- Drives 8Ω speaker
- Produces audible real-time audio output

---

# 📊 Experimental Results

- Smooth playback verified at:
  - 8 kHz
  - 16 kHz
- Stable waveform observed on oscilloscope
- LM386 successfully drove external speaker
- FIFO prevented playback glitches during SD delays

---

# 📷 Real Circuit Implementation

The project includes:
- Physical circuit implementation
- SD card module
- DAC module
- LM386 amplifier
- Speaker integration

All signals were verified using an oscilloscope.

---

# 🛠️ Tech Stack

## Software
- Embedded C
- Timer Interrupts
- FAT32 File Handling
- SPI Communication
- I2C Communication

## Hardware
- TM4C123 Tiva C LaunchPad
- MCP4725 DAC
- LM386 Amplifier
- MicroSD Module

---

# 🚀 Future Improvements

- Stereo audio playback
- Volume control
- Bluetooth audio streaming
- DMA-based audio transfer
- Higher sampling rates

---

# 📂 Project Structure

```text
embedded-audio-player/
│
├── src/
│   ├── main.c
│   ├── audio.c
│   ├── dac.c
│   ├── sdcard.c
│
├── include/
│   ├── audio.h
│   ├── dac.h
│
├── images/
│   ├── block.png
│   ├── realckt.jpg
│   ├── waveform.jpg
│
├── presentation/
│   └── embedded_audio_player.pdf
│
└── README.md
```

---

# 🎓 Academic Context

This project was developed as part of the **Embedded Systems Design** course at:

**Indian Institute of Technology Dharwad**

---

# 👨‍💻 Author

**Abhinav Singh**  
Indian Institute of Technology Dharwad

GitHub: https://github.com/abhinav-singh0
