# SyncBox AI Firmware

**STM32F429 Bare-Metal Firmware for Pyrotechnics & Special Effects Controller**

Firmware Version: **0.79d LR** | Hardware Version: **3** | Target: **STM32F429ZGTx (Cortex-M4, 168 MHz)**

---

## Overview

SyncBox AI is a professional pyrotechnics and special effects synchronization controller. It coordinates wireless firing modules, DMX lighting, audio playback, and SMPTE timecode — all synchronized via GPS or internal clock. The system supports up to 100 slave devices over dual wireless links (2.4 GHz + 433 MHz).

## Architecture

- **Bare-metal** cooperative scheduler with visualSTATE state machine
- **No RTOS** — task scheduling via main-loop `task_Scheduler` switch
- **Memory**: 192 KB Main SRAM + 64 KB CCM RAM + 1 MB Flash (app starts at `0x0800C000`, bootloader below)
- **Toolchain**: Keil MDK-ARM (ARM Compiler v5.06)

## Features

### Communication
| Interface | Details |
|-----------|---------|
| **2.4 GHz Wireless** | Jennic JN5139 module, unicast + broadcast, network scanning, RSSI/LQI monitoring |
| **433 MHz Radio** | 84 channels (0-83), Master-Slave protocol, configurable network ID/key |
| **USB Host** | Mass storage (FAT32 via FatFS) for script loading and audio files |
| **USB Device** | CDC virtual COM port for PC control |
| **GPS** | NMEA parsing ($GNRMC/$GPRMC), PPS time sync, timezone support |
| **DMX 512** | 256-channel output via DMA (UART8), ramp/fade control |
| **SMPTE LTC** | Timecode generation and reception (25/33 fps) |
| **FSK Timecode** | F1 and PD format support |

### Control
- **100 slave devices** with real-time status monitoring (connected, armed, fired, errors)
- **10 AudioBox** peripherals for synchronized audio playback
- **4 pyro rails** with up to 16 channels each
- **16 safety zones** with programmable enable/disable
- **64 sequences** with named programs, time-based triggering
- **Manual fire** and step-fire modes

### Hardware I/O
- **7+ UARTs** (Wireless, GPS, 433 MHz, DMX, TimeCode, FSK, Debug)
- **SPI** LCD display (ILI9341), ADC
- **I2C** temperature/humidity sensor (SHT3x)
- **I2S** audio output (48 kHz WAV playback)
- **ADC** battery voltage, matrix voltage, capacitor monitoring
- **35+ GPIO buttons** with debouncing, rotary encoder
- **RGB LEDs** for status indication
- **Buzzer** for audio feedback
- **SD Card** (SDIO + FatFS)

### Safety
- **IWDG watchdog** for HardFault recovery
- **Bounds-checked** radio message parsing (433 MHz address validation)
- **Buffer overflow guards** on all string operations (`snprintf`/`strncat`)
- **Critical section protection** on shared ISR data (event queue)
- **Flash write protection** with sector bounds assertion

## Project Structure

```
SyncBox_AI/
├── Source/                  # Application source files
│   ├── main.c              # Entry point, scheduler, peripheral init
│   ├── Device.c            # Device management, slave tracking, 433MHz events
│   ├── Wireless.c          # 2.4GHz Jennic wireless driver
│   ├── uart.c              # Multi-UART driver (7+ ports), ring buffers
│   ├── Display.c           # ILI9341 LCD driver, UI rendering
│   ├── SystemTimer.c       # System timing, DMX ramp engine
│   ├── GlobalPositioning.c # GPS NMEA parser
│   ├── TimeCode.c          # SMPTE LTC / FSK timecode codec
│   ├── waveplayer.c        # WAV audio playback engine
│   ├── flash_if.c          # Internal flash programming
│   ├── USBHostMain.c       # USB host, file operations, scripting
│   ├── ADC.c               # Analog monitoring (battery, matrix)
│   ├── Buttons.c           # Button debouncing, encoder
│   └── beeper.c            # Buzzer control
├── Include/                # Header files
│   ├── options.h           # Build configuration, constants, pin mapping
│   ├── Device.h            # Data structures (SlaveInfo, MS_Message, etc.)
│   ├── Wireless.h          # Wireless protocol definitions
│   └── ...
├── EventQueue/             # Event-driven task queue (visualSTATE)
├── MDK-ARM/                # Keil project files, startup assembly
│   └── startup_stm32f407xx.s  # Vector table, stack/heap config
├── Objects/                # Linker scatter file (WP0001.sct)
├── Libraries/              # BSP, HAL, CMSIS, Fonts, LCD drivers
├── USB_DEVICE/             # USB CDC device class
├── vs/                     # visualSTATE state machine files
└── Listings/               # Build output (map files)
```

## Memory Layout

| Region | Address | Size | Usage |
|--------|---------|------|-------|
| **Flash** | `0x0800C000` | 1 MB (minus bootloader) | Application code + const data |
| **Main SRAM** | `0x20000000` | 192 KB | Device data, audio buffers, stack, heap, DMA buffers |
| **CCM RAM** | `0x10000000` | 64 KB | UART ring buffers, wireless data, timers, display, GPS (no DMA) |

**Key placement rules:**
- DMA-accessible buffers (`buf_DMX_DMA`, `hdma_uart8_tx`) must be in Main SRAM
- Non-DMA data (ring buffers, state variables) can use CCM for SRAM relief
- `savedDatas` struct (77 KB) is fixed at `0x2001C000` via linker attribute

## Building

1. Open `MDK-ARM/Project.uvprojx` in **Keil MDK-ARM v5**
2. Select target **STM32F4-DISCO**
3. Build (F7)
4. Output: `Objects/WP0001.hex`

## Configuration

Key build-time settings in `Include/options.h`:

| Define | Default | Description |
|--------|---------|-------------|
| `MAX_NUMBER_OF_DEVICES` | 100 | Maximum slave modules |
| `MAX_NUMBER_OF_AB` | 10 | Maximum AudioBoxes |
| `DMX_CH_MAX` | 256 | DMX channel count |
| `MAX_SEQ` | 64 | Maximum sequences |
| `MAXIMUM_433_MESSAGE_SIZE` | 32 | 433 MHz message buffer |
| `MAX_WIRELESS_MESSAGE_SIZE` | 256 | 2.4 GHz message buffer |
| `UART_BUFFER_SIZE` | 512 | UART ring buffer size |

## License

Proprietary. All rights reserved.
