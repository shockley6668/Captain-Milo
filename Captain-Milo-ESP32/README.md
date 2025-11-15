# Teddy Bear Companion - Captain Milo

English | [中文](README_CN.md)

## Project Overview

**Captain Milo** is an intelligent teddy bear companion developed by the **E094 Team (Cathay Hackathon)** specifically designed for unaccompanied minor passengers. Built on the ESP32 platform, it integrates ASR (Automatic Speech Recognition), LLM (Large Language Models), TTS (Text-to-Speech), and IoT technology to provide companionship, guidance, and safety protection for young travelers flying independently through voice interaction.

## Table of Contents

- [System Architecture](#system-architecture)
- [Main Features](#main-features)
- [Hardware Support](#hardware-support)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Build Instructions](#build-instructions)
- [Configuration Guide](#configuration-guide)
- [API Reference](#api-reference)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

## System Architecture

### Core Components

```
┌─────────────────────────────────────────────────────────────┐
│          Application Layer (Voice Interaction)              │
│                  (Main Event Loop)                           │
└─────────────────────────────────────────────────────────────┘
         ↓                    ↓                    ↓
    ┌─────────┐          ┌─────────┐         ┌─────────┐
    │  Audio  │          │ Protocol│         │  LED    │
    │ Service │          │ Manager │         │ Feedback│
    └─────────┘          └─────────┘         └─────────┘
         ↓                    ↓                    ↓
    ┌─────────────────────────────────────────────────────────┐
    │        Hardware Abstraction Layer (HAL)                 │
    │   • Audio Codec • Network Stack • GPIO and LED          │
    └─────────────────────────────────────────────────────────┘
         ↓
    ┌─────────────────────────────────────────────────────────┐
    │              ESP-IDF Framework                           │
    │    (FreeRTOS, NVS Flash, OTA, WiFi, etc.)              │
    └─────────────────────────────────────────────────────────┘
```

### Voice Interaction Pipeline

```
User Speech → Microphone → Audio Codec → Audio Processor (AEC, Noise Suppression, VAD)
                                                    ↓
                                        Speech Recognition (ASR)
                                                    ↓
                                    Large Language Model (LLM) Processing
                                                    ↓
                                    Text-to-Speech (TTS) Generation
                                                    ↓
                                    Opus Encoding → Speaker Playback
                                                    ↓
                                Protocol Stack (MQTT/WebSocket/MCP) Sync
```

## Main Features

### Voice Interaction
- **Real-time Speech Recognition**: Capture clear voice input via I2S interface
- **Acoustic Echo Cancellation (AEC)**: Dual-mode support (device-side and server-side)
- **Noise Suppression and Voice Activity Detection (VAD)**: Using ESP-ADF front-end processing
- **Text-to-Speech (TTS)**: Natural and fluent voice output, supporting multiple languages
- **Large Language Model Integration**: Intelligent understanding and response to user needs
- **Opus Audio Encoding**: High-quality audio compression with ultra-low latency

### Connectivity and Integration
- **WiFi Integration**: Secure WiFi connection with automatic retry
- **WebSocket Protocol**: Real-time bidirectional communication
- **Model Context Protocol (MCP)**: Advanced AI integration framework

### System Management
- **OTA Firmware Updates**: Seamless over-the-air updates for continuous service optimization
- **Settings Persistence**: Non-volatile storage for user configurations
- **Device State Management**: Comprehensive device state tracking
- **System Information**: Real-time CPU usage, memory, and health monitoring

## Hardware Support

### Primary Target Boards
- **ESP-BOX-3**: Reference development board with dual-microphone array and LVGL display
- **ESP32-S3**: High-performance variant
- **ESP32-C3**: Low-cost option

### Audio Codec Chips
- ES8311, ES8374, ES8388, ES8389
- Support for multiple I2C/I2S configurations
- Automatic codec detection and initialization

### Display Panels
- ILI9341, GC9A01, ST77916, ST7701, ST7796, SPD2010
- OLED displays via GPIO bit-banging
- Configurable SPI/I2C interfaces

### Expansion Interfaces
- I/O Expander (TCA9554) support for extended GPIO
- NV3023 display driver

## Project Structure

```
.
├── CMakeLists.txt                 # Root CMake build configuration
├── sdkconfig.defaults             # Default ESP-IDF configuration
├── main/                          # Main application source code
│   ├── application.cc/.h          # Core application and event loop
│   ├── mcp_server.cc/.h           # Model Context Protocol server
│   ├── ota.cc/.h                  # OTA firmware update handler
│   ├── settings.cc/.h             # Persistent settings manager
│   ├── device_state_event.cc/.h   # Device state and event definitions
│   ├── system_info.cc/.h          # System monitoring and information
│   ├── audio/                     # Audio processing subsystem
│   │   ├── audio_service.cc/.h    # Audio service orchestrator
│   │   ├── audio_codec.cc/.h      # Audio codec hardware abstraction
│   │   ├── audio_processor.h      # Audio signal processing interface
│   │   ├── wake_word.h            # Wake word detection
│   │   ├── codecs/                # Specific codec implementations
│   │   ├── processors/            # Audio processor backends
│   │   ├── wake_words/            # Wake word models
│   │   └── README.md              # Detailed audio architecture
│   ├── led/                       # LED indicator subsystem
│   │   ├── led.h                  # LED abstraction interface
│   │   ├── single_led.cc/.h       # Single GPIO LED control
│   │   ├── gpio_led.cc/.h         # GPIO-based multi-LED control
│   │   └── circular_strip.cc/.h   # Addressable LED ring control
│   ├── protocols/                 # Network protocol implementations
│   │   ├── protocol.cc/.h         # Protocol abstraction base
│   │   ├── mqtt_protocol.cc/.h    # MQTT implementation
│   │   └── websocket_protocol.cc/.h # WebSocket implementation
│   ├── boards/                    # Board-specific configurations
│   │   ├── esp-box-3/            # ESP-BOX-3 board support
│   │   └── common/               # Common board utilities
│   ├── assets/                    # Application resources
│   │   ├── common/               # Shared resources
│   │   ├── audio/                # Audio resources and voice library
│   │   └── locales/              # Multi-language translations
│   ├── main.cc                    # Program entry point
│   └── idf_component.yml          # Dependency specifications
├── docs/                          # Documentation
│   ├── mcp-protocol.md           # MCP protocol documentation
│   ├── mqtt-udp.md               # MQTT/UDP protocol guide
│   ├── websocket.md              # WebSocket implementation guide
│   ├── custom-board.md           # Custom board porting guide
│   └── v0/, v1/                  # Version-specific documentation
├── partitions/                    # Flash partition configurations
│   ├── v1/                       # Version 1 partition scheme
│   └── v2/                       # Version 2 partition scheme
├── scripts/                       # Build and utility scripts
│   ├── build_default_assets.py   # Asset generation
│   ├── gen_lang.py               # Language file generation
│   ├── release.py                # Release build script
│   ├── acoustic_check/           # Audio debugging tools
│   ├── Image_Converter/          # Image to LVGL format conversion
│   ├── ogg_converter/            # Audio format conversion
│   ├── p3_tools/                 # P3 audio format tools
│   └── spiffs_assets/            # SPIFFS filesystem asset packing
└── README.md                      # This file
```

## Getting Started

### Prerequisites

- **ESP-IDF v5.0+**: Download from [https://github.com/espressif/esp-idf](https://github.com/espressif/esp-idf)
- **Python 3.7+**: For build scripts and tools
- **CMake 3.16+**: Build system
- **Hardware**: ESP32-S3 or compatible board with audio codec

### Installation

1. **Clone the Repository**
   ```bash
   git clone <repository-url>
   cd Cathy-Captain-Milo
   ```

2. **Set Up ESP-IDF Environment**
   ```bash
   source /path/to/esp-idf/export.sh
   ```

3. **Install Python Dependencies** (optional, for utility scripts)
   ```bash
   pip install -r scripts/requirements.txt
   ```

## Build Instructions

### Basic Build

```bash
# Configure the project for your target board
idf.py set-target esp32s3
idf.py menuconfig

# Build the project
idf.py build

# Flash to device
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

### Building for Specific Boards

**ESP-BOX-3:**
```bash
idf.py set-target esp32s3
idf.py build -D "IDF_TARGET=esp32s3"
idf.py -p /dev/ttyUSB0 flash monitor
```

**Custom Board:**
Refer to [docs/custom-board.md](docs/custom-board.md) for porting instructions.

### Build Configuration

The project uses `sdkconfig` for ESP-IDF configuration. Key settings:

- **Audio Codec Selection**: Configure via `menuconfig` → Audio Configuration
- **Display Type**: LCD, OLED, or LVGL
- **Partition Scheme**: Located in `partitions/v2/` (recommended)
- **WiFi Settings**: Set SSID and password in configuration
- **OTA Settings**: Enable/disable OTA updates

## Configuration Guide

### Device Configuration (`menuconfig`)

```bash
idf.py menuconfig
```

Key configuration areas:
- **Audio** → Codec selection, sample rate, AEC mode
- **Display** → Panel type, resolution, interface (SPI/I2C)
- **Network** → WiFi SSID, password, protocol selection
- **OTA** → Update server URL, certificate validation
- **System** → Log level, memory management

### Runtime Settings

Edit `main/settings.cc` or use the MCP server to configure at runtime:

```cpp
// Example: Set AEC mode
app.SetAecMode(Application::kAecOnServerSide);

// Example: Change listening mode
app.StartListening();
app.StopListening();
```

## API Reference

### Application Core

```cpp
// Get application instance (singleton)
Application& app = Application::GetInstance();

// Device state management
app.SetDeviceState(DeviceState state);
DeviceState current_state = app.GetDeviceState();

// Voice control
app.StartListening();
app.StopListening();
app.WakeWordInvoke("你好，Milo");

// Audio playback
app.PlaySound("notification.ogg");

// Alert system
app.Alert("warning", "Low battery", "alert.ogg");
app.DismissAlert();

// OTA updates
app.UpgradeFirmware(ota, "https://example.com/firmware.bin");

// System control
app.Reboot();
```

### Audio Service

```cpp
// Get audio service
AudioService& audio = app.GetAudioService();

// Check voice activity
bool has_voice = audio.IsVoiceDetected();

// Get audio metrics
auto metrics = audio.GetAudioMetrics();

// Encode/decode audio
audio.PushAudioFrame(pcm_buffer, size);
Opus packet = audio.PopEncodedAudio();
```

## Documentation

- **[Audio Architecture](main/audio/README.md)**: Detailed audio processing pipeline
- **[MCP Protocol](docs/mcp-protocol.md)**: Model Context Protocol implementation
- **[MQTT/UDP Protocol](docs/mqtt-udp.md)**: IoT connectivity guide
- **[WebSocket Guide](docs/websocket.md)**: Real-time communication setup
- **[Custom Board Guide](docs/custom-board.md)**: Porting to new hardware

## Development Tools

### Scripts

- **Asset Builder**: `python scripts/build_default_assets.py`
- **Language Generator**: `python scripts/gen_lang.py`
- **Audio Format Converter**: `python scripts/ogg_converter/xiaozhi_ogg_converter.py`
- **Image Converter**: `python scripts/Image_Converter/LVGLImage.py`
- **P3 Audio Tools**: `python scripts/p3_tools/convert_audio_to_p3.py`

### Audio Debugging

Use the acoustic debugging tools in `scripts/acoustic_check/`:

```bash
cd scripts/acoustic_check
python main.py
```

## Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is part of the Cathay Pacific Hackathon. Refer to the LICENSE file for details.

## Support & Contact

For questions, issues, or suggestions:
- Create an issue in the repository
- Contact the E094 Team (Captain Milo)
- Refer to the [documentation](docs/)

## Acknowledgments

- **Cathay Pacific Hackathon** Team for inspiration and resources
- **Espressif** for ESP-IDF and community support
- **Open-source community** for audio and display libraries
- **E094 Team members** for development and testing

---

**Project Version**: 2.0.4
**Last Updated**: November 2024
**Status**: Active Development
