# 泰迪熊陪伴助手 Captain Milo
## Teddy Bear Companion for Unaccompanied Minors

[English](README.md) | 中文

## 项目概述

**Captain Milo** 是由**国泰航空黑客松E094团队**开发的一只智能泰迪熊，专为无陪伴未成年乘客设计。基于ESP32平台，集成ASR（语音识别）、LLM（大语言模型）、TTS（文本转语音）和IOT技术，通过语音交互为独自乘机的未成年旅客提供陪伴、指导和安全保护。

## 目录

- [系统架构](#系统架构)
- [主要功能](#主要功能)
- [硬件支持](#硬件支持)
- [项目结构](#项目结构)
- [快速开始](#快速开始)
- [编译说明](#编译说明)
- [配置指南](#配置指南)
- [API参考](#api参考)
- [文档](#文档)
- [贡献](#贡献)
- [许可证](#许可证)

## 系统架构

### 核心组件

```
┌─────────────────────────────────────────────────────────────┐
│                应用层（语音交互）                      │
│                  （主事件循环）                              │
└─────────────────────────────────────────────────────────────┘
         ↓                    ↓                    ↓
    ┌─────────┐          ┌─────────┐         ┌─────────┐
    │ 音频    │          │ 协议    │         │ LED     │
    │ 服务    │          │ 管理器   │         │ 反馈    │
    └─────────┘          └─────────┘         └─────────┘
         ↓                    ↓                    ↓
    ┌─────────────────────────────────────────────────────────┐
    │        硬件抽象层 (HAL)                                 │
    │   • 音频编码器 • 网络栈 • GPIO和LED                    │
    └─────────────────────────────────────────────────────────┘
         ↓
    ┌─────────────────────────────────────────────────────────┐
    │              ESP-IDF 框架                               │
    │    (FreeRTOS、NVS Flash、OTA、WiFi等)                 │
    └─────────────────────────────────────────────────────────┘
```

### 语音交互流程

```
用户语音 → 麦克风 → 音频编码器 → 音频处理器 (AEC、噪声抑制、VAD)
                                    ↓
                            语音识别 (ASR)
                                    ↓
                        大语言模型 (LLM) 处理
                                    ↓
                        文本转语音 (TTS) 生成
                                    ↓
                        Opus编码 → 扬声器播放
                                    ↓
                协议栈 (MQTT/WebSocket/MCP) 同步上报
```

## 主要功能

### 语音交互
- **实时语音识别**：通过I2S接口捕获清晰的语音输入
- **声学回声消除（AEC）**：支持双模式（设备端和服务器端）
- **噪声抑制和语音活动检测（VAD）**：使用ESP-ADF前端处理
- **文本转语音（TTS）**：自然流畅的语音输出，支持多语言
- **大语言模型集成**：智能理解和回应用户需求
- **Opus音频编码**：高质量音频压缩，极低延迟

### 连接性与集成
- **WiFi集成**：安全的WiFi连接，支持自动重试
- **WebSocket协议**：实时双向通信
- **模型上下文协议（MCP）**：高级AI集成框架

### 系统管理
- **OTA固件更新**：无缝的空中更新，持续优化服务
- **设置持久化**：非易失性存储用户配置
- **设备状态管理**：全面的设备状态跟踪
- **系统信息**：实时CPU使用率、内存和健康监控



## 硬件支持

### 主要目标板
- **ESP-BOX-3**：参考开发板，配备双麦克风阵列和LVGL显示屏
- **ESP32-S3**：高性能变体
- **ESP32-C3**：低成本选项

### 音频编码器芯片
- ES8311、ES8374、ES8388、ES8389
- 支持多种I2C/I2S配置
- 自动编码器检测和初始化

### 显示面板
- ILI9341、GC9A01、ST77916、ST7701、ST7796、SPD2010
- 通过GPIO比特操作的OLED显示屏
- 可配置的SPI/I2C接口

### 扩展接口
- I/O扩展器（TCA9554）支持扩展GPIO
- NV3023显示驱动程序

## 📁 项目结构

```
.
├── CMakeLists.txt                 # 根CMake构建配置
├── sdkconfig.defaults             # 默认ESP-IDF配置
├── main/                          # 主应用源代码
│   ├── application.cc/.h          # 核心应用和事件循环
│   ├── mcp_server.cc/.h           # 模型上下文协议服务器
│   ├── ota.cc/.h                  # OTA固件更新处理
│   ├── settings.cc/.h             # 持久化设置管理器
│   ├── device_state_event.cc/.h   # 设备状态和事件定义
│   ├── system_info.cc/.h          # 系统监控和信息
│   ├── audio/                     # 音频处理子系统
│   │   ├── audio_service.cc/.h    # 音频服务协调器
│   │   ├── audio_codec.cc/.h      # 音频编码器硬件抽象
│   │   ├── audio_processor.h      # 音频信号处理接口
│   │   ├── wake_word.h            # 唤醒词检测
│   │   ├── codecs/                # 特定编码器实现
│   │   ├── processors/            # 音频处理器后端
│   │   ├── wake_words/            # 唤醒词模型
│   │   └── README.md              # 详细的音频架构
│   ├── led/                       # LED指示灯子系统
│   │   ├── led.h                  # LED抽象接口
│   │   ├── single_led.cc/.h       # 单GPIO LED控制
│   │   ├── gpio_led.cc/.h         # 基于GPIO的多LED控制
│   │   └── circular_strip.cc/.h   # 可编程LED环控制
│   ├── protocols/                 # 网络协议实现
│   │   ├── protocol.cc/.h         # 协议抽象基类
│   │   ├── mqtt_protocol.cc/.h    # MQTT实现
│   │   └── websocket_protocol.cc/.h # WebSocket实现
│   ├── boards/                    # 板级特定配置
│   │   ├── esp-box-3/            # ESP-BOX-3开发板支持
│   │   └── common/               # 通用板级工具
│   ├── assets/                    # 应用资源
│   │   ├── common/               # 共享资源
│   │   ├── audio/                # 音频资源和语音库
│   │   └── locales/              # 多语言翻译
│   ├── main.cc                    # 程序入口点
│   └── idf_component.yml          # 依赖规范
├── docs/                          # 文档
│   ├── mcp-protocol.md           # MCP协议文档
│   ├── mqtt-udp.md               # MQTT/UDP协议指南
│   ├── websocket.md              # WebSocket实现指南
│   ├── custom-board.md           # 自定义板级移植指南
│   └── v0/, v1/                  # 版本特定文档
├── partitions/                    # Flash分区配置
│   ├── v1/                       # 版本1分区方案
│   └── v2/                       # 版本2分区方案
├── scripts/                       # 构建和工具脚本
│   ├── build_default_assets.py   # 资源生成
│   ├── gen_lang.py               # 语言文件生成
│   ├── release.py                # 发布构建脚本
│   ├── acoustic_check/           # 音频调试工具
│   ├── Image_Converter/          # 图像转LVGL格式工具
│   ├── ogg_converter/            # 音频格式转换
│   ├── p3_tools/                 # P3音频格式工具
│   └── spiffs_assets/            # SPIFFS文件系统资源打包
└── README.md                      # 本文件
```

## 快速开始

### 前置要求

- **ESP-IDF v5.0+**：从 [https://github.com/espressif/esp-idf](https://github.com/espressif/esp-idf) 下载
- **Python 3.7+**：用于构建脚本和工具
- **CMake 3.16+**：构建系统
- **硬件**：ESP32-S3或兼容的带有音频编码器的开发板

### 安装

1. **克隆代码仓库**
   ```bash
   git clone <repository-url>
   cd Cathy-Captain-Milo
   ```

2. **设置ESP-IDF环境**
   ```bash
   source /path/to/esp-idf/export.sh
   ```

3. **安装Python依赖**（可选，用于工具脚本）
   ```bash
   pip install -r scripts/requirements.txt
   ```

## 🔨 编译说明

### 基础编译

```bash
# 为目标板配置项目
idf.py set-target esp32s3
idf.py menuconfig

# 构建项目
idf.py build

# 烧录到设备
idf.py -p /dev/ttyUSB0 flash

# 监控串口输出
idf.py -p /dev/ttyUSB0 monitor
```

### 为特定开发板编译

**ESP-BOX-3:**
```bash
idf.py set-target esp32s3
idf.py build -D "IDF_TARGET=esp32s3"
idf.py -p /dev/ttyUSB0 flash monitor
```

**自定义板**：
参考 [docs/custom-board.md](docs/custom-board.md) 获取移植说明。

### 编译配置

项目使用 `sdkconfig` 进行ESP-IDF配置。主要设置包括：

- **音频编码器选择**：通过 `menuconfig` → Audio Configuration
- **显示类型**：LCD、OLED或LVGL
- **分区方案**：位于 `partitions/v2/`（推荐）
- **WiFi设置**：在配置中设置SSID和密码
- **OTA设置**：启用/禁用OTA更新

## 配置指南

### 设备配置（`menuconfig`）

```bash
idf.py menuconfig
```

主要配置区域：
- **Audio** → 编码器选择、采样率、AEC模式
- **Display** → 面板类型、分辨率、接口（SPI/I2C）
- **Network** → WiFi SSID、密码、协议选择
- **OTA** → 更新服务器URL、证书验证
- **System** → 日志级别、内存管理

### 运行时设置

编辑 `main/settings.cc` 或使用MCP服务器进行运行时配置：

```cpp
// 示例：设置AEC模式
app.SetAecMode(Application::kAecOnServerSide);

// 示例：改变监听模式
app.StartListening();
app.StopListening();
```



## API参考

### 应用核心

```cpp
// 获取应用实例（单例）
Application& app = Application::GetInstance();

// 设备状态管理
app.SetDeviceState(DeviceState state);
DeviceState current_state = app.GetDeviceState();

// 语音控制
app.StartListening();
app.StopListening();
app.WakeWordInvoke("你好，Milo");

// 音频播放
app.PlaySound("notification.ogg");

// 告警系统
app.Alert("warning", "Low battery", "😟", "alert.ogg");
app.DismissAlert();

// OTA更新
app.UpgradeFirmware(ota, "https://example.com/firmware.bin");

// 系统控制
app.Reboot();
```

### 音频服务

```cpp
// 获取音频服务
AudioService& audio = app.GetAudioService();

// 检查语音活动
bool has_voice = audio.IsVoiceDetected();

// 获取音频指标
auto metrics = audio.GetAudioMetrics();

// 编码/解码音频
audio.PushAudioFrame(pcm_buffer, size);
Opus packet = audio.PopEncodedAudio();
```

```


## 文档

- **[音频架构](main/audio/README.md)**：详细的音频处理管道
- **[MCP协议](docs/mcp-protocol.md)**：模型上下文协议实现
- **[MQTT/UDP协议](docs/mqtt-udp.md)**：物联网连接指南
- **[WebSocket指南](docs/websocket.md)**：实时通信设置
- **[自定义板级指南](docs/custom-board.md)**：移植到新硬件

## 开发工具

### 脚本

- **资源构建器**：`python scripts/build_default_assets.py`
- **语言生成器**：`python scripts/gen_lang.py`
- **音频格式转换**：`python scripts/ogg_converter/xiaozhi_ogg_converter.py`
- **图像转换**：`python scripts/Image_Converter/LVGLImage.py`
- **P3音频工具**：`python scripts/p3_tools/convert_audio_to_p3.py`

### 音频调试

使用 `scripts/acoustic_check/` 中的音频调试工具：

```bash
cd scripts/acoustic_check
python main.py
```


