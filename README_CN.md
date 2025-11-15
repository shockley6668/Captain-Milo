# Captain Milo - 智能泰迪熊陪伴系统

[中文](#中文) | [English](README.md)

---

<a name="中文"></a>

## 项目概述

**Captain Milo** 是由国泰航空黑客松 **E094 团队** 开发的智能泰迪熊陪伴系统，专为无陪伴未成年乘客设计。该系统基于 ESP32 和 RDK 平台，集成了 ASR（自动语音识别）、LLM（大语言模型）、TTS（文本转语音）、IOT 和 Web 技术，通过离线和在线语音交互为独自乘机的未成年旅客提供陪伴、指导和安全保护。

> **当前版本说明**：当前 demo 为**在线云端模型推理版本**，集成了实时云端 API 调用。完全离线的模型推理版本的推理模块为RDK-Offline-ASR、RDK-Offline-LLM、RDK-Offline-TTS

![Captain Milo 系统](docs/photo1.png)

## 核心特性

### 智能语音交互
- **实时语音识别 (ASR)**：采用 SenseVoice 离线识别引擎，支持 25+ 语言
- **语音活动检测 (VAD)**：集成 Silero VAD 进行精准的语音端点检测
- **声学回声消除 (AEC)**：支持设备端和服务器端双模式
- **降噪处理**：ESP-ADF 前端处理
- **文本转语音 (TTS)**：自然流畅的语音输出，支持多语言
- **大语言模型 (LLM)**：InternVL2_5-1B, InternVL3-1B等离线模型，支持离线交互

### 连接与集成
- **WiFi 连接**：支持自动重连的安全 WiFi 连接
- **WebSocket 协议**：实时双向通信
- **Model Context Protocol (MCP)**：先进的 AI 集成框架
- **MQTT/WebSocket/MCP 协议栈**：灵活的通信方案

### 系统管理
- **OTA 固件更新**：无缝的空中升级
- **配置持久化**：NVS Flash 存储用户配置
- **设备状态管理**：实时设备状态追踪
- **系统信息监控**：CPU 使用率、内存和健康状态

### 用户交互
- **LED 反馈**：柔光互动灯效，实时情感反馈
- **扬声器**：高质量音频输出，Opus 编码
- **麦克风阵列**：全向拾音，支持双麦克风阵列
- **传感器集成**：IMU、生物信息传感器支持

## 硬件支持

### 主要目标平台
| 平台 | 规格 | 功能 |
|------|------|------|
| **ESP32** | WiFi/BLE/Bluetooth | 主控制器、音频处理 |
| **RDK X5** | WiFi 6/5G/GPS/IoT | AI 加速、离线推理 |
| **Audio Codec** | I2S 接口 | ES8311/ES8374/ES8388 等 |
| **IMU 传感器** | 6 轴 | 姿态感知、运动检测 |
| **RGB LED** | 可编程 | 视觉反馈、情感表达 |
| **扩展传感器** | 多种 | 温度、湿度、生物信息 |

## 项目结构

```
Captain-Milo/
├── Captain-Milo-ESP32/                  # ESP32 主控制程序
│   ├── main/                            # 主应用程序
│   │   ├── application.cc/.h            # 核心应用层
│   │   ├── mcp_server.cc/.h             # MCP 服务器
│   │   ├── ota.cc/.h                    # OTA 更新模块
│   │   ├── settings.cc/.h               # 配置管理
│   │   ├── system_info.cc/.h            # 系统信息
│   │   ├── audio/                       # 音频处理子系统
│   │   │   ├── audio_service.cc/.h      # 音频服务
│   │   │   ├── audio_codec.cc/.h        # 编解码器
│   │   │   ├── wake_word.h              # 唤醒词检测
│   │   │   ├── codecs/                  # 特定编解码器
│   │   │   ├── processors/              # 音频处理器
│   │   │   └── wake_words/              # 唤醒词模型
│   │   ├── led/                         # LED 控制子系统
│   │   │   ├── led.h                    # LED 接口
│   │   │   ├── single_led.cc/.h         # 单 LED 控制
│   │   │   ├── gpio_led.cc/.h           # GPIO LED 控制
│   │   │   └── circular_strip.cc/.h     # 环形 LED 控制
│   │   ├── protocols/                   # 网络协议
│   │   │   ├── protocol.cc/.h           # 协议基类
│   │   │   ├── mqtt_protocol.cc/.h      # MQTT 实现
│   │   │   └── websocket_protocol.cc/.h # WebSocket 实现
│   │   ├── display/                     # 显示模块
│   │   │   ├── display.cc/.h            # 显示接口
│   │   │   ├── lcd_display.cc/.h        # LCD 显示
│   │   │   ├── oled_display.cc/.h       # OLED 显示
│   │   │   └── emote_display.cc/.h      # 表情显示
│   │   └── boards/                      # 板卡配置
│   ├── partitions/                      # 分区表配置
│   ├── docs/                            # 文档
│   ├── scripts/                         # 工具脚本
│   │   ├── build_default_assets.py      # 资源生成
│   │   ├── gen_lang.py                  # 语言生成
│   │   ├── ogg_converter/               # OGG 转换工具
│   │   ├── Image_Converter/             # 图像转换工具
│   │   ├── p3_tools/                    # P3 音频工具
│   │   └── acoustic_check/              # 声学调试工具
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults
│   └── README.md
│
├── RDK-Offline-ASR/                     # 离线语音识别模块（ROS2）
│   ├── src/
│   │   ├── speech_engine.cpp            # 语音引擎核心
│   │   └── hb_audio_capture.cpp         # 音频捕获
│   ├── include/
│   │   ├── speech_engine.h              # 语音引擎接口
│   │   └── sensevoice/                  # SenseVoice ASR 库
│   │       ├── sense-voice.h            # ASR 核心
│   │       ├── sense-voice-encoder.h    # 编码器
│   │       ├── sense-voice-decoder.h    # 解码器
│   │       ├── silero-vad.h             # VAD 模块
│   │       └── common.h                 # 公共定义
│   ├── config/                          # 配置文件
│   ├── launch/                          # ROS2 启动文件
│   ├── SenseVoiceGGUF/                  # 预训练模型
│   ├── CMakeLists.txt
│   └── package.xml
│
├── RDK-Offline-LLM/                     # 离线大语言模型模块（ROS2）
│   ├── src/                             # LLM 推理源码
│   ├── include/                         # LLM 接口头文件
│   ├── launch/                          # ROS2 启动配置
│   │   ├── llama_llm.launch.py          # 纯文本 LLM
│   │   ├── llama_vlm.launch.py          # 视觉语言模型
│   │   ├── ali.launch.py                # 阿里云集成
│   │   └── dosod.launch.py              # 目标检测集成
│   ├── llama.cpp/                       # llama.cpp 集成
│   ├── CMakeLists.txt
│   └── package.xml
│
├── RDK-Offline-TTS/                     # 离线文本转语音模块（ROS2）
│   ├── src/                             # TTS 推理源码
│   ├── include/                         # TTS 接口头文件
│   ├── wetts/                           # TTS 引擎
│   ├── CMakeLists.txt
│   └── package.xml
│
├── Cathay-web/                          # Web 交互界面
│   ├── src/                             # Vue 3 源码
│   ├── index.html                       # 主页面
│   ├── package.json                     # 依赖配置
│   ├── vite.config.ts                   # Vite 构建配置
│   ├── tsconfig.json
│   ├── tailwind.config.js
│   └── README.md
│
└── README.md                            
```

## 系统架构

### ESP32 语音交互（Demo）
```
用户语音 → 麦克风 → 音频编解码器 → 音频处理（AEC/降噪/VAD）
                                    ↓
                        语音识别（ASR）→ 本地/云端
                                    ↓
                        大语言模型处理（LLM）
                                    ↓
                        文本转语音（TTS）→ Opus 编码
                                    ↓
            扬声器播放 ← WebSocket/MQTT/MCP 同步
```

### RDK 平台离线推理
- **RDK-Offline-ASR**：SenseVoice 多语言离线语音识别
- **RDK-Offline-LLM**：llama.cpp 轻量级大语言模型推理
- **RDK-Offline-TTS**：高质量离线文本转语音

## 快速开始

### ESP32 编译与烧写

#### 1. 环境配置
```bash
# 安装 ESP-IDF v5.0+
source /path/to/esp-idf/export.sh

# 进入项目目录
cd Captain-Milo/Captain-Milo-ESP32
```

#### 2. 编译
```bash
# 配置项目
idf.py set-target esp32s3
idf.py menuconfig

# 编译
idf.py build
```

#### 3. 烧写
```bash
# 烧写到设备
idf.py -p /dev/ttyUSB0 flash

# 监控串口输出
idf.py -p /dev/ttyUSB0 monitor
```

### RDK 平台部署

#### 1. 编译 ASR 模块
```bash
cd RDK-Offline-ASR
mkdir build && cd build
cmake ..
make -j$(nproc)
```

#### 2. 编译 LLM 模块
```bash
cd RDK-Offline-LLM
mkdir build && cd build
cmake ..
make -j$(nproc)
```

#### 3. 编译 TTS 模块
```bash
cd RDK-Offline-TTS
mkdir build && cd build
cmake ..
make -j$(nproc)
```

#### 4. ROS2 启动
```bash
# 启动完整系统（LLM）
ros2 launch RDK-Offline-LLM llama_llm.launch.py

# 启动视觉语言模型
ros2 launch RDK-Offline-LLM llama_vlm.launch.py

# 启动 ASR 单独模块
ros2 run RDK-Offline-ASR speech_engine_node
```

### Web 界面开发

```bash
cd Cathay-web

# 安装依赖
npm install

# 开发模式（热重载）
npm run dev

# 生产构建
npm run build

# 预览生产构建
npm run preview
```

## 配置指南



### 音频配置

**VAD 参数** (`RDK-Offline-ASR/include/speech_engine.h`)
```cpp
float threshold = 0.5f;                  // VAD 触发阈值
float neg_threshold = 0.35f;             // VAD 停止阈值
int32_t min_speech_duration_ms = 250;    // 最小语音时长
int32_t max_speech_duration_ms = 5000;   // 最大语音时长
int32_t min_silence_duration_ms = 100;   // 最小静音时长
int32_t speech_pad_ms = 30;              // 语音填充时间
```

**支持的编解码器芯片**
- ES8311、ES8374、ES8388、ES8389
- AC101、ZL38063

### LLM 启动选项

`RDK-Offline-LLM/launch/` 提供多种启动配置：
- `llama_llm.launch.py` - 纯文本 LLM 推理
- `llama_vlm.launch.py` - 视觉语言模型（支持图像理解）
- `ali.launch.py` - 阿里云 LLM 集成
- `dosod.launch.py` - 目标检测与 OD 集成

## API 参考

### 语音引擎 API

```cpp
class speech_engine {
  // 初始化
  int Init(const std::string &cfg_path, 
           const std::string &wakeup_name,
           std::shared_ptr<std::vector<std::string>> v_cmd_word,
           AudioASRFunc asr_func, 
           AudioCmdDataFunc cmd_func);
  
  // 反初始化
  int DeInit();
  
  // 启动处理
  int Start();
  
  // 停止处理
  int Stop();
  
  // 发送音频数据
  void send_data(std::shared_ptr<std::vector<double>> data);
  
  // 处理线程
  void process(void);
};
```

### SenseVoice ASR 接口

```cpp
// 初始化上下文
struct sense_voice_context * sense_voice_small_init_from_file_with_params(
    const char * path_model, 
    struct sense_voice_context_params params);

// 批量 PCM 处理
int sense_voice_batch_pcmf(
    struct sense_voice_context *ctx, 
    const sense_voice_full_params &params,
    std::vector<std::vector<double>> &pcmf32,
    size_t max_batch_len=90000, 
    size_t max_batch_cnt=1,
    bool use_prefix=true, 
    bool use_itn=true);
```

### Silero VAD 接口

```cpp
// VAD 处理
double silero_vad_with_state(
    sense_voice_context &ctx,
    sense_voice_state &state,
    std::vector<float> &pcmf32,
    int n_processors);
```

### ESP32 应用 API

```cpp
// 获取应用实例
Application& app = Application::GetInstance();

// 音频控制
app.StartListening();
app.StopListening();
app.PlaySound("notification.ogg");

// 设备状态
app.SetDeviceState(DeviceState state);
DeviceState current = app.GetDeviceState();

// 系统控制
app.Reboot();
app.UpgradeFirmware(ota, "https://example.com/firmware.bin");

// 音频服务
AudioService& audio = app.GetAudioService();
bool has_voice = audio.IsVoiceDetected();
```

## Web 界面

基于 Vue 3 + TypeScript + Tailwind CSS 的现代前端界面：

```bash
cd Cathay-web

# 开发
npm run dev

# 构建
npm run build

# 类型检查
npm run type-check

# 代码检查
npm run lint
```

## 文档

### ESP32 模块文档
- [音频架构详解](Captain-Milo-ESP32/main/audio/README.md)
- [自定义板卡指南](Captain-Milo-ESP32/docs/custom-board.md)
- [MCP 协议说明](Captain-Milo-ESP32/docs/mcp-protocol.md)
- [WebSocket 集成](Captain-Milo-ESP32/docs/websocket.md)
- [MQTT/UDP 协议](Captain-Milo-ESP32/docs/mqtt-udp.md)

### RDK 模块文档
- [离线 ASR 详解](RDK-Offline-ASR/README.md)
- [离线 LLM 指南](RDK-Offline-LLM/README.md)
- [离线 TTS 指南](RDK-Offline-TTS/README.md)

## 开发工具

### 音频工具
```bash
# OGG 转换工具
python scripts/ogg_converter/xiaozhi_ogg_converter.py

# 声学调试工具
cd scripts/acoustic_check
python main.py

# 图像转 LVGL 格式
python scripts/Image_Converter/LVGLImage.py

# P3 音频转换
python scripts/p3_tools/convert_audio_to_p3.py
```

### 构建脚本
```bash
# 构建默认资源
python scripts/build_default_assets.py

# 生成语言文件
python scripts/gen_lang.py

# 发布构建
python scripts/release.py
```

## 技术亮点

- **完全离线语音交互** - 支持在设备端进行 ASR、LLM、TTS  
- **多语言支持** - 支持 25+ 语言  
- **低延迟处理** - 实时语音识别和响应  
- **OTA 升级** - 无縫固件更新机制  
- **隐私保护** - 全程隐私保护，无需依赖云服务  
- **模块化设计** - ASR、LLM、TTS 独立模块化  
- **ROS2 集成** - 基于 ROS2 的模块间通信  
- **视觉反馈** - LED 情感表辿和互动  
- **多传感器支持** - IMU、音频、环境传感器

## 贡献指南

欢迎贡献！请通过以下方式参与：

1. Fork 此仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request


## 团队

**E094 Team - Cathay Hackathon**  


