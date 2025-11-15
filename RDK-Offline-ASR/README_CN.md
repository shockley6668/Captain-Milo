# RDK 离线语音识别（ASR）模块

## 项目概述

RDK-Offline-ASR 是 Captain Milo 项目的离线语音识别模块，基于 ROS2 框架实现。集成了 SenseVoice 多语言语音识别引擎和 Silero VAD 语音活动检测，提供高效的本地语音识别能力。

## 核心功能

### 语音识别 (ASR)
- **SenseVoice 引擎**：多语言离线语音识别
- **25+ 语言支持**：中文、英文、日文、韩文等
- **实时处理**：低延迟的语音转文本
- **离线推理**：完全本地化，无网络依赖

### 语音活动检测 (VAD)
- **Silero VAD 模块**：精准的语音端点检测
- **实时判断**：区分语音和静音段
- **低误报**：高精度的语音识别

### 音频处理
- **多源输入**：支持不同的音频接口
- **格式转换**：自动处理多种音频格式
- **预处理**：噪声抑制、增益控制

## 项目结构

```
RDK-Offline-ASR/
├── src/
│   ├── speech_engine.cpp         # 语音引擎核心实现
│   └── hb_audio_capture.cpp      # 音频捕获模块
├── include/
│   ├── speech_engine.h            # 语音引擎接口
│   └── sensevoice/
│       ├── sense-voice.h          # SenseVoice ASR 核心
│       ├── sense-voice-encoder.h  # 编码器
│       ├── sense-voice-decoder.h  # 解码器
│       ├── silero-vad.h           # VAD 模块
│       └── common.h               # 公共定义
├── config/
│   └── asr_config.yaml            # ASR 配置文件
├── launch/
│   ├── asr_node.launch.py         # ASR ROS2 启动文件
│   └── asr_with_vad.launch.py     # 带 VAD 的启动文件
├── SenseVoiceGGUF/
│   ├── sense-voice-zh.gguf        # 中文模型
│   ├── sense-voice-en.gguf        # 英文模型
│   └── sense-voice-multilingual.gguf  # 多语言模型
├── CMakeLists.txt
└── package.xml
```

## 快速开始

### 编译

```bash
# 进入模块目录
cd RDK-Offline-ASR

# 创建构建目录
mkdir build && cd build

# 生成构建文件
cmake ..

# 编译
make -j$(nproc)

# 安装
make install
```

### 运行

#### 使用 ROS2 启动

```bash
# 启动 ASR 节点
ros2 launch RDK-Offline-ASR asr_node.launch.py

# 启动带 VAD 的 ASR
ros2 launch RDK-Offline-ASR asr_with_vad.launch.py
```

#### 订阅 ROS2 话题

```bash
# 监听 ASR 识别结果
ros2 topic echo /asr/result

# 监听 VAD 状态
ros2 topic echo /vad/state
```

## API 参考

### 语音引擎 API

```cpp
#include "speech_engine.h"

// 语音引擎类
class speech_engine {
public:
  // 初始化引擎
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
#include "sensevoice/sense-voice.h"

// 初始化上下文
struct sense_voice_context * sense_voice_small_init_from_file_with_params(
    const char * path_model, 
    struct sense_voice_context_params params);

// 编码器处理
bool sense_voice_encode_internal(
    sense_voice_context &ctx,
    sense_voice_state &state,
    const int n_threads);

// 解码器处理
bool sense_voice_decode_internal(
    sense_voice_context &ctx,
    sense_voice_state &state,
    const int n_threads);

// 批量 PCM 处理
int sense_voice_batch_pcmf(
    struct sense_voice_context *ctx, 
    const sense_voice_full_params &params,
    std::vector<std::vector<double>> &pcmf32,
    size_t max_batch_len=90000, 
    size_t max_batch_cnt=1,
    bool use_prefix=true, 
    bool use_itn=true);

// 释放资源
void sense_voice_free(struct sense_voice_context *ctx);
```

### Silero VAD 接口

```cpp
#include "sensevoice/silero-vad.h"

// VAD 编码处理
bool silero_vad_encode_internal(
    sense_voice_context &ctx,
    sense_voice_state &state,
    std::vector<float> chunk,
    const int n_threads,
    float &speech_prob);

// 带状态的 VAD 处理
double silero_vad_with_state(
    sense_voice_context &ctx,
    sense_voice_state &state,
    std::vector<float> &pcmf32,
    int n_processors);

// 获取 VAD 状态
int silero_vad_get_state(sense_voice_context &ctx);

// 重置 VAD 状态
void silero_vad_reset_state(sense_voice_context &ctx);
```

## 配置指南

### ASR 配置 (config/asr_config.yaml)

```yaml
# SenseVoice 模型配置
sensevoice:
  model_path: "./SenseVoiceGGUF/sense-voice-multilingual.gguf"
  language: "zh"  # 主要语言: zh, en, ja, ko, etc.
  num_threads: 4
  use_gpu: false

# VAD 配置
vad:
  enabled: true
  threshold: 0.5           # 触发阈值
  neg_threshold: 0.35      # 停止阈值
  min_speech_duration_ms: 250      # 最小语音时长
  max_speech_duration_ms: 5000     # 最大语音时长
  min_silence_duration_ms: 100     # 最小静音时长
  speech_pad_ms: 30                # 语音填充时间

# 音频配置
audio:
  sample_rate: 16000
  channels: 1
  chunk_size: 512
  format: "pcm_16bit"

# ROS2 话题配置
ros2:
  asr_result_topic: "/asr/result"
  vad_state_topic: "/vad/state"
  audio_input_topic: "/audio/data"
```

### 模型配置

支持的模型及语言：

| 模型文件 | 语言 | 大小 | 推荐设备 |
|---------|------|------|---------|
| sense-voice-zh.gguf | 中文 | ~200MB | RDK X5+ |
| sense-voice-en.gguf | 英文 | ~200MB | RDK X5+ |
| sense-voice-multilingual.gguf | 25+ 语言 | ~300MB | RDK X5+ |

### VAD 参数详解

- **threshold (0.5)**：VAD 触发阈值，范围 0-1，越低越容易触发
- **neg_threshold (0.35)**：VAD 停止阈值，应低于 threshold
- **min_speech_duration_ms (250)**：最小语音时长（毫秒），过短会被忽略
- **max_speech_duration_ms (5000)**：最大语音时长，防止过长输入
- **min_silence_duration_ms (100)**：最小静音时长，触发语音结束
- **speech_pad_ms (30)**：语音填充时间，确保完整捕获

## 使用示例

### 基础使用

```cpp
#include "speech_engine.h"
#include <iostream>

// ASR 结果回调
void asr_callback(const std::string& result) {
    std::cout << "ASR Result: " << result << std::endl;
}

// 命令识别回调
void cmd_callback(const std::string& cmd) {
    std::cout << "Command: " << cmd << std::endl;
}

int main() {
    // 初始化引擎
    speech_engine engine;
    auto cmd_words = std::make_shared<std::vector<std::string>>(
        std::vector<std::string>{"开始", "停止", "暂停"}
    );
    
    int ret = engine.Init("./config/asr_config.yaml", 
                         "唤醒词",
                         cmd_words,
                         asr_callback,
                         cmd_callback);
    
    if (ret != 0) {
        std::cerr << "Failed to init engine" << std::endl;
        return -1;
    }
    
    // 启动处理
    engine.Start();
    
    // 发送音频数据
    auto audio_data = std::make_shared<std::vector<double>>(16000);
    engine.send_data(audio_data);
    
    // 停止处理
    engine.Stop();
    engine.DeInit();
    
    return 0;
}
```

### ROS2 使用

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "speech_engine.h"

class ASRNode : public rclcpp::Node {
public:
    ASRNode() : Node("asr_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/asr/result", 10);
        
        // 初始化引擎
        engine_.Init("./config/asr_config.yaml",
                     "唤醒词",
                     nullptr,
                     [this](const std::string& result) { 
                         this->publish_result(result); 
                     },
                     nullptr);
        
        engine_.Start();
    }
    
private:
    void publish_result(const std::string& result) {
        auto msg = std_msgs::msg::String();
        msg.data = result;
        publisher_->publish(msg);
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    speech_engine engine_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ASRNode>());
    rclcpp::shutdown();
    return 0;
}
```

## 性能指标

### 延迟性能
- **实时因子 (RTF)**：< 0.3（实时处理速度）
- **首字延迟**：~200ms
- **总识别延迟**：~500-800ms

### 精度指标
- **字错误率 (CER)**：< 10%（中文）
- **词错误率 (WER)**：< 15%（英文）
- **VAD 准确度**：> 95%

### 资源占用
- **内存**：~300MB（仅模型） + ~100MB（运行时）
- **CPU**：2-4 核心（RDK X5）
- **功耗**：~5-10W（离线推理）

## 故障排除

### 常见问题

**Q: ASR 识别准确率低**
- A: 检查音频质量，调整 VAD 参数，尝试不同的模型

**Q: 处理延迟高**
- A: 增加线程数，使用 GPU 加速，检查 CPU 使用率

**Q: VAD 误报多**
- A: 调高 threshold 和 neg_threshold，增加 min_silence_duration_ms

**Q: 模型加载失败**
- A: 检查模型文件路径，确保模型文件完整且格式正确

### 调试模式

```bash
# 启用调试输出
export ROS_LOG_LEVEL=DEBUG
ros2 launch RDK-Offline-ASR asr_node.launch.py

# 查看详细日志
ros2 topic echo /asr/result --full-length
```

## 依赖项

- ROS2 (Humble 或更新版本)
- C++ 17 或更新版本
- GGML 库

## 相关资源

- [SenseVoice GitHub](https://github.com/alibaba-damo-academy/SenseVoice)
- [Silero VAD GitHub](https://github.com/snakers4/silero-vad)
- [ROS2 文档](https://docs.ros.org/)
- [GGML 文档](https://github.com/ggerganov/ggml)

