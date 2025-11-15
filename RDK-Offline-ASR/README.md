# RDK Offline Speech Recognition (ASR) Module

[English](#english) | [中文](README_CN.md)

---

<a name="english"></a>

## Project Overview

RDK-Offline-ASR is the offline speech recognition module for the Captain Milo project, implemented based on the ROS2 framework. It integrates the SenseVoice multi-language speech recognition engine and Silero VAD speech activity detection, providing efficient local speech recognition capabilities.

## Core Features

### Speech Recognition (ASR)
- **SenseVoice Engine**: Multi-language offline speech recognition
- **25+ Language Support**: Chinese, English, Japanese, Korean, and more
- **Real-time Processing**: Low-latency speech-to-text conversion
- **Offline Inference**: Fully localized, no network dependency

### Voice Activity Detection (VAD)
- **Silero VAD Module**: Precise speech endpoint detection
- **Real-time Judgment**: Distinguishes speech from silence
- **Low False Positives**: High-precision speech recognition

### Audio Processing
- **Multi-source Input**: Supports different audio interfaces
- **Format Conversion**: Automatically handles multiple audio formats
- **Preprocessing**: Noise suppression, gain control

## Project Structure

```
RDK-Offline-ASR/
├── src/
│   ├── speech_engine.cpp         # Speech engine core implementation
│   └── hb_audio_capture.cpp      # Audio capture module
├── include/
│   ├── speech_engine.h            # Speech engine interface
│   └── sensevoice/
│       ├── sense-voice.h          # SenseVoice ASR core
│       ├── sense-voice-encoder.h  # Encoder
│       ├── sense-voice-decoder.h  # Decoder
│       ├── silero-vad.h           # VAD module
│       └── common.h               # Common definitions
├── config/
│   └── asr_config.yaml            # ASR configuration file
├── launch/
│   ├── asr_node.launch.py         # ASR ROS2 launch file
│   └── asr_with_vad.launch.py     # Launch file with VAD
├── SenseVoiceGGUF/
│   ├── sense-voice-zh.gguf        # Chinese model
│   ├── sense-voice-en.gguf        # English model
│   └── sense-voice-multilingual.gguf  # Multilingual model
├── CMakeLists.txt
└── package.xml
```

## Quick Start

### Build

```bash
# Navigate to module directory
cd RDK-Offline-ASR

# Create build directory
mkdir build && cd build

# Generate build files
cmake ..

# Compile
make -j$(nproc)

# Install
make install
```

### Run

#### Launch ASR Node using ROS2

```bash
# Launch ASR node
ros2 launch RDK-Offline-ASR asr_node.launch.py

# Launch ASR with VAD
ros2 launch RDK-Offline-ASR asr_with_vad.launch.py
```

#### Subscribe to ROS2 Topics

```bash
# Listen to ASR recognition results
ros2 topic echo /asr/result

# Listen to VAD state
ros2 topic echo /vad/state
```

## API Reference

### Speech Engine API

```cpp
#include "speech_engine.h"

// Speech engine class
class speech_engine {
public:
  // Initialize engine
  int Init(const std::string &cfg_path, 
           const std::string &wakeup_name,
           std::shared_ptr<std::vector<std::string>> v_cmd_word,
           AudioASRFunc asr_func, 
           AudioCmdDataFunc cmd_func);
  
  // Deinitialize
  int DeInit();
  
  // Start processing
  int Start();
  
  // Stop processing
  int Stop();
  
  // Send audio data
  void send_data(std::shared_ptr<std::vector<double>> data);
  
  // Processing thread
  void process(void);
};
```

### SenseVoice ASR Interface

```cpp
#include "sensevoice/sense-voice.h"

// Initialize context
struct sense_voice_context * sense_voice_small_init_from_file_with_params(
    const char * path_model, 
    struct sense_voice_context_params params);

// Encoder processing
bool sense_voice_encode_internal(
    sense_voice_context &ctx,
    sense_voice_state &state,
    const int n_threads);

// Decoder processing
bool sense_voice_decode_internal(
    sense_voice_context &ctx,
    sense_voice_state &state,
    const int n_threads);

// Batch PCM processing
int sense_voice_batch_pcmf(
    struct sense_voice_context *ctx, 
    const sense_voice_full_params &params,
    std::vector<std::vector<double>> &pcmf32,
    size_t max_batch_len=90000, 
    size_t max_batch_cnt=1,
    bool use_prefix=true, 
    bool use_itn=true);

// Free resources
void sense_voice_free(struct sense_voice_context *ctx);
```

### Silero VAD Interface

```cpp
#include "sensevoice/silero-vad.h"

// VAD encoding processing
bool silero_vad_encode_internal(
    sense_voice_context &ctx,
    sense_voice_state &state,
    std::vector<float> chunk,
    const int n_threads,
    float &speech_prob);

// VAD processing with state
double silero_vad_with_state(
    sense_voice_context &ctx,
    sense_voice_state &state,
    std::vector<float> &pcmf32,
    int n_processors);

// Get VAD state
int silero_vad_get_state(sense_voice_context &ctx);

// Reset VAD state
void silero_vad_reset_state(sense_voice_context &ctx);
```

## Configuration Guide

### ASR Configuration (config/asr_config.yaml)

```yaml
# SenseVoice model configuration
sensevoice:
  model_path: "./SenseVoiceGGUF/sense-voice-multilingual.gguf"
  language: "zh"  # Primary language: zh, en, ja, ko, etc.
  num_threads: 4
  use_gpu: false

# VAD configuration
vad:
  enabled: true
  threshold: 0.5           # Trigger threshold
  neg_threshold: 0.35      # Stop threshold
  min_speech_duration_ms: 250      # Minimum speech duration
  max_speech_duration_ms: 5000     # Maximum speech duration
  min_silence_duration_ms: 100     # Minimum silence duration
  speech_pad_ms: 30                # Speech padding time

# Audio configuration
audio:
  sample_rate: 16000
  channels: 1
  chunk_size: 512
  format: "pcm_16bit"

# ROS2 topic configuration
ros2:
  asr_result_topic: "/asr/result"
  vad_state_topic: "/vad/state"
  audio_input_topic: "/audio/data"
```

### Supported Models

| Model File | Language | Size | Recommended Device |
|-----------|----------|------|-------------------|
| sense-voice-zh.gguf | Chinese | ~200MB | RDK X5+ |
| sense-voice-en.gguf | English | ~200MB | RDK X5+ |
| sense-voice-multilingual.gguf | 25+ languages | ~300MB | RDK X5+ |

### VAD Parameter Explanation

- **threshold (0.5)**: VAD trigger threshold, range 0-1, lower values trigger more easily
- **neg_threshold (0.35)**: VAD stop threshold, should be lower than threshold
- **min_speech_duration_ms (250)**: Minimum speech duration (milliseconds), shorter inputs are ignored
- **max_speech_duration_ms (5000)**: Maximum speech duration, prevents overly long inputs
- **min_silence_duration_ms (100)**: Minimum silence duration, triggers speech end
- **speech_pad_ms (30)**: Speech padding time, ensures complete capture

## Usage Examples

### Basic Usage

```cpp
#include "speech_engine.h"
#include <iostream>

// ASR result callback
void asr_callback(const std::string& result) {
    std::cout << "ASR Result: " << result << std::endl;
}

// Command recognition callback
void cmd_callback(const std::string& cmd) {
    std::cout << "Command: " << cmd << std::endl;
}

int main() {
    // Initialize engine
    speech_engine engine;
    auto cmd_words = std::make_shared<std::vector<std::string>>(
        std::vector<std::string>{"start", "stop", "pause"}
    );
    
    int ret = engine.Init("./config/asr_config.yaml", 
                         "wake_word",
                         cmd_words,
                         asr_callback,
                         cmd_callback);
    
    if (ret != 0) {
        std::cerr << "Failed to init engine" << std::endl;
        return -1;
    }
    
    // Start processing
    engine.Start();
    
    // Send audio data
    auto audio_data = std::make_shared<std::vector<double>>(16000);
    engine.send_data(audio_data);
    
    // Stop processing
    engine.Stop();
    engine.DeInit();
    
    return 0;
}
```

### ROS2 Usage

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "speech_engine.h"

class ASRNode : public rclcpp::Node {
public:
    ASRNode() : Node("asr_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/asr/result", 10);
        
        // Initialize engine
        engine_.Init("./config/asr_config.yaml",
                     "wake_word",
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

## Performance Metrics

### Latency Performance
- **Real-time Factor (RTF)**: < 0.3 (real-time processing speed)
- **First token latency**: ~200ms
- **Total recognition latency**: ~500-800ms

### Accuracy Metrics
- **Character Error Rate (CER)**: < 10% (Chinese)
- **Word Error Rate (WER)**: < 15% (English)
- **VAD Accuracy**: > 95%

### Resource Usage
- **Memory**: ~300MB (model only) + ~100MB (runtime)
- **CPU**: 2-4 cores (RDK X5)
- **Power**: ~5-10W (offline inference)

## Troubleshooting

### Common Issues

**Q: Low ASR recognition accuracy**
- A: Check audio quality, adjust VAD parameters, try different models

**Q: High processing latency**
- A: Increase thread count, use GPU acceleration, check CPU usage

**Q: High VAD false positives**
- A: Increase threshold and neg_threshold, increase min_silence_duration_ms

**Q: Model loading fails**
- A: Check model file path, ensure model file is complete and in correct format

### Debug Mode

```bash
# Enable debug output
export ROS_LOG_LEVEL=DEBUG
ros2 launch RDK-Offline-ASR asr_node.launch.py

# View detailed logs
ros2 topic echo /asr/result --full-length
```

## Dependencies

- ROS2 (Humble or newer)
- C++ 17 or newer
- GGML library

## Related Resources

- [SenseVoice GitHub](https://github.com/alibaba-damo-academy/SenseVoice)
- [Silero VAD GitHub](https://github.com/snakers4/silero-vad)
- [ROS2 Documentation](https://docs.ros.org/)
- [GGML Documentation](https://github.com/ggerganov/ggml)
