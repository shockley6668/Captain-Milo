# RDK Offline Text-to-Speech (TTS) Module

[English](#english) | [中文](README_CN.md)

---

<a name="english"></a>

## Project Overview

RDK-Offline-TTS is the offline text-to-speech module for the Captain Milo project, implemented based on the ROS2 framework and high-quality TTS engine. It provides natural, fluent local speech synthesis capabilities, supporting multiple languages, voice options, and emotional expression.

## Core Features

### Text-to-Speech (TTS)
- **High-quality Synthesis**: Natural, fluent voice output
- **Multi-language Support**: 25+ languages
- **Multiple Voice Options**: Multiple pre-trained voice libraries
- **Offline Inference**: Fully localized, no network dependency

### Voice Control
- **Pre-trained Voices**: Chinese female/male voices, child voices, etc.
- **Voice Cloning**: Learn new voices from samples
- **Emotional Expression**: Happy, sad, surprised, and other emotions
- **Speed Control**: Flexible playback speed adjustment

### Audio Processing
- **Output Formats**: WAV, MP3, Opus, and more
- **Quality Selection**: 8kHz to 44.1kHz
- **Real-time Synthesis**: Low-latency streaming output
- **Audio Enhancement**: Normalization, compression, etc.

## Project Structure

```
RDK-Offline-TTS/
├── src/
│   ├── tts_node.cpp               # TTS ROS2 node
│   ├── tts_engine.cpp             # TTS inference engine
│   └── audio_processor.cpp        # Audio processing module
├── include/
│   ├── tts_engine.h               # TTS engine interface
│   ├── audio_processor.h          # Audio processing interface
│   └── voice_config.h             # Voice configuration
├── launch/
│   ├── tts_node.launch.py         # TTS launch file
│   └── tts_with_audio.launch.py   # Launch with audio processing
├── wetts/
│   ├── vits.h                     # VITS TTS engine
│   ├── glow_tts.h                 # Glow-TTS engine
│   └── mel_processor.h            # Mel spectrogram processing
├── voices/
│   ├── zh_female_1.pth            # Chinese female voice model 1
│   ├── zh_male_1.pth              # Chinese male voice model 1
│   ├── en_female_1.pth            # English female voice model 1
│   └── multilingual_1.pth         # Multilingual model 1
├── CMakeLists.txt
└── package.xml
```

## Quick Start

### Build

```bash
# Navigate to module directory
cd RDK-Offline-TTS

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

#### Launch TTS Node

```bash
# Launch TTS node
ros2 launch RDK-Offline-TTS tts_node.launch.py

# Send synthesis request
ros2 service call /tts/synthesize "text: 'Hello, I am Captain Milo'"

# Listen to output
ros2 topic echo /tts/audio
```

#### Use Specific Voice

```bash
# Launch with specified voice
ros2 launch RDK-Offline-TTS tts_node.launch.py voice_name:=zh_female_1

# Set speech rate
ros2 launch RDK-Offline-TTS tts_node.launch.py speech_rate:=1.2

# Set pitch
ros2 launch RDK-Offline-TTS tts_node.launch.py pitch:=1.0
```

## API Reference

### TTS Engine API

```cpp
#include "tts_engine.h"

class TTSEngine {
public:
    // Initialize
    int Init(const std::string &model_path,
             const TTSConfig &config);
    
    // Deinitialize
    void Deinit();
    
    // Synthesize text
    std::vector<float> Synthesize(
        const std::string &text,
        const SynthesisParams &params);
    
    // Stream synthesis (callback style)
    void SynthesizeStream(
        const std::string &text,
        std::function<void(const std::vector<float>&)> callback,
        const SynthesisParams &params);
    
    // Set voice
    void SetVoice(const std::string &voice_name);
    
    // Get available voices
    std::vector<std::string> GetAvailableVoices() const;
    
    // Save audio file
    int SaveAudioFile(const std::string &filename,
                     const std::vector<float> &audio,
                     const AudioFormat &format);
};

// Synthesis parameters
struct SynthesisParams {
    float speed = 1.0f;             // Speech speed (0.5-2.0)
    float pitch = 1.0f;             // Pitch (0.5-2.0)
    float energy = 1.0f;            // Energy (0.5-2.0)
    int sample_rate = 22050;        // Sample rate
    std::string emotion = "neutral"; // Emotion (neutral, happy, sad, etc.)
    std::string language = "en";    // Language code
};

// TTS configuration
struct TTSConfig {
    int sample_rate = 22050;        // Sample rate
    int n_mel_channels = 80;        // Mel spectrogram channels
    bool use_gpu = false;           // Use GPU
    int n_threads = 4;              // Number of CPU threads
    float phoneme_threshold = 0.5f; // Phoneme threshold
};

// Audio format
struct AudioFormat {
    enum Format { WAV, MP3, OGG_OPUS };
    Format format = WAV;
    int bit_depth = 16;
    int channels = 1;
    int quality = 192;              // for MP3/OGG
};
```

### Audio Processing API

```cpp
#include "audio_processor.h"

class AudioProcessor {
public:
    // Normalize audio
    std::vector<float> Normalize(const std::vector<float> &audio);
    
    // Dynamic range compression
    std::vector<float> Compress(const std::vector<float> &audio,
                               float threshold = -20.0f,
                               float ratio = 4.0f);
    
    // Limiter
    std::vector<float> Limit(const std::vector<float> &audio,
                            float threshold = -3.0f);
    
    // Volume adjustment
    std::vector<float> AdjustVolume(const std::vector<float> &audio,
                                   float db);
    
    // Fade in/out
    std::vector<float> FadeInOut(const std::vector<float> &audio,
                                int fade_samples);
};
```

### ROS2 Service Interface

```cpp
// Text synthesis service
srv Synthesize:
  string text
  float speed
  float pitch
  string voice_name
  ---
  sensor_msgs/msg/Audio audio

// Stream synthesis
srv SynthesizeStream:
  string text
  float speed
  float pitch
  ---
  bool success

// Save to file
srv SaveAudio:
  string text
  string filename
  string format
  ---
  bool success
```

## Configuration Guide

### TTS Configuration (launch/tts_node.launch.py)

```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='./voices/zh_female_1.pth',
            description='Path to TTS model'
        ),
        DeclareLaunchArgument(
            'voice_name',
            default_value='zh_female_1',
            description='Voice name'
        ),
        DeclareLaunchArgument(
            'speech_rate',
            default_value='1.0',
            description='Speech rate (0.5-2.0)'
        ),
        DeclareLaunchArgument(
            'pitch',
            default_value='1.0',
            description='Pitch (0.5-2.0)'
        ),
        DeclareLaunchArgument(
            'sample_rate',
            default_value='22050',
            description='Output sample rate'
        ),
    ])
```

### Voice Configuration

Available voices:

| Voice Name | Type | Language | Sample |
|-----------|------|----------|--------|
| zh_female_1 | Female | Chinese | Standard Mandarin |
| zh_female_2 | Female | Chinese | Gentle tone |
| zh_male_1 | Male | Chinese | Standard Mandarin |
| zh_male_2 | Male | Chinese | Deep tone |
| zh_child | Child | Chinese | Child voice |
| en_female_1 | Female | English | American English |
| en_male_1 | Male | English | American English |
| ja_female_1 | Female | Japanese | Standard Japanese |
| multilingual_1 | Multilingual | Multilingual | Universal voice |

### Emotion Expression Configuration

Supported emotion types:

- **neutral**: Neutral tone
- **happy**: Happy, excited
- **sad**: Sad, depressed
- **angry**: Angry, serious
- **surprise**: Surprised, amazed
- **fear**: Fearful, worried

## Usage Examples

### Basic Text Synthesis

```cpp
#include "tts_engine.h"
#include <iostream>

int main() {
    TTSEngine engine;
    
    // Initialize
    TTSConfig config;
    config.sample_rate = 22050;
    
    engine.Init("./voices/zh_female_1.pth", config);
    
    // Synthesize text
    std::string text = "Welcome to use Captain Milo companion";
    SynthesisParams params;
    params.speed = 1.0f;
    params.pitch = 1.0f;
    params.emotion = "happy";
    
    std::vector<float> audio = engine.Synthesize(text, params);
    
    // Save file
    AudioFormat format;
    format.format = AudioFormat::WAV;
    engine.SaveAudioFile("output.wav", audio, format);
    
    engine.Deinit();
    return 0;
}
```

### Stream Synthesis

```cpp
// Stream synthesis with real-time output
engine.SynthesizeStream(
    "Testing stream synthesis",
    [](const std::vector<float>& chunk) {
        // Process audio chunks in real-time
        std::cout << "Received " << chunk.size() << " samples" << std::endl;
    },
    params
);
```

### Audio Processing

```cpp
#include "audio_processor.h"

int main() {
    TTSEngine engine;
    AudioProcessor processor;
    
    // Synthesize audio
    auto audio = engine.Synthesize("Hello World", params);
    
    // Process audio
    audio = processor.Normalize(audio);
    audio = processor.Compress(audio);
    audio = processor.AdjustVolume(audio, -3.0f);
    audio = processor.FadeInOut(audio, 2205);  // 100ms fade
    
    // Save processed audio
    engine.SaveAudioFile("processed.wav", audio, format);
    
    return 0;
}
```

### ROS2 Integration

```cpp
#include "rclcpp/rclcpp.hpp"
#include "tts_engine.h"

class TTSNode : public rclcpp::Node {
public:
    TTSNode() : Node("tts_node") {
        engine_.Init("./voices/zh_female_1.pth", config_);
        
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Audio>(
            "/tts/audio", 10
        );
        
        // Create service
        service_ = this->create_service<tts_msgs::srv::Synthesize>(
            "/tts/synthesize",
            [this](const std::shared_ptr<tts_msgs::srv::Synthesize::Request> req,
                   std::shared_ptr<tts_msgs::srv::Synthesize::Response> res) {
                auto audio = engine_.Synthesize(req->text, params_);
                res->success = true;
            }
        );
    }
    
private:
    TTSEngine engine_;
    TTSConfig config_;
    SynthesisParams params_;
    rclcpp::Publisher<sensor_msgs::msg::Audio>::SharedPtr publisher_;
    rclcpp::Service<tts_msgs::srv::Synthesize>::SharedPtr service_;
};
```

### Multilingual Synthesis

```cpp
// Chinese
SynthesisParams params_zh;
params_zh.language = "zh";
auto audio_zh = engine.Synthesize("你好", params_zh);

// English
SynthesisParams params_en;
params_en.language = "en";
auto audio_en = engine.Synthesize("Hello", params_en);

// Japanese
SynthesisParams params_ja;
params_ja.language = "ja";
auto audio_ja = engine.Synthesize("こんにちは", params_ja);
```

## Performance Metrics

### Synthesis Speed
- **Real-time Factor (RTF)**: < 0.5 (fast synthesis)
- **First token latency**: ~100-200ms
- **Average speed**: 1000+ characters/hour

### Audio Quality Metrics
- **MOS Score**: 4.0+ (Chinese)
- **Sample Rate**: 22.05kHz - 44.1kHz
- **Bit Depth**: 16-bit PCM

### Resource Usage
- **Memory**: ~500MB (model) + ~100MB (runtime)
- **CPU**: 1-2 cores (RDK X5)
- **Power**: ~2-5W (during synthesis)

## Troubleshooting

### Common Issues

**Q: Slow synthesis speed**
- A: Reduce input text length, use stream synthesis, choose faster model

**Q: Poor audio quality**
- A: Check sample rate settings, adjust energy parameter, use better voice

**Q: Unnatural voice**
- A: Try different voices, adjust speed and pitch, check text encoding

**Q: Limited language support**
- A: Use multilingual model, check language code is correct

### Debug

```bash
# Enable debug logging
export ROS_LOG_LEVEL=DEBUG
ros2 launch RDK-Offline-TTS tts_node.launch.py

# Test synthesis
ros2 service call /tts/synthesize "text: 'test'"

# Listen to audio output
ros2 topic echo /tts/audio
```

## Dependencies

- ROS2 (Humble or newer)
- C++ 17 or newer
- PyTorch (for model inference)
- SoundFile (audio processing)
- Optional: CUDA (GPU acceleration)

## Related Resources

- [VITS GitHub](https://github.com/jaywalnut310/vits)
- [Glow-TTS GitHub](https://github.com/jaywalnut310/glow-tts)
- [ROS2 Documentation](https://docs.ros.org/)
- [PyTorch Documentation](https://pytorch.org/docs/)

## Contributing

Issues and Pull Requests are welcome!

## License

MIT License
