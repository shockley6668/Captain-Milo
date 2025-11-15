# RDK Offline Large Language Model (LLM) Module

[English](#english) | [中文](README_CN.md)

---

<a name="english"></a>

## Project Overview

RDK-Offline-LLM is the offline large language model inference module for the Captain Milo project, implemented based on the ROS2 framework and llama.cpp. It provides lightweight, efficient local LLM inference capabilities, supporting multiple models and vision-language understanding.

## Core Features

### Text Generation (LLM)
- **llama.cpp Integration**: Efficient LLM inference framework
- **Multi-model Support**: GPT, Qwen, Llama, and more
- **Quantization Support**: INT8, INT4 and other quantized models
- **Offline Inference**: Fully localized, no network dependency

### Vision Language Model (VLM)
- **Image Understanding**: Supports image input and analysis
- **InternVL Integration**: Advanced vision-language capabilities
- **Multimodal**: Text + image + visual reasoning

### Dialogue Management
- **Context Preservation**: Multi-turn conversation support
- **Prompt Engineering**: Flexible prompt management
- **Streaming Output**: Real-time generation result streaming

## Project Structure

```
RDK-Offline-LLM/
├── src/
│   ├── llm_node.cpp               # LLM ROS2 node
│   ├── vlm_node.cpp               # VLM ROS2 node
│   └── llm_engine.cpp             # LLM inference engine
├── include/
│   ├── llm_engine.h               # LLM engine interface
│   └── vlm_engine.h               # VLM engine interface
├── launch/
│   ├── llama_llm.launch.py        # Pure text LLM launch
│   ├── llama_vlm.launch.py        # Vision language model launch
│   ├── ali.launch.py              # Alibaba Cloud integration launch
│   └── dosod.launch.py            # Object detection integration launch
├── llama.cpp/
│   ├── common.h                   # Common utilities
│   ├── llama.h                    # llama.cpp core
│   └── llm.cpp                    # LLM implementation
├── models/
│   ├── qwen-1.8b.gguf             # Qwen 1.8B model
│   ├── internvl-1b.gguf           # InternVL 1B model
│   └── llama2-7b-q4.gguf          # Llama 2 7B Q4 model
├── CMakeLists.txt
└── package.xml
```

## Quick Start

### Build

```bash
# Navigate to module directory
cd RDK-Offline-LLM

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

#### Launch Pure Text LLM

```bash
# Launch LLM node
ros2 launch RDK-Offline-LLM llama_llm.launch.py

# Send inference request
ros2 service call /llm/generate_text "prompt: 'Hello, introduce yourself'"

# Listen to output
ros2 topic echo /llm/response
```

#### Launch Vision Language Model

```bash
# Launch VLM node
ros2 launch RDK-Offline-LLM llama_vlm.launch.py

# Send request with image
ros2 service call /vlm/analyze_image "image_path: '/path/to/image.jpg', question: 'What is in the image?'"

# Listen to analysis result
ros2 topic echo /vlm/analysis
```

## API Reference

### LLM Engine API

```cpp
#include "llm_engine.h"

class LLMEngine {
public:
    // Initialize
    int Init(const std::string &model_path,
             const LLMConfig &config);
    
    // Deinitialize
    void Deinit();
    
    // Generate text
    std::string Generate(const std::string &prompt,
                        const GenerateParams &params);
    
    // Stream generation (callback style)
    void GenerateStream(const std::string &prompt,
                       std::function<void(const std::string&)> callback,
                       const GenerateParams &params);
    
    // Reset context
    void ResetContext();
    
    // Set system prompt
    void SetSystemPrompt(const std::string &prompt);
};

// Generation parameters
struct GenerateParams {
    int max_tokens = 256;
    float temperature = 0.7f;
    float top_p = 0.9f;
    int top_k = 40;
    float frequency_penalty = 0.0f;
    float presence_penalty = 0.0f;
    std::vector<std::string> stop_sequences;
};

// LLM configuration
struct LLMConfig {
    int n_gpu_layers = 0;           // Number of GPU layers
    int n_threads = 4;              // Number of CPU threads
    int context_size = 2048;        // Context size
    bool use_memory_mapped = true;  // Use memory mapping
    int batch_size = 512;           // Batch size
};
```

### VLM Engine API

```cpp
#include "vlm_engine.h"

class VLMEngine {
public:
    // Initialize
    int Init(const std::string &model_path,
             const VLMConfig &config);
    
    // Deinitialize
    void Deinit();
    
    // Analyze image
    std::string AnalyzeImage(const cv::Mat &image,
                            const std::string &question);
    
    // Load image from file
    std::string AnalyzeImageFile(const std::string &image_path,
                                const std::string &question);
    
    // Analyze multiple images
    std::string AnalyzeMultipleImages(
        const std::vector<cv::Mat> &images,
        const std::string &question);
    
    // Get image understanding results
    std::vector<std::string> GetImageCaption(const cv::Mat &image);
};

// VLM configuration
struct VLMConfig {
    int image_size = 448;           // Input image size
    bool use_gpu = false;           // Use GPU
    int n_threads = 4;              // Number of CPU threads
    int max_tokens = 512;           // Maximum generation tokens
};
```

### ROS2 Service Interface

```cpp
// Text generation service
srv GenerateText:
  ---
  string response
  
// Stream text generation
srv GenerateTextStream:
  ---
  bool success

// Image analysis service
srv AnalyzeImage:
  string image_path
  string question
  ---
  string analysis
  
// Image caption generation
srv GenerateCaption:
  string image_path
  ---
  string caption
```

## Configuration Guide

### LLM Configuration (launch/llama_llm.launch.py)

```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='./models/qwen-1.8b.gguf',
            description='Path to LLM model'
        ),
        DeclareLaunchArgument(
            'n_gpu_layers',
            default_value='0',
            description='Number of layers to offload to GPU'
        ),
        DeclareLaunchArgument(
            'n_threads',
            default_value='4',
            description='Number of CPU threads'
        ),
        DeclareLaunchArgument(
            'context_size',
            default_value='2048',
            description='Context size'
        ),
    ])
```

### Model Selection

Supported models and specifications:

| Model | Size | Parameters | Recommended Device | Inference Speed |
|-------|------|-----------|-------------------|-----------------|
| Qwen-1.8B | ~1.5GB | 1.8B | RDK X5 | Fast |
| Qwen-7B-Q4 | ~4GB | 7B | RDK X5 | Medium |
| Llama-2-7B-Q4 | ~4GB | 7B | RDK X5 | Medium |
| InternVL-1B | ~2GB | 1B | RDK X5 | Fast |
| GPT-3.5-like | ~10GB | 13B | RDK X5+ | Slow |

### Parameter Tuning

**Generation parameter effects**:

- **temperature (0.7)**: Controls randomness, higher is more random
- **top_p (0.9)**: Nucleus sampling, keeps tokens with cumulative probability top_p
- **top_k (40)**: Only consider top k tokens by probability
- **max_tokens (256)**: Maximum generation length
- **frequency_penalty (0.0)**: Penalty for repeated tokens

## Usage Examples

### Basic Text Generation

```cpp
#include "llm_engine.h"
#include <iostream>

int main() {
    LLMEngine engine;
    
    // Initialize
    LLMConfig config;
    config.n_threads = 4;
    config.context_size = 2048;
    
    engine.Init("./models/qwen-1.8b.gguf", config);
    
    // Set system prompt
    engine.SetSystemPrompt("You are a helpful AI assistant.");
    
    // Generate text
    std::string prompt = "Tell me about the Captain Milo project";
    GenerateParams params;
    params.max_tokens = 512;
    params.temperature = 0.7f;
    
    std::string response = engine.Generate(prompt, params);
    std::cout << "Response: " << response << std::endl;
    
    engine.Deinit();
    return 0;
}
```

### Stream Generation

```cpp
// Stream generation example
engine.GenerateStream(
    "Tell me an interesting story",
    [](const std::string& chunk) {
        std::cout << chunk << std::flush;  // Real-time output
    },
    params
);
std::cout << std::endl;
```

### Image Analysis

```cpp
#include "vlm_engine.h"
#include <opencv2/opencv.hpp>

int main() {
    VLMEngine engine;
    
    VLMConfig config;
    config.image_size = 448;
    
    engine.Init("./models/internvl-1b.gguf", config);
    
    // Load and analyze image
    cv::Mat image = cv::imread("photo.jpg");
    std::string analysis = engine.AnalyzeImage(
        image,
        "What objects are in this image? Please describe in detail."
    );
    
    std::cout << "Analysis: " << analysis << std::endl;
    
    engine.Deinit();
    return 0;
}
```

### ROS2 Integration

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "llm_engine.h"

class LLMNode : public rclcpp::Node {
public:
    LLMNode() : Node("llm_node") {
        engine_.Init("./models/qwen-1.8b.gguf", config_);
        
        // Create service
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/llm/generate",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                res->message = engine_.Generate("Hello, how are you?", params_);
                res->success = true;
            }
        );
    }
    
private:
    LLMEngine engine_;
    LLMConfig config_;
    GenerateParams params_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};
```

## Performance Metrics

### Inference Speed
- **First token latency**: ~500ms (1.8B model)
- **Output speed**: 50-100 tokens/second
- **Throughput**: Single-core processing

### Memory Usage
- **1.8B model**: ~2-3GB RAM
- **7B Q4 model**: ~5-6GB RAM
- **Runtime overhead**: ~500MB

### Accuracy Metrics
- **MMLU (Multi-task understanding)**: 60%+ (1.8B)
- **CEVAL (Chinese evaluation)**: 70%+ (1.8B)
- **Generation quality**: Stable and fluent

## Troubleshooting

### Common Issues

**Q: Slow inference speed**
- A: Reduce max_tokens, lower context_size, use quantized models

**Q: Out of memory**
- A: Use smaller models (1.8B vs 7B), enable quantization

**Q: Poor generation quality**
- A: Adjust temperature, optimize system prompt, choose better model

**Q: GPU acceleration not working**
- A: Check CUDA environment, ensure model supports GPU offloading

### Debug

```bash
# Enable debug logging
export LLAMA_DEBUG=1
ros2 launch RDK-Offline-LLM llama_llm.launch.py

# Monitor performance
ros2 topic pub /llm/monitor std_msgs/String '{data: "get_stats"}'
```

## Dependencies

- ROS2 (Humble or newer)
- C++ 17 or newer
- llama.cpp
- OpenCV (required for VLM)
- Optional: CUDA (GPU acceleration)

## Related Resources

- [llama.cpp GitHub](https://github.com/ggerganov/llama.cpp)
- [InternVL GitHub](https://github.com/OpenGVLab/InternVL)
- [Qwen GitHub](https://github.com/QwenLM/Qwen)
- [ROS2 Documentation](https://docs.ros.org/)
