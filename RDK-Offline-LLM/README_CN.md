# RDK 离线大语言模型（LLM）模块

## 项目概述

RDK-Offline-LLM 是 Captain Milo 项目的离线大语言模型推理模块，基于 ROS2 框架和 llama.cpp 实现。提供轻量级、高效的本地 LLM 推理能力，支持多种模型和视觉语言理解。

## 核心功能

### 文本生成 (LLM)
- **llama.cpp 集成**：高效的 LLM 推理框架
- **多模型支持**：GPT、Qwen、Llama 等
- **量化支持**：INT8、INT4 等量化模型
- **离线推理**：完全本地化，无网络依赖

### 视觉语言模型 (VLM)
- **图像理解**：支持图像输入和分析
- **InternVL 集成**：先进的视觉语言能力
- **多模态**：文本 + 图像 + 视觉推理

### 对话管理
- **上下文保持**：多轮对话支持
- **提示词工程**：灵活的 prompt 管理
- **流式输出**：实时返回生成结果

## 项目结构

```
RDK-Offline-LLM/
├── src/
│   ├── llm_node.cpp               # LLM ROS2 节点
│   ├── vlm_node.cpp               # VLM ROS2 节点
│   └── llm_engine.cpp             # LLM 推理引擎
├── include/
│   ├── llm_engine.h               # LLM 引擎接口
│   └── vlm_engine.h               # VLM 引擎接口
├── launch/
│   ├── llama_llm.launch.py        # 纯文本 LLM 启动
│   ├── llama_vlm.launch.py        # 视觉语言模型启动
│   ├── ali.launch.py              # 阿里云集成启动
│   └── dosod.launch.py            # 目标检测集成启动
├── llama.cpp/
│   ├── common.h                   # 公共工具
│   ├── llama.h                    # llama.cpp 核心
│   └── llm.cpp                    # LLM 实现
├── models/
│   ├── qwen-1.8b.gguf             # 千问 1.8B 模型
│   ├── internvl-1b.gguf           # InternVL 1B 模型
│   └── llama2-7b-q4.gguf          # Llama 2 7B Q4 模型
├── CMakeLists.txt
└── package.xml
```

## 快速开始

### 编译

```bash
# 进入模块目录
cd RDK-Offline-LLM

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

#### 启动纯文本 LLM

```bash
# 启动 LLM 节点
ros2 launch RDK-Offline-LLM llama_llm.launch.py

# 发送推理请求
ros2 service call /llm/generate_text "prompt: '你好，请介绍一下你自己'"

# 监听输出
ros2 topic echo /llm/response
```

#### 启动视觉语言模型

```bash
# 启动 VLM 节点
ros2 launch RDK-Offline-LLM llama_vlm.launch.py

# 发送包含图像的请求
ros2 service call /vlm/analyze_image "image_path: '/path/to/image.jpg', question: '图片中有什么?'"

# 监听分析结果
ros2 topic echo /vlm/analysis
```

## API 参考

### LLM 引擎 API

```cpp
#include "llm_engine.h"

class LLMEngine {
public:
    // 初始化
    int Init(const std::string &model_path,
             const LLMConfig &config);
    
    // 反初始化
    void Deinit();
    
    // 生成文本
    std::string Generate(const std::string &prompt,
                        const GenerateParams &params);
    
    // 流式生成（回调方式）
    void GenerateStream(const std::string &prompt,
                       std::function<void(const std::string&)> callback,
                       const GenerateParams &params);
    
    // 重置上下文
    void ResetContext();
    
    // 设置系统提示词
    void SetSystemPrompt(const std::string &prompt);
};

// 生成参数
struct GenerateParams {
    int max_tokens = 256;
    float temperature = 0.7f;
    float top_p = 0.9f;
    int top_k = 40;
    float frequency_penalty = 0.0f;
    float presence_penalty = 0.0f;
    std::vector<std::string> stop_sequences;
};

// LLM 配置
struct LLMConfig {
    int n_gpu_layers = 0;           // GPU 层数
    int n_threads = 4;              // CPU 线程数
    int context_size = 2048;        // 上下文大小
    bool use_memory_mapped = true;  // 使用内存映射
    int batch_size = 512;           // 批处理大小
};
```

### VLM 引擎 API

```cpp
#include "vlm_engine.h"

class VLMEngine {
public:
    // 初始化
    int Init(const std::string &model_path,
             const VLMConfig &config);
    
    // 反初始化
    void Deinit();
    
    // 分析图像
    std::string AnalyzeImage(const cv::Mat &image,
                            const std::string &question);
    
    // 从文件加载图像
    std::string AnalyzeImageFile(const std::string &image_path,
                                const std::string &question);
    
    // 多图像分析
    std::string AnalyzeMultipleImages(
        const std::vector<cv::Mat> &images,
        const std::string &question);
    
    // 获取图像理解结果
    std::vector<std::string> GetImageCaption(const cv::Mat &image);
};

// VLM 配置
struct VLMConfig {
    int image_size = 448;           // 输入图像大小
    bool use_gpu = false;           // 是否使用 GPU
    int n_threads = 4;              // CPU 线程数
    int max_tokens = 512;           // 最大生成 token
};
```

### ROS2 服务接口

```cpp
// 文本生成服务
srv GenerateText:
  ---
  string response
  
// 流式文本生成
srv GenerateTextStream:
  ---
  bool success

// 图像分析服务
srv AnalyzeImage:
  string image_path
  string question
  ---
  string analysis
  
// 图像标题生成
srv GenerateCaption:
  string image_path
  ---
  string caption
```

## 配置指南

### LLM 配置 (launch/llama_llm.launch.py)

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

### 模型选择

支持的模型及规格：

| 模型 | 大小 | 参数量 | 推荐设备 | 推理速度 |
|------|------|--------|---------|---------|
| Qwen-1.8B | ~1.5GB | 1.8B | RDK X5 | 快速 |
| Qwen-7B-Q4 | ~4GB | 7B | RDK X5 | 中等 |
| Llama-2-7B-Q4 | ~4GB | 7B | RDK X5 | 中等 |
| InternVL-1B | ~2GB | 1B | RDK X5 | 快速 |
| GPT-3.5-like | ~10GB | 13B | RDK X5+ | 慢 |

### 参数调优

**生成参数影响**：

- **temperature (0.7)**：控制随机性，越高越随机
- **top_p (0.9)**：核采样，保留概率和为 top_p 的 token
- **top_k (40)**：仅考虑概率最高的 k 个 token
- **max_tokens (256)**：最大生成长度
- **frequency_penalty (0.0)**：对重复 token 的惩罚

## 使用示例

### 基础文本生成

```cpp
#include "llm_engine.h"
#include <iostream>

int main() {
    LLMEngine engine;
    
    // 初始化
    LLMConfig config;
    config.n_threads = 4;
    config.context_size = 2048;
    
    engine.Init("./models/qwen-1.8b.gguf", config);
    
    // 设置系统提示词
    engine.SetSystemPrompt("你是一个有帮助的 AI 助手。");
    
    // 生成文本
    std::string prompt = "请用中文介绍 Captain Milo 项目";
    GenerateParams params;
    params.max_tokens = 512;
    params.temperature = 0.7f;
    
    std::string response = engine.Generate(prompt, params);
    std::cout << "Response: " << response << std::endl;
    
    engine.Deinit();
    return 0;
}
```

### 流式生成

```cpp
// 流式生成示例
engine.GenerateStream(
    "讲一个有趣的故事",
    [](const std::string& chunk) {
        std::cout << chunk << std::flush;  // 实时输出
    },
    params
);
std::cout << std::endl;
```

### 图像分析

```cpp
#include "vlm_engine.h"
#include <opencv2/opencv.hpp>

int main() {
    VLMEngine engine;
    
    VLMConfig config;
    config.image_size = 448;
    
    engine.Init("./models/internvl-1b.gguf", config);
    
    // 加载和分析图像
    cv::Mat image = cv::imread("photo.jpg");
    std::string analysis = engine.AnalyzeImage(
        image,
        "这张图片中有什么物体？请详细描述。"
    );
    
    std::cout << "Analysis: " << analysis << std::endl;
    
    engine.Deinit();
    return 0;
}
```

### ROS2 集成

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "llm_engine.h"

class LLMNode : public rclcpp::Node {
public:
    LLMNode() : Node("llm_node") {
        engine_.Init("./models/qwen-1.8b.gguf", config_);
        
        // 创建服务
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

## 性能指标

### 推理速度
- **首字延迟**：~500ms（1.8B 模型）
- **输出速度**：50-100 token/秒
- **吞吐量**：单核处理

### 内存占用
- **1.8B 模型**：~2-3GB RAM
- **7B Q4 模型**：~5-6GB RAM
- **运行时开销**：~500MB

### 精度指标
- **MMLU (多任务理解)**：60%+（1.8B）
- **CEVAL (中文评估)**：70%+（1.8B）
- **生成质量**：稳定流畅

## 故障排除

### 常见问题

**Q: 推理速度慢**
- A: 减少 max_tokens，降低 context_size，使用量化模型

**Q: 内存不足**
- A: 使用更小的模型（1.8B vs 7B），启用量化

**Q: 生成结果不佳**
- A: 调整 temperature，优化 system prompt，选择更好的模型

**Q: GPU 加速不工作**
- A: 检查 CUDA 环境，确保模型支持 GPU 卸载

### 调试

```bash
# 启用调试日志
export LLAMA_DEBUG=1
ros2 launch RDK-Offline-LLM llama_llm.launch.py

# 监控性能
ros2 topic pub /llm/monitor std_msgs/String '{data: "get_stats"}'
```

## 依赖项

- ROS2 (Humble 或更新版本)
- C++ 17 或更新版本
- llama.cpp
- OpenCV (VLM 需要)
- 可选：CUDA（GPU 加速）

## 相关资源

- [llama.cpp GitHub](https://github.com/ggerganov/llama.cpp)
- [InternVL GitHub](https://github.com/OpenGVLab/InternVL)
- [Qwen GitHub](https://github.com/QwenLM/Qwen)
- [ROS2 文档](https://docs.ros.org/)

