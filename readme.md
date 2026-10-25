# GKD2026 RMUL 视觉代码

感谢同济大学的开源， 本项目是基于同济大学开源框架改造的视觉算法库

## 依赖环境

编译要求：

- CMake `>= 3.16`
- C++17
- OpenCV
- fmt
- Eigen3
- spdlog
- yaml-cpp
- nlohmann_json
- OpenVINO 2024.6
- `usb-1.0`

本仓库还依赖以下厂商或外设 SDK/库：

- 海康工业相机 SDK

## 编译

### 1. 准备模型和 SDK

仓库默认使用以下模型文件：

- `assets/yolov5.xml`
- `assets/yolov5.bin`
- `assets/yolov8.xml`
- `assets/yolov8.bin`
- `assets/yolo11.xml`
- `assets/yolo11.bin`
- `assets/yolo11_buff_int8.xml`
- `assets/yolo11_buff_int8.bin`
- `assets/tiny_resnet.onnx`

OpenVINO 路径会优先按下面顺序查找：

```cmake
$ENV{OpenVINO_DIR}
$ENV{OPENVINO_DIR}
/opt/intel/openvino_2024.6.0/runtime/cmake
/opt/intel/openvino/runtime/cmake
```

如果开发机安装路径不同，优先通过环境变量覆盖。

### 2. 运行 CMake

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## 开发环境

仓库现在提供了一套面向 `Linux x86_64 Intel NUC` 的开发配置，包含：

- `CMakePresets.json`：统一 `cmake` / `clangd` 使用的构建目录和编译器
- `.clangd`：默认读取 `build/dev-debug/compile_commands.json`
- `Dockerfile.dev` + `docker-compose.yml`：提供和 NUC 对齐的容器开发环境
- `.devcontainer/devcontainer.json`：VS Code 进入容器后自动启用 `clangd` 与 CMake Presets

### 容器启动

默认容器平台是 `linux/amd64`，和 Intel NUC 一致：

```bash
docker compose build
docker compose up -d
docker compose exec sp-vision-dev bash
```

容器内推荐直接使用 Preset：

```bash
cmake --preset dev-debug
cmake --build --preset dev-debug -j
```

如果需要发布版：

```bash
cmake --preset dev-release
cmake --build --preset dev-release -j
```

### clangd

`clangd` 默认读取：

```text
build/dev-debug/compile_commands.json
```

因此第一次进入容器后，先执行一次：

```bash
cmake --preset dev-debug
```

这样补全、跳转、诊断都会和真实编译参数保持一致。

macOS 本地编辑也可以生成编译数据库；开发 `omniperception` 时包含该模块：

```bash
brew install cmake ninja opencv eigen fmt spdlog yaml-cpp nlohmann-json openvino
cmake --preset dev-debug -DSP_VISION_BUILD_OMNIPERCEPTION=ON
```

VS Code 使用 `llvm-vs-code-extensions.vscode-clangd` 扩展，并关闭 C/C++ 扩展的
IntelliSense，避免两套诊断同时运行。修改 CMake 目标或依赖后，重新执行上面的
配置命令；需要刷新语言服务时执行 `clangd: Restart language server`。
clangd 的数据库配置见 [官方说明](https://clangd.llvm.org/config#compilationdatabase)。

本地数据库使用本机头文件和 SDK。Linux CAN、epoll 与 ROS2 等平台专用代码
仍需要在 Linux/容器内解析和编译；macOS 上不通过伪造头文件或屏蔽诊断隐藏这些差异。

### VS Code Dev Container

如果你用 VS Code，可以直接 `Reopen in Container`。仓库已经预设：

- `clangd`
- `CMake Tools`
- 默认 `cmake` configure preset 为 `dev-debug`

容器创建完成后会自动执行一次：

```bash
cmake --preset dev-debug
```

### 说明

- 这套开发环境默认面向 `Linux x86_64` 部署目标，而不是 macOS 原生运行。
- 如果宿主机是 macOS，建议只把它当作 Docker/VS Code 的入口，实际编译、补全、调试都在容器里完成。
- 相机、串口、USB 透传这类硬件联调，优先在 NUC 或原生 Linux 主机上做。

## 运行示例

GKD 英雄：

```bash
./build/gkdhero configs/gkdhero.yaml
```

GKD 步兵：

```bash
./build/gkdinfantry configs/gkdinfantry.yaml
```

GKD 步兵 MPC：

```bash
./build/gkdinfantry_MPC configs/gkdinfantry.yaml
```

调试版程序通常可直接替换为对应的 `*_debug` 可执行文件。

当前仓库不再从配置文件读取敌方颜色。保留的 GKD 链路都会通过 `io/gkdcontrol` 从电控侧接收运行时姿态和敌方颜色。



## 代码链路概览

### 自瞄

`相机 -> 检测器/YOLO -> 解算器 -> 跟踪器 -> 瞄准器/规划器 -> 射击逻辑 -> 电控发送`

## 致谢

感谢同济大学开源的视觉框架与相关思路。
