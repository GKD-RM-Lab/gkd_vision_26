# 视觉端修改报告：图传、动态调参及画面回执

日期：2026-11-07。本文按当前视觉工作区源码整理，供本次 Git 提交和后续联调使用。

本次整理仅新增本报告，没有修改业务代码、配置值或 Git 暂存区。报告区分当前待提交改动、已经存在于基线中的图传能力，以及尚未解决的构建问题；不将此前版本的测试结果作为当前工作区重新验证通过的证明。

## 1. 功能范围与接入入口

当前 `gkdhero_fiexed_debug` 已接入以下功能：

1. 将海康相机完整画面经过自瞄绘制后送入独立视频线程，缩放、H.265 编码、分片，经本机 UDP 发给电控。
2. 接收电控转发的运行时参数请求，在视觉主线程的帧边界应用 V1/V2 和敌方颜色修改。
3. 在原始画面左上角绘制指令结果与实际参数，通过原视频链路返回选手端。

本次 `HEAD` 基线为 `92e1eee`（`video_stream`）。`io/video/`、`io/gkdcontrol/video_sender.*` 及原有视频接入已在基线中；本次待提交的主要功能增量是动态调参、颜色切换与 Compact16 画面回执。

运行时参数功能目前仅接入 `src/gkdhero_fiexed_debug.cpp`。其他英雄、步兵、哨兵入口不会因公共类新增接口而自动获得远程调参功能。报告没有将其描述为已全部适配。

## 2. 通信链路和线程划分

```text
选手端
  │ MQTT 3.1.1：CustomControl，固定 16 B 业务 data
  ▼
Broker → 电控参数桥接模块
  │ 解包、校验，并补齐视觉会话、版本、过期时间
  │ 本机 UDP JSON → 127.0.0.1:11455
  ▼
视觉 ParameterChannel 后台线程 → 有界请求队列
  ▼
视觉主线程：每帧最多处理一条请求 → RuntimeParams → Tracker/颜色模式
  │
  ├─ 本机 UDP JSON 状态/回执 → 127.0.0.1:11456 → 电控内部协调
  │                                       （不再转成独立 MQTT 回执）
  ▼
自瞄绘制红框 → 左上角参数/结果文字 → 完整画面快照
  ▼
VideoStreamer 后台线程：resize → H.265 → VS v2 分片
  │ 本机 UDP → 127.0.0.1:11454
  ▼
电控 → MQTT CustomByteBlock → Broker → 选手端重组、解码、展示
```

- 视觉端不直接连接 MQTT，也不依赖 Protobuf 或 libmosquitto；这些外部协议适配在电控端完成。
- 原控制 UDP 端口 `11451` 保持独立。参数 `11455/11456` 与视频 `11454` 不复用控制消息格式。
- 网络线程只解析、排队、发送本机反馈；不直接修改 EKF、Tracker 或正在绘制的图像。
- 主线程使用 `try_lock` 尝试取请求、提交状态和回执；不能立即取得锁时跳过，不等待网络线程。
- 视频编码和分片在独立线程完成。主线程仍需执行限帧判断及必要的 `cv::Mat::clone()`，不能理解为视频完全没有主线程开销。
- 主循环先执行原有 `gkdcontrol.send(command)`，再提交参数反馈和视频。视频发送使用独立非阻塞 socket，并检查控制发送活动标志。这是避免视频阻塞控制的软件措施，不是操作系统层面的实时发送保证。
- 原本地 `imshow`/`waitKey` 仍在主循环中，并非本次新增的后台操作。

## 3. 运行时参数实现

### 3.1 配置与运行状态

新增 `tasks/auto_aim/runtime_params.hpp/.cpp`：

- `RuntimeConfig`：启动时读取 YAML 中的权限、端口、队列和有效期限制。
- `RuntimeState`：保存当前 V1/V2、颜色模式、实际敌方颜色、会话、版本、跟踪状态和最近指令结果。
- `RuntimeParams::begin_frame()`：更新本帧自动颜色来源和帧信息。
- `RuntimeParams::apply()`：校验请求并应用修改。
- `RuntimeParams::draw_overlay()`：在原始完整画面左上角绘制反馈。

YAML 顶层 `process_v1/process_v2` 仍然是启动初值。远程调参修改当前进程内存，不写回 YAML；重启后重新读取 YAML，颜色模式重新从 `auto` 开始。除 `video.enabled` 外，本文涉及的配置不支持通过编辑 YAML 自动热更新。

### 3.2 V1/V2 更新

- V1/V2 成对更新；也可以与颜色放在同一条 Apply 请求中。
- 所有选中字段全部通过校验后才一次更新，失败不会只修改其中一部分。
- 新增 `Target::set_process_noise()` 和 `Tracker::set_process_noise()`，更新后续过程噪声矩阵 Q 的参数。
- V1 作用于位置/速度过程噪声，V2 作用于偏航角/角速度过程噪声；不修改 EKF 预测/观测方程。
- 仅修改 V1/V2 时保留当前目标的状态 x 和协方差 P，不重建滤波器；后续新建目标同样使用最新参数。
- 成功处理一条新修改请求后，`revision` 加 1。相同请求精确重试只确认，不重复应用、不再次增加版本。

### 3.3 敌方颜色切换

- 模式为 `auto`、`red`、`blue`。`auto` 使用原有电控颜色来源，另外两种强制指定敌方颜色。
- 同时受视觉端 `runtime_params.color_override_enabled` 和电控端对应权限约束。
- 选手端应检查 Shift、Ctrl、鼠标左键、鼠标右键同时持续按住，默认至少 500 ms。
- 视觉端检查消息声明的持续时间；无法从网络消息独立验证四个物理按键是否真的按下。
- 实际生效的敌方颜色变化时，调用 `Tracker::reset()`、`Aimer::reset_tracking()`、`CenterAimer::reset_tracking()` 清除旧目标和瞄准缓存，避免沿用另一颜色的目标。只切换模式但实际颜色相同，不额外重置。

### 3.4 请求保护

视觉端检查本次进程 `session_id`、请求 ID、参数版本、绝对过期时间、排队时长、参数有限性和范围、颜色权限及确认时长。

本机 JSON 数据报上限为 2048 B，并检查字段集合、类型及嵌套深度。请求和回执队列分别限长；请求满时不覆盖已有写操作，Compact16 请求可转为 `vision_queue_full` 画面诊断。成功请求缓存最多 128 条，用于当前进程内去重。

`bridge_notice` 只更新画面中的拒绝原因，不修改业务参数；要求来自配置的本机电控状态端口，旧会话或过期通知会忽略。旧 `get/set_ekf/set_enemy_color` 仍是本机接口能力，外部 MQTT 已统一使用 Apply。

## 4. 视频和消息长度

### 4.1 图像来源与编码

图像源为原主循环中的 `img`，已包含重投影绘制、红色瞄准框和新的参数回执。先保留完整图像引用，再执行原本地显示缩放；提交时对完整画面做快照，不是传原始未绘制相机图，也不是截取 ROI 或本地显示窗口。

视频队列默认容量 1，积压时覆盖旧帧，工作线程取最新帧。工作线程执行 `cv::resize()`，再通过 FFmpeg 的 `libx265` 编码为 Annex B H.265，采用低延迟配置。丢弃已编码数据后强制后续关键帧，帮助接收端恢复参考链。

`video.enabled` 约每秒检查一次 YAML，可运行时关闭；再次开启等待新的可用画面。分辨率、帧率、码率等编码/传输参数需要重启生效。

### 4.2 本机 UDP 视频格式

```text
6 B 本机信封 + 36 B VS v2 片头 + 1～204 B H.265 字节
```

| 部分 | 内容 |
|---|---|
| 6 B 本机信封 | `0x6B`、版本 2、`0x0310` 命令号、实际分片长度；多字节字段小端 |
| 36 B VS v2 片头 | `VS` 标记、版本、关键帧标志、帧 ID、采集时间戳、整帧编码长度、分片序号/总数、有效长度、宽高、编码标识、保留字节、帧内字节偏移 |
| 有效视频字节 | 默认每片最多 204 B；末片按实际长度发送，不补齐 |

当前默认 `video_block_max_bytes=240`，因此视频 UDP 有效载荷为 **43～246 B**，不再固定为旧的 306 B；不包含 UDP/IP 头。本机信封不是官方 UART 帧。

### 4.3 MQTT 两个方向的边界

以下 MQTT PUBLISH 长度按当前主题名和 QoS 计算，不含 TCP/IP 和独立确认报文：

| 方向 | 主题与 QoS | 业务 data | Protobuf payload | MQTT PUBLISH |
|---|---|---:|---:|---:|
| 选手端 → 电控 | `CustomControl`，QoS 1 | 固定 16 B | 18 B | 37 B |
| 电控 → 选手端 | `CustomByteBlock`，QoS 0 | VS 视频 37～240 B | 39～243 B | 58～263 B |

Compact16 的 16 B 字段为：flags 1 B、序号 2 B、Unix 毫秒时间戳低 32 位 4 B、V1 float32 4 B、V2 float32 4 B、四键时长 1 B（50 ms 单位），全部多字节字段小端。未选中字段应按协议清零。

视觉收到的仍是电控转换后的 JSON，并不是直接解析这 16 B。当前 `CustomByteBlock` 只上传视频；应用回执和状态不再作为独立 MQTT 消息上传。本机 JSON 状态保留用于电控协调，长度不受上述外部业务 data 预算约束。

300 B 的业务 data 预算中当前为视频保留 240 B，其余 60 B 仅作预算预留，没有发送 60 B 填充，也尚未实现与其他业务数据在一条消息中复用。降低视频块长度时，需要同时降低码率或重新评估分片发送能力。

这些长度说明当前实现的封装边界，不替代 RMUL 对设备、链路和赛场使用方式的正式要求。

## 5. 左上角画面回执

```text
CMD #43 OK r2
V1=120 V2=100
EN=auto>BLUE tracking
```

| 显示 | 含义 |
|---|---|
| `CMD -- READY` | 尚未处理修改请求 |
| `#43` | 最近请求的选手端序号 |
| `OK / ERR / DUP` | 修改成功 / 拒绝 / 精确重复请求已确认 |
| `r2` | 当前参数版本号 |
| `V1 / V2` | 当前视觉实际生效值，不是选手端输入草稿 |
| `EN=auto>BLUE` | 当前模式为自动，实际敌方为蓝色 |
| `tracking / detecting / lost / temp_lost / switching` | 跟踪 / 确认检测 / 丢失 / 暂时丢失 / 切换目标 |
| 第四行 | 最近拒绝原因，例如 `invalid_v1_v2`、`color_override_disabled` |

最后结果会保留到下一条结果到来。文字有描边，并按原图宽度缩放以适配最终 256×192 图像。`OK` 表示参数应用成功，`tracking` 表示正在跟踪；当前没有独立实现 EKF 收敛判定指标。

当前源码将原来的两行控制/误差图像文字注释掉，以避免覆盖 HUD；终端中的帧耗时、目标中心及开火状态日志仍保留。早期报告中“旧文字下移”的描述不再对应当前文件。

## 6. 当前 YAML 配置快照

以下是整理报告时 `configs/gkdhero_fiexed.yaml` 的实际值，不是推荐值或自动修改操作。

| 配置 | 当前值 | 作用 |
|---|---|---|
| `process_v1 / process_v2` | `120 / 100` | 启动过程噪声参数 |
| `runtime_params.enabled` | `true` | 开启本机运行时参数服务 |
| `runtime_params.color_override_enabled` | `true` | 允许经过校验的颜色修改 |
| `runtime_params.show_overlay` | `true` | 绘制左上角反馈 |
| `request_port / status_port` | `11455 / 11456` | 本机参数请求/状态端口 |
| `runtime_params.queue_capacity` | `8` | 请求、回执队列分别限长 |
| `max_request_ttl_ms / max_queue_age_ms` | `2000 / 500` | 有效期和队列年龄限制 |
| `status_hz` | `1.0` | 本机 UDP 状态频率，不是 MQTT 上传频率 |
| `process_min_exclusive / process_max_inclusive` | `0 / 1000000` | 数值输入保护，不代表经过实测的调参范围 |
| `color_confirmation_hold_ms` | `500` | 四键声明持续时间下限 |
| `video.enabled` | `true` | 开启图传，可单独热关闭 |
| `video.width / height / fps` | `256 / 192 / 10` | 目标尺寸与编码帧率 |
| `video.bitrate / gop` | `32000 / 10` | H.265 目标 bit/s 与关键帧间隔 |
| `video.queue_capacity` | `1` | 单帧覆盖队列 |
| `video.packet_hz` | `40` | 视频分片发送上限，不是图像 FPS |
| `video.max_frame_age_ms` | `500` | 从采集时刻计算的图像年龄限制 |
| `video.stats_interval_s` | `2.0` | 统计日志间隔 |
| `video_block_max_bytes / other_info_reserved_bytes` | `240 / 60` | 视频块与其他数据预算 |
| `video.udp_port` | `11454` | 电控视频接收端口 |

实际完整发送帧率会受编码大小、关键帧和分片限速影响，不能用配置中的 `fps=10` 作为端到端达到 10 FPS 的证明。

## 7. 文件修改清单

### 7.1 当前新增/修改的功能文件

| 文件 | 作用 |
|---|---|
| `tasks/auto_aim/runtime_params.hpp/.cpp`（新增） | 配置、运行状态、校验、事务应用、去重和 HUD |
| `io/gkdcontrol/parameter_channel.hpp/.cpp`（新增） | 本机 UDP 参数服务、解析、限长队列及后台反馈 |
| `io/gkdcontrol/CMakeLists.txt`（新增） | 编译 `runtime_parameter_support` 静态库 |
| `src/gkdhero_fiexed_debug.cpp` | 初始化模块，帧边界应用请求，绘制 HUD，控制发送后提交反馈与视频 |
| `tasks/auto_aim/target.hpp/.cpp` | 增加过程噪声更新接口，保留 x/P |
| `tasks/auto_aim/tracker.hpp/.cpp` | 增加噪声更新和颜色切换复位；头文件改用所需的检测数据声明，避免引入完整感知器依赖 |
| `tasks/auto_aim/aimer.hpp/.cpp` | 增加清除锁定目标与调试瞄准点的接口 |
| `tasks/auto_aim/center_aimer.hpp/.cpp` | 增加清除上一帧 pitch 缓存的接口 |
| `configs/gkdhero_fiexed.yaml` | 增加参数服务配置，更新图传注释，当前打开视频和颜色修改 |
| `CMakeLists.txt` | 接入参数库及测试，新增独立参数测试构建选项 |
| `tests/runtime_params_test.cpp`、`tests/runtime_params/CMakeLists.txt`（新增） | 参数事务、真实 Tracker/EKF、颜色缓存和本机 UDP 测试 |

### 7.2 已有图传相关文件

`io/video/video_streamer.*`、`h265_encoder.*`、`video_packet.*`、`io/gkdcontrol/video_sender.*`、`io/video/CMakeLists.txt`、`tests/video_streamer_test.cpp` 为当前链路已有实现。本次通过它们传输新增的画面回执，不应在提交说明中将全部已有视频代码表述为本次新加。

原 YOLO 检测实现、EKF 预测/更新数学公式、弹道计算、红框计算及绘制代码、原云台角度和开火指令计算没有因本次参数适配重写；新增的业务行为是噪声输入可变、颜色切换时清理旧状态及画面反馈。

### 7.3 文档与已有删除项

- 新增本报告 `VISION_CHANGES.md`，作为当前视觉端实现和配置状态的汇总。
- 当前工作区还包含 `COMPACT16_CHANGES.md` 和历史 `CUSTOM_MESSAGE_V1_CHANGES.md`。前者记录跨视觉/电控的 Compact16 改造，后者是旧协议历史，当前实现以源码与本报告为准。
- 当前 Git 还显示 `VIDEO_STREAMING_CHANGES.md`、`src/gkdsentry_video_debug.cpp` 已删除。这些删除在整理报告前已存在；本次没有恢复文件或将其解释为 Compact16 所必需的删除。

## 8. 构建与验证状态

原构建方式仍为 CMake。新增参数模块需要现有 OpenCV、Eigen3、fmt、spdlog、yaml-cpp、nlohmann_json 和线程库；图传额外需要 FFmpeg 的 `libavcodec/libavutil/libswscale`，且 FFmpeg 必须提供 `libx265` 编码器。完整视觉工程仍需要原相机 SDK、OpenVINO 等依赖。

**当前全量构建待处理项：** 根 `CMakeLists.txt` 仍为 `gkdsentry_video_debug` 指定 `src/gkdsentry_video_debug.cpp`，但该源文件当前已删除。正常完整 CMake 配置/生成在依赖满足后仍会因缺失源文件而失败。提交前应根据是否保留该哨兵视频入口，恢复文件或同步移除目标；本次只写报告，没有擅自选择处理方式。

解决上述源文件与构建目标不一致后，完整编译命令保持为：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

已有独立测试可绕过相机 SDK/OpenVINO 和上述完整目标问题；在依赖就绪的桌面测试环境可分别执行：

```bash
cmake -S . -B build-runtime-test -DSP_VISION_RUNTIME_TESTS_ONLY=ON
cmake --build build-runtime-test -j"$(nproc)"
ctest --test-dir build-runtime-test --output-on-failure

cmake -S . -B build-video-test -DSP_VISION_VIDEO_TESTS_ONLY=ON
cmake --build build-video-test -j"$(nproc)"
ctest --test-dir build-video-test --output-on-failure
```

现有测试覆盖参数事务/原子性、无部分修改、权限与过期拒绝、重复请求、真实目标/EKF 状态保留、跟踪复位、HUD、本机 UDP，以及视频分包、编码解码、背压、关键帧恢复和开关热加载。测试使用合成图像与本机通信，不需要运行机器人主程序。

本次报告整理进行了源码、配置、Git 差异和文档格式核对，未重新执行完整编译、单元测试或机器人硬件联调。此前 Compact16 阶段曾进行构建和端到端合成图像验证，记录见 `COMPACT16_CHANGES.md`，不能代替当前删除文件之后的构建复验。

视频日志继续统计编码耗时、单帧编码大小、完整本地发送帧率、分片业务字节带宽、队列丢帧及传输错误；`sent_frames` 仅代表所有分片被本机发送接口接受，不是选手端接收确认，带宽统计也不包含 UDP/IP、MQTT/TCP 外层开销。

本次没有执行 `git add`、`git commit` 或部署。视觉报告只属于本仓库，电控与 Broker 的文件需要在对应仓库单独提交。
