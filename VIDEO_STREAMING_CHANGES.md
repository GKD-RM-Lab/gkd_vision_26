# 视觉视频流 v2 与 MQTT 转发修改报告

日期：2026-11-03。对应电控工程：`../GKD_Control_UI`。

本次将已有视频传输从固定 306 字节 UDP 改为可变长度，并适配电控 MQTT 转发。
原有英雄/哨兵检测、EKF、弹道预测、红框绘制、控制指令及主程序接入位置均未修改。
已有的 `gkdsentry_video_debug` 独立入口继续保留；没有改动原 `gkdsentry_debug`。

## 1. 当前完整路径

```text
海康相机完整图像 -> 原检测/跟踪/预测/绘制红框
  -> 原控制指令发送 -> VideoStreamer 提交绘制后的完整图像
  -> 最新帧队列 -> 独立线程缩放 256×192 -> H.265 Annex B
  -> v2 可变分片 -> 本机 UDP 127.0.0.1:11454（43～246 字节）
  -> 电控校验并剥离 6 字节本地封装，不解码、不 imshow
  -> 有界队列 -> Protobuf CustomByteBlock -> MQTT 3.1.1
  -> 自建 Broker 192.168.1.10:3333，Topic: CustomByteBlock
  -> 终端 192.168.1.2 订阅、解析、重组、H.265 解码和显示（终端本次未修改）
```

视觉和电控默认位于同一台 NUC，因此两者 UDP 使用回环地址。
`192.168.1.2` 是订阅端地址，发布端连接的是 Broker `192.168.1.10`。
本次是自建 Broker 测试路径，不包含官方图传硬件、裁判系统串口帧或官方链路验收。
控制仍使用原 UDP 11451 和 14 字节 `0x6A` 消息，视频独立使用 11454 和 `0x6B`。

## 2. 长度预算及可配置长度

| 层级 | 本次限制/长度 |
| --- | --- |
| 协议 `CustomByteBlock.data` 上限 | 300 字节 |
| 给未来其他信息预留 | 60 字节，仅预留，不发送填充 |
| 视频块（含视频片头） | 最多 240 字节，可调小 |
| 视频片头 | 36 字节 |
| 每片 H.265 数据 | 最多 204 字节，可随视频块预算调小 |
| 本地 UDP 外封装 | 6 字节，不进入 MQTT data |
| UDP 应用数据 | 实际 43～246 字节，不含 IP/UDP 头 |
| Protobuf 序列化 payload | 最多 243 字节，不含 MQTT/TCP/IP 头 |

300 的限制针对官方自定义数据字段，不是整个 MQTT 网络包。
当前只发送视频；尚未设计未来 60 字节的字段或混合消息格式。
以后若将其他信息放在同一条消息中，需要另行定义封装/版本，不能直接追加字节破坏现有长度校验。

例如配置 `video_block_max_bytes: 200`，每片最多 164 字节 H.265，UDP 最多 206 字节。
两端解析器无需改代码，按真实长度和 `fragment_offset` 处理。
降低分片长度会增加包数，应同时调低码率/帧率并遵守包频率预算。

## 3. UDP 与视频片头 v2

所有多字节整数使用小端序，禁止直接发送依赖编译器对齐的 C++ struct。

### 本地 UDP 外封装

| 偏移 | 字节 | 含义 |
| --- | --- | --- |
| 0 | 1 | `0x6B` 独立视频消息 |
| 1 | 1 | 版本 `2` |
| 2 | 2 | 本地消息命令 `0x0310` |
| 4 | 2 | 本次视频块实际长度 N，37≤N≤240 |
| 6 | N | 完整视频块，片头 + 编码字节 |

满包开头为 `6B 02 10 03 F0 00`。最后一片按真实长度发送，不补零。

### 视频块内部（同时也是 MQTT Protobuf 的 data 内容）

| 内部偏移 | 字节 | 含义 |
| --- | --- | --- |
| 0 | 2 | ASCII `VS` |
| 2 | 1 | 版本 `2` |
| 3 | 1 | bit0：关键帧；其余位为 0 |
| 4 | 4 | 编码帧序号 frame_id |
| 8 | 8 | 相机采集时间戳，Unix 微秒 |
| 16 | 4 | 此编码帧完整字节数 |
| 20 | 2 | 分片序号，从 0 开始 |
| 22 | 2 | 本帧分片总数 |
| 24 | 2 | 本片 H.265 字节数 P |
| 26 | 2 | 图像宽度 |
| 28 | 2 | 图像高度 |
| 30 | 1 | 编码类型 `1`：H.265 Annex B |
| 31 | 1 | 保留，0 |
| 32 | 4 | 本片编码字节在整帧内的偏移 fragment_offset |
| 36 | P | H.265 原始二进制字节，1≤P≤204 |

N 必须严格等于 36+P。重组不能继续用 `index*268` 或固定 204 推算位置，必须使用 offset。
新旧版本不兼容：旧 306 字节 v1 消息会被新电控拒收，视觉和电控必须一起升级。

帧号在工作线程完成编码时分配。编码前因图像队列覆盖而丢弃的快照不会产生序号缺口；
已编码但没发完的帧会留下缺口，并强制下一编码帧为 IDR，便于接收端恢复参考链。

## 4. 配置与延迟处理

英雄 `configs/gkdhero_fiexed.yaml`、哨兵 `configs/gkdsentry.yaml` 当前默认值一致：

```yaml
video:
  enabled: false
  width: 256
  height: 192
  fps: 10
  bitrate: 32000
  queue_capacity: 1
  gop: 10
  packet_hz: 40
  max_frame_age_ms: 500
  stats_interval_s: 2.0
  video_block_max_bytes: 240
  other_info_reserved_bytes: 60
  udp_port: 11454
```

只有 `enabled` 支持运行时热更新（约 1 秒检查一次）；其余参数修改后重启视觉进程。
保留原有 `enabled: false`，联调时在实际运行的 YAML 内改为 true。

配置校验：视频块 37～240 字节、预留至少 60 字节、二者之和≤300；包频率 1～50；
`bitrate <= (video_block_max_bytes - 36) * packet_hz * 8`。
该码率公式只是必要条件，还需为关键帧峰值及每帧末片余量留余地。
默认 40 包/秒的纯 H.265 理论预算为 65,280 bit/s，默认编码 32,000 bit/s。
10 FPS 是目标上限，不保证复杂画面或弱网络下每秒收到 10 帧。

快照提交不等待编码/网络，队列覆盖旧帧；工作线程总取最新帧。
保留原非阻塞 UDP、控制发送保护、低优先级编码线程、逐片限速及 500ms 图像截止期限。
当前 Linux 调度及 socket 保护不构成硬实时调度保证，仍需机器人实测控制周期。

## 5. 本次逐文件修改

| 文件 | 修改 |
| --- | --- |
| `io/video/video_packet.hpp` | vector 可变长分片；v2；300/60/240/36/204 常量；offset 和预算参数 |
| `io/video/video_packet.cpp` | 依据预算分包；真实长度；写入/校验 offset、边界、版本 |
| `io/video/video_streamer.hpp` | 默认 10 FPS/32000 bit/s/GOP10/40 包每秒；新增长度预算配置 |
| `io/video/video_streamer.cpp` | 读取/校验预算；按配置分包；编码帧单独编号 |
| `io/gkdcontrol/video_sender.cpp` | 外封装 v2；写实际长度；仅发送实际 6+N 字节 |
| `configs/gkdhero_fiexed.yaml` | 英雄视频默认参数和 60 字节预留 |
| `configs/gkdsentry.yaml` | 已有哨兵视频入口使用相同 v2 参数 |
| `tests/video_streamer_test.cpp` | 长度边界、37/100/200/240 预算、offset 重组、v2 UDP、原控制包及原编码回归 |
| `readme.md` | 更新现行参数和 MQTT 路径 |
| `VIDEO_STREAMING_CHANGES.md` | 本报告，取代旧固定长度说明 |

视频模块仍在原 `io/video`、发送接口仍在 `io/gkdcontrol`。
本次没有增加视觉编码依赖，也无需修改已有视觉 CMake 接入。
主程序 `src/gkdhero_fiexed_debug.cpp`、`src/gkdsentry_video_debug.cpp` 和全部自瞄算法实现未修改。

## 6. 编译与本地测试

原视觉编译方式继续使用：

```bash
cmake -S . -B build
cmake --build build -j4
```

仍需原工程 SDK/OpenVINO/OpenCV 等依赖；H.265 视频依赖 FFmpeg 开发库和运行时 libx265。
完整工程所有默认目标已编译通过，包括英雄、哨兵视频版、其他主程序及校准/测试可执行文件。

无需相机的独立测试：

```bash
cmake -S . -B build/video-test -DSP_VISION_VIDEO_TESTS_ONLY=ON
cmake --build build/video-test -j4
ctest --test-dir build/video-test --output-on-failure
```

若本机原依赖采用私有安装目录，继续沿用原 CMAKE_PREFIX_PATH/fmt_DIR 设置，不要改动系统库。
本次测试使用已有 `/home/optimist/vision_test/gkd_vision/.deps/{fmt,spdlog,yaml-cpp}`。

已通过：可变分包/重组、真实编码解码的红框和四角检查、完整绘制快照、原 14 字节控制包、
队列压力、编码帧丢弃后的 IDR 恢复、开关热更新；编译关闭 H.265 的模式也通过。
独立测试输出 `video_test_output/fragments.bin` 改为逐条 `u16小端长度 + 视频块`，不再是固定 300 字节记录。

电控测试还通过真实链路：视觉编码器 → UDP → MQTT → Protobuf → 重组 → H.265 解码。
具体依赖安装、自建 Broker、XMake 命令、终端订阅契约和测试结果见
[电控修改报告](../GKD_Control_UI/doc/VIDEO_RECEIVER_CHANGES.md)。

## 7. 联调范围

本次测试使用本机回环 Broker，没有启动电机/CAN/IMU 主程序，未接入实体海康相机。
NUC `192.168.1.10` 与终端 `192.168.1.2` 的无线网络、实际运动画面码率、终端订阅解码仍需实机验收。
终端需订阅 CustomByteBlock，按 .proto 解析 data，再按 v2 片头重组；不可直接对 MQTT payload 做 imdecode。
对丢片/帧号缺口应丢弃不完整帧、等待 IDR，不能把缺字节的 H.265 帧送入解码器。
