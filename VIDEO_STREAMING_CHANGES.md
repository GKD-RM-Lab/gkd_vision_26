# 自瞄绘制后视频传输：修改说明与联调约定

更新日期：2026-11-02。

本次以 `src/gkdsentry_debug.cpp` 为基础，新建独立主程序
`src/gkdsentry_video_debug.cpp`，构建目标为 `gkdsentry_video_debug`。
复用工作区已有的 `gkdhero_fiexed_debug` 视频初版及 `io/video`、
`io/gkdcontrol/video_sender` 模块，保持项目原有 `src` / `io` / `configs` / `tests` 目录约定。
原 `src/gkdsentry_debug.cpp` 不作修改，英雄初版也继续保留。

**现有哨兵基线使用 `auto_aim::Detector`，配置为 `use_traditional: true`；
本次逐行保留这一检测入口，没有切换成英雄初版的 YOLO 或 CenterAimer/CenterShooter。**
EKF、弹道预测、红框绘制、开火判定、控制发送位置全部沿用哨兵基线。

本仓库 `GKDControl` 实际通过本机 UDP 桥接发送控制，持有物理串口的桥接/电控代码不在当前工程。
本次完成视觉端本地编码、分包及独立发送接口；没有连接实体海康相机、电控或终端，
没有完成物理串口转发、硬件控制优先级与终端显示验收。接入方式和后续验收步骤见第 4、7 节。

## 1. 完成的功能与数据路径

```text
现有海康 Camera::read(img, t)
  -> 原检测器 / EKF / 弹道 / 红框等绘制
  -> 保留完整 img 的浅引用 video_frame
  -> 原窗口缩放、显示与 GKDControl::send(command)
  -> VideoStreamer::submit(video_frame, t)：按 15 FPS 上限复制快照
  -> 有界图像队列：覆盖旧帧，工作线程总取最新帧
  -> 独立低优先级线程 cv::resize(256 x 192)
  -> FFmpeg/libx265，YUV420P、H.265 Annex B、低延迟低码率
  -> 每帧元数据、300 字节分片、最多 50 包/秒
  -> GKDControl::try_send_video -> 独立非阻塞 UDP socket -> 127.0.0.1:11454
  -> 【后续适配】桥接端串口调度 -> 电控 -> 官方 0x0310 -> 自定义客户端
```

源图像保留的是所有自瞄绘制完成后的整张画面，位于原有红框绘制之后、
`cv::resize(img, img, {}, 0.5, 0.5)` 之前。缩放完整画面，不裁剪 ROI，
没有增加新的相机采集线程，也没有改用原始相机帧。

主程序只保留浅引用、按目标帧率复制图像和尝试入队；复制安排在原控制发送之后。
这是一次必要的内存复制，仍有少量 CPU/内存开销，但不等待编码、分片、UDP 发送或队列空位。
`try_lock` 失败直接丢弃，不等待队列锁。队列默认容量 1，无积压式编码包队列。

编码线程使用单线程 x265、`ultrafast` / `zerolatency`、无 B 帧与前向缓存，
尝试降低自身 Linux 调度优先级；不能取得额外调度权限也不影响运行。
编码器异常、配置错误会关闭视频并记录日志，不向自瞄主循环传播异常。

## 2. 逐文件变更

### 2.1 本次哨兵接入实际新增/修改

| 文件 | 修改内容 |
| --- | --- |
| `src/gkdsentry_video_debug.cpp`（新增） | 复制哨兵原主程序，仅新增视频头文件、初始化、完整绘制图像引用和原控制发送之后的快照提交 |
| 根目录 `CMakeLists.txt` | 新增 `gkdsentry_video_debug` 可执行目标，链接已有 `video_streaming` 模块及原自瞄依赖 |
| `configs/gkdsentry.yaml` | 新增独立 `video` 节，默认关闭；256×192、15 FPS、48000 bit/s、容量 1；支持运行时开关 |
| `tests/video_streamer_test.cpp` | 补充主程序显示缩放后仍使用完整图像的回归场景；保存实际分片重组并解码得到的 `streamed_frame.png` |
| `readme.md` | 补充哨兵视频版构建、运行及开启方式 |
| `VIDEO_STREAMING_CHANGES.md` | 更新本说明，区分本次哨兵接入与工作区已有初版，记录当前验证结果及硬件联调边界 |

新主程序与原哨兵程序相比只有四类视频接入增加：`#include`、初始化、绘制后保留 `img`、
控制发送后 `submit()`。原窗口名称、日志、Detector、Tracker、Aimer、Shooter 及控制调用保持原样。
可直接执行 `diff -u src/gkdsentry_debug.cpp src/gkdsentry_video_debug.cpp` 审阅全部接入差异。

### 2.2 工作区已有视频初版（本次继续复用）

以下文件在本次开始前已经存在于工作区修改中；为覆盖整体视频功能列出职责，
不表示这些算法或兼容性文件又被本次改动。

| 文件 | 修改内容 |
| --- | --- |
| `src/gkdhero_fiexed_debug.cpp` | 初始化 VideoStreamer；绘制后保留完整画面；原控制发送后提交快照 |
| `io/video/video_streamer.hpp` / `.cpp` | 独立模块、配置、有界队列、工作线程、帧率控制、限时发送、运行时开关和统计 |
| `io/video/h265_encoder.hpp` / `.cpp` | FFmpeg/libx265 封装、BGR 转 YUV420P、H.265 低延迟编码、强制恢复 IDR |
| `io/video/video_packet.hpp` / `.cpp` | 显式小端序元数据、300 字节分包、字段校验与解析接口 |
| `io/video/CMakeLists.txt` | video_streaming 静态库、编码依赖、本地测试目标 |
| `io/gkdcontrol/video_sender.hpp` / `.cpp` | 独立非阻塞 UDP 视频发送接口与消息封装 |
| `io/gkdcontrol.hpp` / `.cpp` | 新增独立 `try_send_video()`；读取视频端口，禁止使用原控制端口 |
| `io/gkdcontrol/send_control.hpp` / `.cpp` | 控制发送活动标记和 socket 优先级；控制包格式、角度符号及火控字段保持原样 |
| `io/CMakeLists.txt` | 加入 VideoSender；显式声明原有 logger 所需的 spdlog 依赖 |
| 根目录 `CMakeLists.txt` | 视频开关、模块链接、FFmpeg 依赖、仅构建视频测试模式、CTest；兼容 yaml-cpp 的命名空间目标 |
| `configs/gkdhero_fiexed.yaml` | 增加 `video` 配置节，默认关闭 |
| `tests/video_streamer_test.cpp` | 编码、图像完整性、分片重组、UDP 包格式、回压丢帧、IDR 恢复、热开关与关闭编码构建测试 |
| `io/dm_imu/dm_imu.cpp` | 补充 `usleep()` 所需 `<unistd.h>`，解决原有编译错误 |
| `tasks/auto_aim/detector.cpp`、`tasks/auto_aim/yolos/{yolov5,yolov8,yolo11}.cpp` | 仅调试图片文件名中的 `armor.name` 显式转 `int`，兼容 fmt 11；检测计算与判定未改 |
| `readme.md` | 增加本说明入口 |
| `VIDEO_STREAMING_CHANGES.md` | 本说明、协议表、测试结果、后续适配步骤 |

已有英雄初版保留原 YOLO 推理/后处理、EKF 跟踪、弹道预测、红框绘制、CenterAimer/CenterShooter、
云台控制命令计算和原控制发送调用顺序。上述图片文件名格式化、IMU 头文件兼容修改均为已有变更，
本次没有继续编辑这些文件，也没有更改任何自瞄算法文件。

## 3. 配置与关闭方式

`configs/gkdsentry.yaml` 与已有 `configs/gkdhero_fiexed.yaml` 的默认视频配置相同：

```yaml
video:
  enabled: false
  width: 256
  height: 192
  fps: 15
  bitrate: 48000
  queue_capacity: 1
  gop: 15
  packet_hz: 50
  max_frame_age_ms: 500
  stats_interval_s: 2.0
  udp_port: 11454
```

| 配置 | 语义/限制 |
| --- | --- |
| `enabled` | 运行期间修改原配置文件中的此字段即可开关；工作线程约每秒检查一次文件变更 |
| `width` / `height` | 编码输出宽高；偶数，宽 16–1920、高 16–1080；默认 256×192 |
| `fps` | 提交与编码的目标上限，1–60；默认 15，不代表链路保证每秒送达 15 帧 |
| `bitrate` | H.265 目标 bit/s；至少 1000，不能超过 `268 * packet_hz * 8` 的理论净荷预算 |
| `queue_capacity` | 原始快照队列容量，1–8；满时丢旧帧，消费时跳过旧帧只取最新帧 |
| `gop` | 编码帧间的周期关键帧间隔，1–600；默认 15 |
| `packet_hz` | 0x0310 分片发送上限，1–50；成功发送之间至少相隔一个包周期，不补发突发包 |
| `max_frame_age_ms` | 从相机采集时间计算到最后一片发送的最大期限，20–2000 ms；默认 500 |
| `stats_interval_s` | 统计打印周期，0.1–3600 秒 |
| `udp_port` | 视频桥接目标端口；不得占用 11451/11452/11453；默认 11454 |

只有 `enabled` 热更新，其他参数需要重启程序。删除整个 `video` 配置节会关闭视频。
默认配置节应在程序启动时存在；如果启动时完全没有该节，后补视频端口需要重启 GKDControl。
在发送一个帧期间修改文件，开关可能到该帧结束或过期才被检测，因此默认关闭延迟约为
1 秒加上最多 500 ms 的在途帧期限；检测关闭后立即停止后续分片并清空快照。
编码器销毁，后续重新开启从独立关键帧开始。
也提供 `VideoStreamer::set_enabled(false)` 程序接口，可即时通知发送等待退出。

保持 `enabled: false` 可正常使用原自瞄。也可使用
`-DSP_VISION_ENABLE_VIDEO=OFF` 编译完全不依赖 FFmpeg 的空视频实现，原自瞄仍可链接运行。

低码率不是任意复杂场景下的严格单帧大小保证。代码会检查帧的分片数及剩余发送期限，
无法在期限内送完的帧直接丢弃；不会通过突破官方频率限制来追赶 15 FPS。
每片最多 268 字节 H.265 净荷，50 包/秒的理论上限为 107200 bit/s，
还受末片填零和逐帧包数取整影响，因此默认留出余量选用 48000 bit/s。

## 4. 视觉与桥接端的消息约定

### 4.1 本机 UDP 消息

当前 GKDControl 实际使用 UDP，串口写入不在本仓库的主程序链路中。
本次复用 GKDControl 的对外接口，新增独立视频消息，不把图像字节混入原 `Vison_control`。
原控制目的地保持 `127.0.0.1:11451`，视频默认目的地为 `127.0.0.1:11454`。

视频 UDP 数据报固定 306 字节：

| 偏移 | 字节数 | 内容 |
| --- | --- | --- |
| 0 | 1 | 视频消息类型 `0x6B`；原控制仍为 `0x6A` |
| 1 | 1 | 本机封装版本 `1` |
| 2 | 2 | 小端命令号 `0x0310`，线上字节 `10 03` |
| 4 | 2 | 小端长度 `300`，线上字节 `2C 01` |
| 6 | 300 | 下表定义的视频分片 |

该 UDP 封装不是官方裁判系统串口帧。后续桥接端须校验类型、版本、长度，去除前 6 字节，
再把恰好 300 字节的分片交给电控或装入官方串口外层。不能将 306 字节整体作为官方数据段。

### 4.2 0x0310 自定义数据段

本实现沿用初版指定的官方 **2026 V1.3.0** 通信协议约束：数据段 300 字节，最多 50 包/秒。
依据：[官方协议 PDF，命令码表及 0x0310 说明](https://bbs-web-static.robomaster.com/ef4f084944e34393aa70378a4a405c681774586313231/RoboMaster%202026%20%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%E9%AB%98%E6%A0%A1%E7%B3%BB%E5%88%97%E8%B5%9B%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%20V1.3.0%EF%BC%8820260327%EF%BC%89.pdf)。
300 字节内部的以下布局是本工程自定义视频协议，不是官方规定的视频格式。
此处明确指定 V1.3.0，并不将其视为当前最新版本；发布版本入口见
[RoboMaster 官方通信协议资料页](https://bbs.robomaster.com/wiki/20204847/811363)。
部署时应与现场协议/固件版本核对，旧版 150 字节格式不能直接使用本实现。

所有多字节字段均显式小端序，独立于编译器结构体对齐方式。

| 偏移 | 大小 | 字段 |
| --- | --- | --- |
| 0 | 2 | ASCII `VS`，魔数 |
| 2 | 1 | 应用协议版本 `1` |
| 3 | 1 | flags：bit0=关键帧，其余为 0 |
| 4 | 4 | `frame_id`，uint32 帧序号；提交快照时递增，丢帧时允许跳号 |
| 8 | 8 | `timestamp_us`，uint64，采集时刻换算为 Unix 微秒 |
| 16 | 4 | `encoded_size`，uint32，该帧完整 H.265 编码字节数 |
| 20 | 2 | `fragment_index`，uint16，从 0 开始 |
| 22 | 2 | `fragment_count`，uint16，总片数 |
| 24 | 2 | `payload_size`，uint16，本片有效编码字节数，1–268 |
| 26 | 2 | `width`，uint16 |
| 28 | 2 | `height`，uint16 |
| 30 | 1 | 编码类型，`1`=H.265 HEVC Annex B |
| 31 | 1 | 保留，0 |
| 32 | 268 | 编码净荷，最后一片不足部分填 0，填零不属于码流 |

每片都包含完整帧元数据，允许按帧号和片号重组、乱序接收与重复片去重。
应验证同一帧的时间戳、分辨率、编码长度、片数和关键帧标记一致，验证片号范围与净荷长度，
限制重组缓存，然后将每片 `payload_size` 个字节按片号拼接。
拼接后的总字节数必须等于 `encoded_size`，只将完整帧交给 HEVC 解码器。
本协议单帧最大为 `65535 * 268` 字节；终端应采用更小的、符合实际分辨率的内存上限。
帧序号会回绕，程序重启后也会重置，终端需结合时间戳和关键帧处理流重启。

编码采用 VPS/SPS/PPS 随 IDR 重复发送的 Annex B 流。视觉端丢弃已编码帧或仅发出部分分片后，
下一帧强制 IDR；未编码的旧快照丢弃不破坏参考帧链。无 ACK/重传，链路自身丢片时终端应
丢弃残帧，等待下一个可解码 IDR；仅拿到 P 帧不能从零开始解码。
终端应按时间戳显示、设置重组超时，避免自身再次累积显示延迟。

官方串口外层的 SOF、序号、命令号及 CRC8/CRC16 由后续桥接/电控端负责。
本次没有直接写入任何实体串口，也没有擅自改变 `io/gimbal` 的另一套串口协议。

### 4.3 控制优先级的范围

视觉端控制和视频使用不同 socket、不同内核发送缓存、不同端口，无共享发送锁。
控制 socket 设置优先级 6，视频优先级 0；视频采用 `MSG_DONTWAIT`、有界小发送缓存，
控制发送活动时视频让出。控制发送不需要等待视频线程、编码器或视频队列。
主线程的快照复制安排在本次控制包发送之后。

这些措施保证视觉端没有由视频队列/发送互斥引入的控制阻塞；socket 优先级仍受系统调度影响，
不构成跨进程和物理串口的硬实时保证。端到端的“控制永远优先”需要后续桥接/电控适配：

1. 控制消息使用独立高优先级队列；视频使用有界低优先级队列，丢弃旧帧/过期分片。
2. 每次串口写入前先检查控制队列，禁止把整帧视频的多片一次性堵进 UART/DMA 队列。
3. 一次最多发送一个低优先级分片，随后重新检查控制；已在物理线上发送的字节不能抢占，
   需要计算这一片的最坏串行化时间并为控制预留带宽。
4. 桥接端和电控端同样执行全局 0x0310 50 包/秒上限；UDP 发送成功不代表下游已接收。

若官方外层为 5 字节头 + 2 字节命令 + 300 字节数据 + 2 字节 CRC，则每包 309 字节；
50 包/秒、8N1 所需串口线速约 154500 bit/s，还未包含原控制流量。
因此后续不能使用 115200 波特率满速承载此视频配置，应按实际串口和控制流量调整。

## 5. 构建与本地测试

原有依赖之外，开启视频编码需要 `pkg-config`、FFmpeg 开发库
`libavcodec` / `libavutil` / `libswscale`，且 FFmpeg 运行环境须具有 `libx265` 编码器。
编码器不可用时程序记录错误并关闭视频。编码接口依据
[FFmpeg 编码 API](https://www.ffmpeg.org/doxygen/3.4/group__lavc__encdec.html) 与
[FFmpeg libx265 文档](https://www.ffmpeg.org/ffmpeg-codecs.html)。

正常构建：

```bash
cmake -S . -B build/video-full -DCMAKE_BUILD_TYPE=Release -DSP_VISION_ENABLE_VIDEO=ON
cmake --build build/video-full --target gkdsentry_video_debug gkdsentry_debug -j4
./build/video-full/gkdsentry_video_debug configs/gkdsentry.yaml
```

运行前或运行期间将哨兵配置中 `video.enabled` 改为 `true`；默认 `false`。
主程序初始化仍需正常相机和电控 IMU 输入，`--help` 不需要连接硬件。
已有英雄视频版可继续构建目标 `gkdhero_fiexed_debug`。

不连接海康相机、IMU、模型或电控，单独运行本地图像编码/分包测试：

```bash
cmake -S . -B build/video-local -DCMAKE_BUILD_TYPE=Release -DSP_VISION_VIDEO_TESTS_ONLY=ON
cmake --build build/video-local --target video_streamer_test -j4
ctest --test-dir build/video-local --output-on-failure
```

本地测试仅在回环地址绑定 UDP，原控制格式测试占用 11451 和 11452；运行前须停止真实桥接程序。
测试会先绑定自己的控制接收端，再发送测试包，避免向真实桥接端发送测试控制命令。
如需自定义产物目录，也可直接运行：

```bash
./build/video-local/io/video/video_streamer_test ./build/video-local/artifacts
```

CTest 默认产物位于 `build/video-local/io/video/video_test_output/`：

| 产物 | 用途 |
| --- | --- |
| `annotated_input.png` | 640×480 合成完整画面，包含红框和四角色块 |
| `decoded_frame.png` | 实际 H.265 解码后的 256×192 图像 |
| `streamed_frame.png` | 经过与主程序相同的显示缩放、快照队列、编码、分片、重组后实际解码的 256×192 图像 |
| `local_encoded.h265` | 本地直接编码的 12 帧 Annex B 码流 |
| `fragments.bin` | 实际工作线程产生的定长 300 字节分片记录，无本机 UDP 外层 |
| `reassembled.h265` | 对完整分片重组后保存的可解码码流 |
| `hot_reload.yaml` | 测试热开关使用的临时配置 |

可用 FFmpeg 工具复查：

```bash
ffprobe -v error -count_frames -select_streams v:0 \
  -show_entries stream=codec_name,width,height,nb_read_frames \
  build/video-local/io/video/video_test_output/reassembled.h265
ffplay -framerate 15 build/video-local/io/video/video_test_output/reassembled.h265
```

验证完全不编译编码依赖的模式：

```bash
cmake -S . -B build/video-off -DSP_VISION_VIDEO_TESTS_ONLY=ON -DSP_VISION_ENABLE_VIDEO=OFF
cmake --build build/video-off --target video_streamer_test -j4
ctest --test-dir build/video-off --output-on-failure
```

本机缺少系统 spdlog/yaml-cpp 开发包时，验证使用了已有的外部依赖前缀
`/home/optimist/vision_test/gkd_vision/.deps/{spdlog,yaml-cpp,fmt}`，通过
`CMAKE_PREFIX_PATH` 和 `fmt_DIR` 传给 CMake；没有在工程中写死该机器路径。
完整构建目录使用 `build/video-full`，仅视频测试目录使用 `build/video-local`。

## 6. 调试统计与本次验证结果

日志前缀为 `[VideoStreamer]`，默认每 2 秒输出一次：

- `encode`：最近一帧编码耗时，包括 BGR 转 YUV 与编码，不含 resize/队列/发送等待。
- `size`：最近一帧完整 H.265 字节数。
- `send FPS`：统计区间内全部分片已提交本机 UDP 的帧数/秒；小区间计数可能略高于 15。
- `B/s`：统计区间内成功提交分片的 300 字节数据段带宽；包括头和填零，不包括 UDP/IP/官方串口外层。
- `queue drops`：覆盖旧快照、取最新跳过旧帧、过期快照和入队失败等累计数量；正常 FPS 节流不计丢帧。
- `transport drops`：已编码但无法在期限内完整发送的帧数，包括运行中关闭造成的中断。
- `errors`：编码/传输回调异常数；异常会关闭视频。

`stats()` 还提供累计提交数、编码数、完整发送帧数、成功分片数和字节数。
`sent_frames` 仅表示本地发送成功，没有远端 ACK，不能解释为自定义终端已收到的帧数。

### 6.1 本次哨兵接入验证（2026-11-02）

- `gkdsentry_video_debug` 与原 `gkdsentry_debug` 全工程编译、链接通过；新程序 `--help` 通过，默认配置路径为 `configs/gkdsentry.yaml`。
- 与原哨兵源码对比，仅增加视频接入语句，检测、EKF、弹道、红框、云台与火控代码保持原样。
- 开启 H.265 的独立 CTest 通过：真实 HEVC 编解码、完整视野四角色块和红框颜色校验、乱序分片重组、源 Mat 覆盖后快照保持、回压丢帧、发送限速、IDR 恢复、运行时 API/YAML 开关。
- 本次补充显示缩放回归：保留完整图像引用后对显示 Mat 原地缩小，再提交保留的完整画面；实际重组码流解码后继续校验红框及四角。
- `ffprobe` 检查本次重组产物：`codec_name=hevc`、`width=256`、`height=192`、`nb_read_frames=30`。已查看 `streamed_frame.png`，完整红框和四角色块均保留。
- 本次合成画面回压测试中，快照提交最大约 1.09 ms，队列丢帧 15、发送超时丢帧 2；该数字仅代表本机测试，不作为海康实机或硬实时延迟保证。
- 关闭 H.265 编译选项的独立 CTest 通过，无 FFmpeg 编码依赖。
- 测试中的 UDP 监听需要回环 socket 权限，默认沙箱拒绝创建 socket；获准在沙箱外重新运行后通过。没有连接真实桥接进程或向实体电控发送测试命令。

编码大小、发送 FPS、带宽、耗时和丢帧的运行日志保存在
`build/video-local/io/video/logs/`；CTest 完整日志在
`build/video-local/Testing/Temporary/LastTest.log`。
这些测试验证的是合成图像和本地接口，海康实拍画面和电控端到端验收仍待执行。

### 6.2 已有英雄视频初版的历史验证记录

以下记录来自初版说明，保留用于追溯，不代替本次哨兵硬件联调结果：

- `gkdhero_fiexed_debug` 全工程目标编译、链接通过。
- 开启 H.265 的本地测试通过：真实编码、真实解码、分片边界/乱序重组、末片填零、非法头拒绝。
- 红框解码后仍为红色，四角色块完整保留；已人工查看解码图像。
- 深拷贝验证通过：提交后覆盖源 Mat，不影响传输快照。
- UDP 视频封装校验通过，原控制包仍为 14 字节、头 0x6A，yaw 符号、pitch、fire、mode 保持原样。
- 成功分片间隔至少约 20 ms，无积压追赶突发发送；过期帧与回压丢帧验证通过。
- 运行中 `set_enabled()` 和 YAML 开关验证通过；重新启动编码与强制 IDR 可独立解码。
- 关闭 H.265 编译选项的本地测试通过，不需编码库链接。
- `ffprobe` 对一次重组产物识别为 HEVC、256×192、30 个完整帧。
- 一次持续拒绝发送的回压测试中，提交耗时最大约 1.12 ms，队列丢帧 15、发送超时丢帧 2；
  该值来自本机合成图像测试，不是海康实机/赛场端到端性能承诺。

本次没有连接实体海康相机、电控或图传链路；真实红框画面的端到端送达、串口控制优先级
及无线丢片恢复仍属于后续联调，不作为本地测试通过的结论。

## 7. 后续电控/终端适配及验收

1. 本机桥接端监听独立视频端口，按第 4 节校验封装，取得 300 字节分片。
2. 明确视觉桥接与电控串口的独立消息类型，并与原控制包分开解析；不改变已有控制字段含义。
3. 在真正持有串口/DMA 的模块实现控制队列最高优先级、限速、过期清理，记录控制发送延迟。
4. 电控将分片数据封装为官方 0x0310 外层；一片对应一个官方包，填入正确序号与 CRC。
5. 自定义终端按帧号、片号、片数、编码长度重组，只送完整 H.265 access unit 给解码器；
   丢片超时释放缓存，等待 IDR 恢复；处理视觉程序重启和帧号回绕。
6. 打开 `video.enabled`，核对终端画面与本地调试窗口：完整视野、原红框位置、颜色及更新时刻一致。
7. 对比关闭/开启视频时原自瞄循环耗时和控制发送延迟；模拟低带宽、丢包、终端离线，确认控制优先，
   延迟不随运行时间累积，统计能够显示实际发送帧率和丢帧。

桥接端/电控实现及现场验收需在后续提供对应代码和硬件后完成，本次没有修改这些部分。
