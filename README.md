# send_lb

默认已适配 Ubuntu 的 UDP 推流编译与运行。

## Ubuntu 依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev \
  libavcodec-dev libavutil-dev libswscale-dev libyaml-cpp-dev
```

## 编译

```bash
cmake -S . -B build
cmake --build build -j
```

## 运行

```bash
./build/send_lb
```

程序启动时固定读取 `config/send_lb.yaml`。
传输方式由 `transport.mode` 决定，支持 `udp` 和 `serial`。

Hikrobot 模式现在使用仓库内的 `third_party/hikrobot` 头文件和库文件，不需要再手动指定 `HIKROBOT_ROOT`。
Hikrobot 模式会把原始帧自动录到 `/home/arty/Documents/video/` 下，文件名形如 `hikrobot_raw_YYYYMMDD_HHMMSS.avi`，录制文件的 fps 会按相机实际收到的帧率自动估算，不再固定为 27。

## 传输协议

当前推流使用固定 300 字节整包的流式协议：

- 每个 wire packet 都固定 300 字节。
- 前 16 字节是协议头，后面是连续 HEVC 字节流。
- 码流数据按顺序连续填充，允许一个包里混入多帧内容，也允许一帧拆到多个包里。
- 协议头里包含：
  - `magic/version`
  - `packet_seq`
  - `payload_len`
  - `flags`，至少包含 `keyframe` 和 `end_of_stream`
  - `sync_offset`，用于标记关键帧在包内的起始位置

接收端处理方式：

- `packet_seq` 连续时，顺序拼接 payload。
- 一旦发现缺包，就丢弃当前损坏的码流片段。
- 直到收到带 `keyframe` 标志的新同步点，再重新开始解码。

这个版本不做重传和纠错，目标是先把简单的固定包长流跑通。
