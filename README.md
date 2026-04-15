# Send_HEVC (Ubuntu Adapted)

默认已适配 Ubuntu 的 UDP 推流编译与运行。

## Ubuntu 依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev \
  libavcodec-dev libavutil-dev libswscale-dev
```

## 编译

```bash
cmake -S . -B build
cmake --build build -j
```

## 运行

```bash
./build/CameraStreamer
./build/LowBandwidthStreamerApp
./build/BufferedLowBandwidthStreamerApp
```

可选编码器参数：

```bash
./build/CameraStreamer libx265
```

串口模式编译（Ubuntu/Windows）：

```bash
cmake -S . -B build -DLOW_BANDWIDTH_TRANSPORT_MODE=2
cmake --build build -j
```

说明：
- `LOW_BANDWIDTH_TRANSPORT_MODE=1`：UDP
- `LOW_BANDWIDTH_TRANSPORT_MODE=2`：Serial(raw)
- `LOW_BANDWIDTH_TRANSPORT_MODE=3`：Serial(fixed300+crc)
