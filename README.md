# send_lb

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
./build/send_lb file /home/arty/Documents/video/1.avi libx265
./build/send_lb camera 0 libx265
./build/send_lb --source=hikrobot --vid-pid=2bdf:0001 libx265
```

Hikrobot 模式现在使用仓库内的 `third_party/hikrobot` 头文件和库文件，不需要再手动指定 `HIKROBOT_ROOT`。
Hikrobot 模式会把原始帧自动录到 `/home/arty/Documents/video/` 下，文件名形如 `hikrobot_raw_YYYYMMDD_HHMMSS.avi`，录制文件的 fps 会按相机实际收到的帧率自动估算，不再固定为 27。

串口模式编译（Ubuntu）：

```bash
cmake -S . -B build -DLOW_BANDWIDTH_TRANSPORT_MODE=2
cmake --build build -j
```

说明：
- `LOW_BANDWIDTH_TRANSPORT_MODE=1`：UDP
- `LOW_BANDWIDTH_TRANSPORT_MODE=2`：Serial(raw)
