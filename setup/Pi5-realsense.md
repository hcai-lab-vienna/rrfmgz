# RealSense on Raspberry Pi 5

Adapted from: [dannyh147/Pi5-realsense](https://github.com/dannyh147/Pi5-realsense) for Python 3.12

This README guides you through setting up the Intel RealSense SDK (librealsense) and Python wrapper on a Raspberry Pi 5 (64‑bit Raspberry Pi OS) with Python 3.12, and streaming both infrared cameras as separate RTSP feeds using **rtsp-simple-server** (MediaMTX) and **FFmpeg**.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Build and Install librealsense](#build-and-install-librealsense)
3. [Generate and Install Python Bindings](#generate-and-install-python-bindings)
4. [Verify the Python Wrapper](#verify-the-python-wrapper)
5. [Install RTSP Server (MediaMTX)](#install-rtsp-server-mediamtx)
6. [Streaming Script (](#streaming-script-stream_rtsppy)[`stream_rtsp.py`](#streaming-script-stream_rtsppy)[)](#streaming-script-stream_rtsppy)
7. [Run and Test](#run-and-test)
8. [Troubleshooting](#troubleshooting)
9. [License & References](#license--references)

---

## Prerequisites

* **Hardware**: Raspberry Pi 5 (64‑bit), Intel RealSense camera (e.g. D435i)
* **OS**: Raspberry Pi OS (Bookworm) 64‑bit
* **Python**: Python 3.12 installed (`python3.12` + `python3.12-venv`)

Install system dependencies:

```bash
sudo apt update && sudo apt install -y \
  git cmake build-essential libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev python3-pybind11 pybind11-dev \
  ffmpeg wget
```

---

## Build and Install librealsense

1. Clone the SDK and create a build directory:

   ```bash
   cd ~
   git clone https://github.com/IntelRealSense/librealsense.git
   cd librealsense
   mkdir build && cd build
   ```

2. Configure with UVC (RSUSB) backend and Python bindings enabled:

   ```bash
   cmake ../ \
     -DFORCE_RSUSB_BACKEND=ON \
     -DBUILD_PYTHON_BINDINGS=ON \
     -DPYTHON_EXECUTABLE=$(which python3.12) \
     -DCMAKE_BUILD_TYPE=Release
   ```

3. Compile and install:

   ```bash
   make -j$(nproc)
   sudo make install
   sudo ldconfig
   ```

---

## Generate and Install Python Bindings

1. \*\*Generate \*\*\`\` (needed by `setup.py`):

   ```bash
   cd ~/librealsense
   python3.12 wrappers/python/find_librs_version.py \
     "$(pwd)" "$(pwd)/wrappers/python/pyrealsense2"
   ```

2. \*\*Copy the compiled \*\*\`\` into the Python package:

   ```bash
   cp build/Release/pyrealsense2.cpython-312-aarch64-linux-gnu.so \
     wrappers/python/pyrealsense2/pyrealsense2.so
   ```

3. **Install into system site-packages**:

   ```bash
   cd wrappers/python/pyrealsense2
   sudo cp * /usr/local/lib/python3.12/dist-packages/pyrealsense2/
   ```

> This places a real `pyrealsense2/` folder under `/usr/local/lib/python3.12/dist-packages/`, containing:
>
> * `__init__.py`
> * `_version.py`
> * `pyrealsense2.so`

---

## Verify the Python Wrapper

Run this test:

```bash
python3.12 - <<EOF
import pyrealsense2 as rs
print("pipeline available?", "pipeline" in dir(rs))
p = rs.pipeline()
print("Pipeline created:", p)
EOF
```

* Should print `True` and show a pipeline object without errors.

---

## Install RTSP Server (MediaMTX)

1. Download and install the ARM64 server:

   ```bash
   wget https://github.com/bluenviron/mediamtx/releases/download/v1.12.3/mediamtx_v1.12.3_linux_arm64.tar.gz
   tar xzf mediamtx_v1.12.3_linux_arm64.tar.gz
   sudo mv mediamtx /usr/local/bin/rtsp-simple-server
   sudo chmod +x /usr/local/bin/rtsp-simple-server
   ```

2. Launch the server (default listens on `rtsp://0.0.0.0:8554`):

   ```bash
   rtsp-simple-server &
   ```

---

## Streaming Script (`stream_rtsp.py`)

Save the following as `stream_rtsp.py`:

```python
#!/usr/bin/env python3.12
"""
Dual RealSense IR → RTSP
"""
import pyrealsense2 as rs
import numpy as np
import subprocess, signal, sys

# RTSP mountpoints
RTSP1 = 'rtsp://localhost:8554/ir1'
RTSP2 = 'rtsp://localhost:8554/ir2'
# RealSense params
W, H, FPS = 1280, 720, 30

def start_ffmpeg(url):
    cmd = [
        'ffmpeg','-re','-f','rawvideo','-pix_fmt','gray8',
        '-s',f'{W}x{H}','-r',str(FPS),'-i','-',
        '-c:v','libx264','-preset','ultrafast','-tune','zerolatency',
        '-f','rtsp','-rtsp_transport','tcp',url
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)

# Setup
ff1 = start_ffmpeg(RTSP1)
ff2 = start_ffmpeg(RTSP2)
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
cfg.enable_stream(rs.stream.infrared, 2, W, H, rs.format.y8, FPS)
pipe.start(cfg)
print("Streaming IR1 → /ir1 and IR2 → /ir2")

# Graceful shutdown
 def shutdown(*_):
    for ff in (ff1, ff2): ff.stdin.close(); ff.wait()
    pipe.stop(); sys.exit(0)
for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, shutdown)

# Main loop
while True:
    frames = pipe.wait_for_frames()
    ir1 = np.asanyarray(frames.get_infrared_frame(1).get_data())
    ir2 = np.asanyarray(frames.get_infrared_frame(2).get_data())
    ff1.stdin.write(ir1.tobytes())
    ff2.stdin.write(ir2.tobytes())
```

Make it executable:

```bash
chmod +x stream_rtsp.py
```

---

## Run and Test

1. **Start RTSP server** (if not already running):

   ```bash
   rtsp-simple-server &
   ```

2. **Run streaming script**:

   ```bash
   python3.12 stream_rtsp.py
   ```

3. **View streams** in VLC or FFplay:

   ```bash
   ffplay rtsp://<pi-ip>:8554/ir1
   ffplay rtsp://<pi-ip>:8554/ir2
   ```
    You can also view the using vlc 
---

## Troubleshooting

* \`\`

  * Ensure `_version.py` and `pyrealsense2.so` are in the installed package folder.
  * Remove any `.egg` installs before `setup.py install`.

* \*\*FFmpeg \*\*\`\`

  * Make sure `rtsp-simple-server` is running and listening on port 8554.

* **No camera detected**

  * Confirm USB3 connection and run `realsense-viewer` to verify.

---

## License & References

* OG README.md: [dannyh147/Pi5-realsense](https://github.com/dannyh147/Pi5-realsense)
* Intel RealSense SDK: [https://github.com/IntelRealSense/librealsense](https://github.com/IntelRealSense/librealsense)
* MediaMTX (rtsp-simple-server): [https://github.com/bluenviron/mediamtx](https://github.com/bluenviron/mediamtx)
* FFmpeg Documentation: [https://ffmpeg.org](https://ffmpeg.org)

---

*End of README*
