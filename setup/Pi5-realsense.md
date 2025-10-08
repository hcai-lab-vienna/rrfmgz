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

## Troubleshooting

* **Issue with setup.py**

  * Ensure `_version.py` and `pyrealsense2.so` are in the installed package folder.
  * Remove any `.egg` installs before `setup.py install`.

* **No camera detected**

  * Confirm USB3 connection and run `realsense-viewer` to verify.

---

## License & References

* OG README.md: [dannyh147/Pi5-realsense](https://github.com/dannyh147/Pi5-realsense)
* Intel RealSense SDK: [https://github.com/IntelRealSense/librealsense](https://github.com/IntelRealSense/librealsense)

---

*End of README*
