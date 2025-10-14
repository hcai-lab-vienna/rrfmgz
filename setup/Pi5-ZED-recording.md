# ZED recording on Raspberry Pi 5

See: [stereolabs/zed-open-capture](https://github.com/stereolabs/zed-open-capture.git)

```
sudo apt install build-essential cmake libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev libopencv-dev libopencv-viz-dev
git clone https://github.com/stereolabs/zed-open-capture.git
cd zed-open-capture/udev
sudo bash install_udev_rule.sh
cd ..
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```
