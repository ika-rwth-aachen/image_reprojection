# GStreamer Image Reprojection Plugin

## Build & Install

```bash
# install dependencies
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake pkg-config \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libjson-glib-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good

# build
cmake -S . -B build
cmake --build build

# install to /opt/gstreamer/lib/gstreamer-1.0
sudo cmake --install build

# inspect plugin
export GST_PLUGIN_PATH="/opt/gstreamer/lib/gstreamer-1.0:${GST_PLUGIN_PATH}"
gst-inspect-1.0 imagereprojection
```
