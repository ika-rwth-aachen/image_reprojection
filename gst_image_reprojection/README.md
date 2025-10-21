# GStreamer Image Reprojection Plugin

This plugin mirrors the ROS `image_reprojection` node and projects multiple camera feeds into a single stitched output using either a planar pinhole mosaic or an equirectangular panorama. It consumes the JSON configuration exported by the ROS node (see `output.gstreamer.config_export_path`) so that ROS and GStreamer produce identical virtual camera outputs.

## Features
- Planar pinhole or equirectangular panorama projection (select via `projection-mode` property).
- Static per-camera intrinsics and transforms are cached exactly as in the ROS component.
- Supports an arbitrary number of cameras; sink pads are named `sink_0`, `sink_1`, … matching the camera ordering in the exported JSON.
- BGR8 input/output (uses `video/x-raw,format=BGR`).

## Build Requirements
Install the following packages before building:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake pkg-config \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libjson-glib-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good
```

## Build & Install

```bash
cmake -S . -B build
cmake --build build
sudo cmake --install build
```

The shared object is installed to `lib/gstreamer-1.0/gstimagereprojection.so`. Ensure that directory is on `GST_PLUGIN_PATH` (or copy it into an existing plugin directory).

## Docker Build
A convenience Dockerfile is provided:

```bash
docker build -t gst-image-reprojection -f Dockerfile .
```

The image compiles the plugin and leaves the artifact in `/opt/gstreamer/lib/gstreamer-1.0`. Runtime-layer dependencies (`gstreamer1.0-plugins-*, libjson-glib-1.0-0`) are installed so the plugin can be inspected immediately (`gst-inspect-1.0 imagereprojection`).

## Runtime Properties
- `config-path` (string, required): Path to the JSON config exported by the ROS `image_reprojection` node.
- `projection-mode` (enum): `auto` (default), `planar`, or `equirect`. In `auto`, planar is chosen if enabled in the config; otherwise the equirectangular output is produced.

## Example Usage
1. Export the configuration from the ROS node by setting the parameter `output.gstreamer.config_export_path` (e.g. `/tmp/reprojection.json`).
2. Launch a GStreamer pipeline that feeds each camera stream into the plugin’s request pads in the same order as listed in the JSON. Example with two test sources and the planar projection:

```bash
for p in planar equirect; do
  GST_PLUGIN_PATH=/opt/gstreamer/lib/gstreamer-1.0 gst-launch-1.0 -e \
    imagereprojection name=reproj config-path=/work/reiher/git/its-modules/perception/image_reprojection/config.json projection-mode=$p ! \
    videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 key-int-max=30 ! h264parse ! mp4mux faststart=true ! filesink location=/work/reiher/git/its-modules/perception/image_reprojection/$p.mp4 \
    videotestsrc pattern=smpte is-live=true num-buffers=150 ! video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! queue ! reproj.sink_0 \
    videotestsrc pattern=ball is-live=true num-buffers=150 ! video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! queue ! reproj.sink_1 \
    videotestsrc pattern=checkers-8 is-live=true num-buffers=150 ! video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! queue ! reproj.sink_2 \
    videotestsrc pattern=zone-plate is-live=true num-buffers=150 ! video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! queue ! reproj.sink_3 \
    videotestsrc pattern=snow is-live=true num-buffers=150 ! video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! queue ! reproj.sink_4 \
    videotestsrc pattern=red is-live=true num-buffers=150 ! video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! queue ! reproj.sink_5 \
    videotestsrc pattern=green is-live=true num-buffers=150 ! video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! queue ! reproj.sink_6 \
    videotestsrc pattern=blue is-live=true num-buffers=150 ! video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! queue ! reproj.sink_7
done
```

The first `videotestsrc` is implicitly connected to `reproj.sink_0` because it is upstream of the element; the second source is explicitly linked to `reproj.sink_1`. Add more `sink_N` pads as required.

For equirectangular output, set `projection-mode=equirect` and ensure the JSON enables that projection.

## Notes & Limitations
- Input buffers must be BGR with tightly packed rows; conversion can be done upstream with `videoconvert` if needed.
- The plugin assumes static rigs – intrinsics and transforms are fixed by the configuration. Generate a new JSON if the physical setup changes.
- Timestamp selection follows the first available input buffer. Optionally insert `rtpjitterbuffer`/`queue` stages to align feeds before the plugin.
