# image_reprojection - Agent Notes

This repository contains image reprojection and stitching tools for ROS 2 and
GStreamer. The ROS 2 package ingests multiple calibrated camera streams and
publishes one or more virtual stitched outputs. The GStreamer plugin consumes a
JSON configuration exported by the ROS 2 node and applies the same reprojection
logic to raw BGR video streams.

Supported projection modes:

- Planar pinhole mosaic: projection onto a plane at finite depth.
- Equirectangular panorama: spherical lat/lon mapping.

Both projection modes can be enabled independently.

## Repository Layout

- `README.md` - top-level project overview.
- `image_reprojection/` - ROS 2 C++ component package and executable.
- `image_reprojection/include/image_reprojection/image_reprojection.hpp` - ROS node interface and state.
- `image_reprojection/src/image_reprojection.cpp` - subscriptions, synchronization,
  reprojection, blending, TF caching, and JSON export.
- `image_reprojection/config/params.yml` - example ROS parameters.
- `image_reprojection/launch/image_reprojection_launch.py` - launch file for the ROS node.
- `gst_image_reprojection/` - standalone GStreamer plugin.
- `gst_image_reprojection/src/gstimagereprojection.c` - GStreamer element implementation.
- `docker/` - development container dependency hooks.

## Build and Run

From the ROS workspace root:

```bash
colcon build --packages-select image_reprojection
source install/setup.bash
ros2 run image_reprojection image_reprojection --ros-args --params-file PATH/TO/params.yml
```

The repository-level development build in the README uses:

```bash
colcon build
colcon test
colcon test-result --verbose
```

The GStreamer plugin is built separately with CMake from `gst_image_reprojection/`.
See `gst_image_reprojection/README.md` for the install path and example pipeline.

## ROS Node Behavior

- One `image_transport` image subscription per input topic.
- One reliable `sensor_msgs/msg/CameraInfo` subscription per input.
- Input image transport is configured with `input.<IMAGE_TOPIC>.image_transport`
  and defaults to `raw`.
- CameraInfo is treated as static. Images are ignored until the corresponding
  CameraInfo has been received.
- Camera intrinsics and frame IDs are latched from CameraInfo.
- Output images are published as BGR8 through `image_transport`; output
  CameraInfo is published through regular ROS publishers.

## Synchronization Modes

`params.sync_mode` controls how input frames are assembled:

- `wait_all` (aliases: `all`, `sync`) groups images into timestamp buckets and
  processes a bucket once all configured cameras are present within
  `params.frame_time_tolerance`.
- `lead_latest` (aliases: `lead`, `lead_image`) processes whenever camera 0
  arrives, using the latest available images from the other cameras if their
  stamps are within tolerance.

Old partial `wait_all` buckets are removed after `params.frame_timeout`.

## Transforms and Caching

- Transforms are looked up from each input camera frame to each projection target frame.
- Empty projection target frames default to the first usable input camera frame.
- By default, TF is treated as static. The node looks up time 0 transforms,
  caches them, and precomputes per-camera warp maps.
- If a static transform is not ready when CameraInfo arrives, lookup is retried during frame processing.
- Set `params.recompute_every_frame: true` for dynamic rigs. This disables
  warp-map reuse and performs stamped TF lookups for each processed frame.

## Projection and Blending

For each output pixel, every camera that sees the target ray contributes a
bilinearly sampled BGR color and a per-camera weight. The final pixel is:

```text
blended = (1 - blend_factor) * dominant + blend_factor * weighted_average
```

`dominant` is the color from the camera with the largest weight at that pixel.
`weighted_average` is normalized by total weight.

### Planar Pinhole Mosaic

- Target frame: `output.projection.planar.optical_frame_id`; defaults to the
  first usable input frame if empty.
- Output size: `width`, `height`.
- Horizontal FOV: `fov_x` in degrees, clamped to `[1, 179]`.
- Square pixels are assumed:
  - `fx = fy = (width / 2) / tan(fov_x / 2)`
  - `cx = width / 2`
  - `cy = height / 2`
- Projection plane: `depth` meters along `+Z` in the target frame.
- Blending: `blend_factor` clamped to `[0, 1]`.
- Output CameraInfo uses `distortion_model = "plumb_bob"` and the computed
  pinhole intrinsics.
- Static precompute includes per-column `x_norm`, per-row `y_norm`, and
  per-camera warp maps when cached TF is available.

### Equirectangular Panorama

- Target frame: `output.projection.equirectangular.optical_frame_id`; defaults
  to the first usable input frame if empty.
- Output size: `width`, `height`.
- Horizontal FOV: `fov_x` in degrees, clamped to `[1, 360]`.
- Vertical FOV is inferred linearly:
  - `vfov = hfov * (height / width)`, clamped to `(0, pi]`.
- Sampling sphere: `radius` meters along ray directions.
- Blending: `blend_factor` clamped to `[0, 1]`.
- Output CameraInfo uses `distortion_model = "equirectangular"`.
- `K[0,0]` stores horizontal FOV in radians and `K[1,1]` stores vertical FOV
  in radians for consumers.
- Static precompute includes sin/cos tables for latitude and longitude plus
  per-camera warp maps when cached TF is available.

## Parameters

Use `image_reprojection/config/params.yml` as the reference structure.

```yaml
input:
  image_topics: [<image_topic>, ...]
  <image_topic>:
    image_transport: raw
    camera_info_topic: <camera_info_topic>

output:
  projection:
    planar:
      enabled: true
      image_topic: <planar_image_topic>
      camera_info_topic: <planar_camera_info_topic>
      optical_frame_id: <target_frame_or_empty>
      width: 1280
      height: 720
      depth: 1.0
      fov_x: 90.0
      blend_factor: 1.0
    equirectangular:
      enabled: false
      image_topic: <equirectangular_image_topic>
      camera_info_topic: <equirectangular_camera_info_topic>
      optical_frame_id: <target_frame_or_empty>
      width: 2048
      height: 1024
      radius: 1.0
      fov_x: 360.0
      blend_factor: 1.0
  gstreamer:
    config_export_path: ""

params:
  sync_mode: wait_all
  transform_timeout: 0.05
  frame_timeout: 1.0
  frame_time_tolerance: 0.005
  recompute_every_frame: false
```

At least one projection must be enabled.

## Image Formats

The ROS node decodes images manually without `cv_bridge`.

Supported input encodings:

- `bgr8`
- `rgb8`
- `bgra8`
- `rgba8`
- `mono8`

Input row stride is respected. Each accepted image is copied or converted into a
contiguous BGR8 buffer for reprojection and bilinear sampling.

The GStreamer plugin currently accepts and produces raw BGR video caps.

## GStreamer Config Export

`output.gstreamer.config_export_path` enables JSON export from the ROS node. The
export is written once all CameraInfo data and required cached transforms are
available. It includes:

- camera count and per-camera intrinsics;
- input image and CameraInfo topic names;
- input frame IDs;
- planar and equirectangular transform matrices;
- enabled projection metadata, dimensions, FOV/intrinsics, depth or radius, and
  blend factor.

The GStreamer element `imagereprojection` uses this file through its `config-path`
property. Its `projection-mode` property accepts `auto`, `planar`, and
`equirectangular`; `auto` prefers planar when enabled.

## Logging and Timing

The ROS node logs:

- startup projection and sync mode summary;
- subscription setup;
- ignored images before CameraInfo is ready;
- partial frame readiness in debug logs;
- missing TF or projection failures;
- per-frame publish summaries with sync wait, transform lookup, projection time,
  total processing latency, and cameras used.

## Dependencies

Keep dependency declarations synchronized with source changes.

ROS package dependencies currently include:

- `ament_cmake`
- `image_transport`
- `rclcpp`
- `rclcpp_components`
- `sensor_msgs`
- `tf2`
- `tf2_geometry_msgs`
- `tf2_ros`

Runtime dependency:

- `image_transport_plugins`

The GStreamer plugin depends on GStreamer, GStreamer Base/Video, JSON-GLib, and
`libm`.

## Tips for Extending

- Add projection modes by following the existing pattern: parameters, output
  publishers, CameraInfo setup, direction/precompute tables, TF lookup/cache
  handling, reprojection method, and optional GStreamer config fields.
- Keep cached precompute paths and `recompute_every_frame` behavior aligned.
- Update both `package.xml` and `CMakeLists.txt` when adding or removing ROS dependencies.
- Update the GStreamer plugin and JSON export together when changing the exported schema.
- Prefer focused tests or reproducible launch/config examples when changing
  synchronization, TF behavior, projection geometry, or image format handling.
