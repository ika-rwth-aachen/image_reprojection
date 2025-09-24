# image_reprojection — Agent Notes

This package provides a ROS 2 C++ component node that stitches multiple camera images into one or more virtual outputs by reprojection. Two projection modes are supported and can be enabled independently:

- Planar pinhole mosaic (projection onto a plane at finite depth)
- Equirectangular panorama (spherical, lat/long mapping)

The node is optimized for static CameraInfo and TF: intrinsics, direction tables, and transforms are cached; only images are synchronized by timestamp tolerance.

## Layout

- image_reprojection/include/image_reprojection/image_reprojection.hpp — node interface and state
- image_reprojection/src/image_reprojection.cpp — implementation (subs, aggregation, reprojection, blending)
- image_reprojection/config/params.yml — example params (new structure)
- image_reprojection/CMakeLists.txt, package.xml — build metadata

Build (from workspace root at /docker-ros/ws):

- colcon build --packages-select image_reprojection
- source install/setup.bash

Run:

- ros2 run image_reprojection image_reprojection --ros-args --params-file PATH/TO/params.yml

## Subscriptions and Aggregation

- One Image subscription per input topic (SensorDataQoS)
- One CameraInfo subscription per input (reliable QoS)
- CameraInfo is latched (treated as static). Images are ignored until the corresponding CameraInfo is seen once.
- Images are grouped into buckets keyed by stamp (nanoseconds) with tolerance `params.frame_time_tolerance`. A bucket that has all N cameras is processed once; old partial buckets are dropped after `params.frame_timeout`.

## Transforms and Caching

- Assumes transforms camera→target frame are static.
- On first CameraInfo for a camera, the node tries to cache TF at time 0 for both projections’ target frames. If unavailable, it falls back to stamped lookup during processing.
- Set `params.recompute_every_frame: true` to force stamped TF lookups every frame (dynamic rigs), otherwise cached transforms are reused.

## Projections

Both projections use the same accumulation + blending strategy: each output pixel gathers bilinearly sampled colors from all cameras that see it and keeps a per-camera weight. The final pixel is

```
blended = (1 − blend_factor) * dominant + blend_factor * weighted_average
```

where dominant is the color from the camera with the largest weight at that pixel; weighted_average normalizes by total weight.

### Planar pinhole mosaic

- Target frame: `output.projection.planar.optical_frame_id` (defaults to first camera if empty)
- Output size: `width`, `height`
- FOV: `fov_x` (deg). Square pixels assumed; intrinsics computed as
  - fx = fy = (width/2) / tan(fov_x/2), cx = width/2, cy = height/2
- Depth: `depth` (meters) — distance of the projection plane along +Z in the target frame
- Blending: `blend_factor` in [0,1]
- CameraInfo: plumb_bob; K/P filled from computed intrinsics
- Precompute: per-column x_norm, per-row y_norm once

### Equirectangular panorama

- Target frame: `output.projection.equirectangular.optical_frame_id` (defaults to first camera if empty)
- Output size: `width`, `height`
- Horizontal FOV: `fov_x` (deg). Vertical is inferred linearly to avoid stretching:
  - vfov = hfov * (height/width), clamped to (0, π]
- Radius: `radius` (meters) — sampling sphere radius along ray directions
- Blending: `blend_factor` in [0,1]
- CameraInfo: `distortion_model = "equirectangular"`; K(0,0) stores hfov (rad), K(1,1) stores vfov (rad) for consumers
- Precompute: sin/cos(lat) per row and sin/cos(lon) per column once

## Parameters (new structure)

See config/params.yml for a working example.

- input:
  - image_topics: [<image_topic> ...]
  - <image_topic>:
    - camera_info_topic: <string>

- output:
  - projection:
    - planar:
      - enabled: bool
      - image_topic: string
      - camera_info_topic: string
      - optical_frame_id: string (defaults to first camera)
      - width: int, height: int
      - depth: double (meters)
      - fov_x: double (deg)
      - blend_factor: double in [0,1]
    - equirectangular:
      - enabled: bool
      - image_topic: string
      - camera_info_topic: string
      - optical_frame_id: string (defaults to first camera)
      - width: int, height: int
      - radius: double (meters)
      - fov_x: double (deg); vfov inferred by aspect
      - blend_factor: double in [0,1]

- params:
  - sync_queue_size: int (internal buffering for image buckets)
  - transform_timeout: double (s) TF lookup timeout
  - frame_timeout: double (s) — drop old partial buckets
  - frame_time_tolerance: double (s) — max skew when grouping images
  - recompute_every_frame: bool (default false) — redo TF lookups each frame

## Image formats

Manual decoding (no cv_bridge dependency): BGR8, RGB8, BGRA8, RGBA8, MONO8. Stride is respected; rows are copied/converted into a contiguous BGR8 buffer for reprojection and sampling.

## Logging & Timing

- Partial frame readiness (x/y)
- Per‑projection processing times (ms)
- Success/failure per projection and overall frame publish status

## Dependencies

- always keep package.xml and CMakeLists.txt up-to-date
- rclcpp, rclcpp_components, sensor_msgs, tf2, tf2_ros, tf2_geometry_msgs
- message_filters is still listed in CMake but not used by the current implementation (safe to remove if desired)

## Tips for extending

- Add new projection modes by following the pattern: per‑projection parameters, precompute direction fields, reproject method, publishers, and CameraInfo.
- For dynamic rigs, prefer `recompute_every_frame: true` and consider throttling or GPU offload if performance is tight.
- Keep precompute + cached TF to minimize per‑frame cost.

