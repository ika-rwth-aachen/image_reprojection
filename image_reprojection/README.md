# `image_reprojection`

Reprojects multiple camera images using various projection methods

## Nodes

### `image_reprojection`

```mermaid
flowchart LR
    NODE("image_reprojection")
    S0:::hidden -->|input.image_topics_0| NODE
    S1:::hidden -->|input.image_topics_0.camera_info_topic| NODE
    S2:::hidden -->|input.image_topics_1| NODE
    S3:::hidden -->|input.image_topics_1.camera_info_topic| NODE
    S4:::hidden -->|...| NODE
    NODE -->|~/output/planar/image| P0:::hidden
    NODE -->|~/output/planar/camera_info| P1:::hidden
    NODE -->|~/output/equirectangular/image| P2:::hidden
    NODE -->|~/output/equirectangular/camera_info| P3:::hidden
    classDef hidden display: none;
```

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `input.image_topics[:]` | `sensor_msgs/msg/Image` | input images |
| `input.<IMAGE_TOPIC>.camera_info_topic` | `sensor_msgs/msg/Image` | input camera infos |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/output/planar/image` | `sensor_msgs/msg/Image` | planar projection image |
| `~/output/planar/camera_info` | `sensor_msgs/msg/CameraInfo` | planar projection camera info |
| `~/output/equirectangular/image` | `sensor_msgs/msg/Image` | equirectangular projection image |
| `~/output/equirectangular/camera_info` | `sensor_msgs/msg/CameraInfo` | equirectangular projection camera info |

#### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `input.image_topics` | `string[]` | TODO | images topics to process |
| `input.<IMAGE_TOPIC>.image_transport` | `string` | TODO | image transport type for subscription |
| `input.<IMAGE_TOPIC>.camera_info_topic` | `string` | TODO | corresponding camera info topic |
| `output.projection.planar.enabled` | `bool` | TODO | whether to enable planar projection |
| `output.projection.planar.image_topic` | `string` | TODO | output image topic |
| `output.projection.planar.camera_info_topic` | `string` | TODO | output camera info topic |
| `output.projection.planar.optical_frame_id` | `string` | TODO | frame in which projection plane is defined |
| `output.projection.planar.width` | `int` | TODO | projection width |
| `output.projection.planar.height` | `int` | TODO | projection height |
| `output.projection.planar.depth` | `float` | TODO | location of projection plane in specified frame |
| `output.projection.planar.fov_x` | `float` | TODO | horizontal field-of-view (vertical is computed via specified aspect ratio) |
| `output.projection.planar.blend_factor` | `float` | TODO | factor by how much to blend between overlapping partitions of the output |
| `output.projection.equirectangular.enabled` | `bool` | TODO | whether to enable equirectangular projection |
| `output.projection.equirectangular.image_topic` | `string` | TODO | output image topic |
| `output.projection.equirectangular.camera_info_topic` | `string` | TODO | output camera info topic |
| `output.projection.equirectangular.optical_frame_id` | `string` | TODO | frame in which projection plane is defined |
| `output.projection.equirectangular.width` | `int` | TODO | projection width |
| `output.projection.equirectangular.height` | `int` | TODO | projection height |
| `output.projection.equirectangular.radius` | `float` | TODO | radius of spherical projection plane around specified frame |
| `output.projection.equirectangular.fov_x` | `float` | TODO | horizontal field-of-view (vertical is computed via specified aspect ratio) |
| `output.projection.equirectangular.blend_factor` | `float` | TODO | factor by how much to blend between overlapping partitions of the output |
| `output.gstreamer.config_export_path` | `string` | TODO | filepath for GStreamer config export |
| `params.recompute_every_frame` | `bool` | TODO | whether to recompute projection every frame |
| `params.transform_timeout` | `float` | TODO | how long to wait for transforms |
| `params.frame_timeout` | `float` | TODO | how long to wait for frames from all inputs |
| `params.frame_time_tolerance` | `float` | TODO | how much time stamp difference to accept between inputs |
| `params.sync_mode` | `string` | TODO | `wait_all`: wait for all inputs; `lead_latest`: start publishing with leading after timeout has passed |

## Launch Files

### [`image_reprojection_launch.py`](launch/image_reprojection_launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `name` | `"image_reprojection"` | node name |
| `namespace` | `""` | node namespace |
| `params` | `os.path.join(get_package_share_directory("image_reprojection"), "config", "params.yml")` | path to parameter file |
| `log_level` | `"info"` | ROS logging level (debug, info, warn, error, fatal) |
| `use_sim_time` | `"false"` | use simulation clock |
