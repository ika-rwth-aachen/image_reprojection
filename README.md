# image_reprojection

Reprojects multiple camera images to the image plane of a virtual output camera

- [Container Images](#container-images)
- [image_reprojection](#image_reprojection)


### Container Images

| Description | Image:Tag | Default Command |
| --- | --- | -- |
| latest | `gitlab.ika.rwth-aachen.de:5050/fb-fi/its-modules/perception/image_reprojection:latest` | `ros2 launch image_reprojection image_reprojection_launch.py` |


## `image_reprojection`

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `input.image_topics[:]` | `sensor_msgs/msg/Image` | input images |
| `input.<IMAGE_TOPIC>.camera_info_topic` | `sensor_msgs/msg/Image` | input camera infos |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/output/planar/image` | `sensor_msgs/msg/Image` | planar projection image |
| `~/output/planar/camera_info` | `sensor_msgs/msg/CameraInfo` | planar projection camera info |
| `~/output/equirectangular/image` | `sensor_msgs/msg/Image` | equirectangular projection image |
| `~/output/equirectangular/camera_info` | `sensor_msgs/msg/CameraInfo` | equirectangular projection camera info |

### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `input.image_topics` | `string[]` | images topics to process |
| `input.<IMAGE_TOPIC>.image_transport` | `string` | image transport type for subscription |
| `input.<IMAGE_TOPIC>.camera_info_topic` | `string` | corresponding camera info topic |
| `output.projection.planar.enabled` | `bool` | whether to enable planar projection |
| `output.projection.planar.image_topic` | `string` | output image topic |
| `output.projection.planar.camera_info_topic` | `string` | output camera info topic |
| `output.projection.planar.optical_frame_id` | `string` | frame in which projection plane is defined |
| `output.projection.planar.width` | `int` | projection width |
| `output.projection.planar.height` | `int` | projection height |
| `output.projection.planar.depth` | `float` | location of projection plane in specified frame |
| `output.projection.planar.fov_x` | `float` | horizontal field-of-view (vertical is computed via specified aspect ratio) |
| `output.projection.planar.blend_factor` | `float` | factor by how much to blend between overlapping partitions of the output |
| `output.projection.equirectangular.enabled` | `bool` | whether to enable equirectangular projection |
| `output.projection.equirectangular.image_topic` | `string` | output image topic |
| `output.projection.equirectangular.camera_info_topic` | `string` | output camera info topic |
| `output.projection.equirectangular.optical_frame_id` | `string` | frame in which projection plane is defined |
| `output.projection.equirectangular.width` | `int` | projection width |
| `output.projection.equirectangular.height` | `int` | projection height |
| `output.projection.equirectangular.radius` | `float` | radius of spherical projection plane around specified frame |
| `output.projection.equirectangular.fov_x` | `float` | horizontal field-of-view (vertical is computed via specified aspect ratio) |
| `output.projection.equirectangular.blend_factor` | `float` | factor by how much to blend between overlapping partitions of the output |
| `output.gstreamer.config_export_path` | `string` | filepath for GStreamer config export |
| `params.recompute_every_frame` | `bool` | whether to recompute projection every frame |
| `params.transform_timeout` | `float` | how long to wait for transforms |
| `params.frame_timeout` | `float` | how long to wait for frames from all inputs |
| `params.frame_time_tolerance` | `float` | how much time stamp difference to accept between inputs |
| `params.sync_mode` | `string` | `wait_all`: wait for all inputs; `lead_latest`: start publishing with leading after timeout has passed |
