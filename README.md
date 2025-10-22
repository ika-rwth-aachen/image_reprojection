# image_reprojection

ROS node and corresponding GStreamer pluging for reprojecting multiple camera images using various projection methods

> [!WARNING]  
> 99% of this project has been vibe-coded.

- [Container Images](#container-images)
- [Projection Methods](#projection-methods)
- [ROS Node: image_reprojection](#ros-node-image_reprojection)
- [GStreamer Plugin: imagereprojection](#gstreamer-plugin-imageprojection)


## Container Images

| Description | Image:Tag | Default Command |
| --- | --- | -- |
| latest ROS node and GStreamer plugin | `gitlab.ika.rwth-aachen.de:5050/fb-fi/its-modules/perception/image_reprojection:latest` | `ros2 launch image_reprojection image_reprojection_launch.py` |


## Projection Methods

### Planar Projection

All input images are projected to a planar surface at `output.projection.planar.depth` in the frame `output.projection.planar.frame_id`.

![./assets/planar.png](./assets/planar.png)

### Equirectangular Projection

All input images are projected to a sphere with radius `output.projection.equirectangular.radius` in the frame `output.projection.equirectangular.frame_id` and then unrolled to a 2D image using [equirectangular projection](https://www.researchgate.net/profile/Kailun-Yang/publication/347957084/figure/fig1/AS:974408793481217@1609328568414/llustration-of-the-equirectangular-projection_W640.jpg).

![./assets/equirectangular.png](./assets/equirectangular.png)


## ROS Node: `image_reprojection`

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


## GStreamer Plugin: `imageprojection`

The GStreamer pluging mirrors the ROS node and projects multiple camera feeds into a single stitched output using one of the supported projection methods. It consumes the JSON configuration exported by the ROS node (see `output.gstreamer.config_export_path`) so that ROS and GStreamer produce identical outputs.

### Properties

| Parameter | Type | Description |
| --- | --- | --- |
| `config-path` | `string` | filepath to config exported by ROS Node (see `output.gstreamer.config_export_path`) |
| `projection-mode` | `string` | projection method (`auto`, `planar`, `equirectangular`) |

### Example Pipeline

The example pipeline processes 8 dummy input streams (for 5s) and exports the output to a file. The example config is attached.

<details>
<summary>config.json</summary>

```json
{
  "generated_by": "image_reprojection",
  "camera_count": 8,
  "cameras": [
    {
      "index": 0,
      "name": "input_0",
      "image_topic": "/drivers/zed_camera/front_center/left/image_rect_color",
      "camera_info_topic": "/drivers/zed_camera/front_center/left/camera_info",
      "frame_id": "front_center_left_camera_optical_frame",
      "intrinsics": {
        "fx": 858.6144409,
        "fy": 858.6144409,
        "cx": 628.9880371,
        "cy": 375.044342,
        "width": 1280,
        "height": 720
      },
      "planar_transform": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
      "equirectangular_transform": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    },
    {
      "index": 1,
      "name": "input_1",
      "image_topic": "/drivers/zed_camera/front_left/left/image_rect_color",
      "camera_info_topic": "/drivers/zed_camera/front_left/left/camera_info",
      "frame_id": "front_left_left_camera_optical_frame",
      "intrinsics": {
        "fx": 490.3730164,
        "fy": 490.3730164,
        "cx": 657.3430176,
        "cy": 348.342865,
        "width": 1280,
        "height": 720
      },
      "planar_transform": [0.7052780781, -0.05601773825, 0.7067141187, 0.5348981121, 0.07186262408, 0.9973875211, 0.007341383958, 0.07559561823, -0.7052790907, 0.04560861387, 0.7074611357, -0.3438772089, 0, 0, 0, 1],
      "equirectangular_transform": [0.7052780781, -0.05601773825, 0.7067141187, 0.5348981121, 0.07186262408, 0.9973875211, 0.007341383958, 0.07559561823, -0.7052790907, 0.04560861387, 0.7074611357, -0.3438772089, 0, 0, 0, 1]
    },
    {
      "index": 2,
      "name": "input_2",
      "image_topic": "/drivers/zed_camera/front_right/left/image_rect_color",
      "camera_info_topic": "/drivers/zed_camera/front_right/left/camera_info",
      "frame_id": "front_right_left_camera_optical_frame",
      "intrinsics": {
        "fx": 853.3219604,
        "fy": 853.3219604,
        "cx": 622.7871704,
        "cy": 351.5395508,
        "width": 1280,
        "height": 720
      },
      "planar_transform": [0.7206790637, 0.0476010503, -0.6916327256, -0.5305931279, -0.06165291044, 0.9980877278, 0.004450429028, 0.09300954174, 0.6905219806, 0.03943383946, 0.7222356725, -0.4663019134, 0, 0, 0, 1],
      "equirectangular_transform": [0.7206790637, 0.0476010503, -0.6916327256, -0.5305931279, -0.06165291044, 0.9980877278, 0.004450429028, 0.09300954174, 0.6905219806, 0.03943383946, 0.7222356725, -0.4663019134, 0, 0, 0, 1]
    },
    {
      "index": 3,
      "name": "input_3",
      "image_topic": "/drivers/zed_camera/mid_left/left/image_rect_color",
      "camera_info_topic": "/drivers/zed_camera/mid_left/left/camera_info",
      "frame_id": "mid_left_left_camera_optical_frame",
      "intrinsics": {
        "fx": 492.7843323,
        "fy": 492.7843323,
        "cx": 650.1942749,
        "cy": 342.7900696,
        "width": 1280,
        "height": 720
      },
      "planar_transform": [-0.004364204577, -0.06242730173, 0.998039972, 1.38415597, 0.3382162205, 0.9391395505, 0.06022202973, 0.1963577516, -0.9410583095, 0.3378161285, 0.01701532892, -0.5868134738, 0, 0, 0, 1],
      "equirectangular_transform": [-0.004364204577, -0.06242730173, 0.998039972, 1.38415597, 0.3382162205, 0.9391395505, 0.06022202973, 0.1963577516, -0.9410583095, 0.3378161285, 0.01701532892, -0.5868134738, 0, 0, 0, 1]
    },
    {
      "index": 4,
      "name": "input_4",
      "image_topic": "/drivers/zed_camera/mid_right/left/image_rect_color",
      "camera_info_topic": "/drivers/zed_camera/mid_right/left/camera_info",
      "frame_id": "mid_right_left_camera_optical_frame",
      "intrinsics": {
        "fx": 494.1058044,
        "fy": 494.1058044,
        "cx": 662.2318726,
        "cy": 352.0738831,
        "width": 1280,
        "height": 720
      },
      "planar_transform": [0.01061994877, 0.08437551445, -0.9963774331, -1.271145576, -0.3608254252, 0.929622769, 0.07487669769, 0.3397872176, 0.9325729083, 0.3587231243, 0.04031737705, -0.749896989, 0, 0, 0, 1],
      "equirectangular_transform": [0.01061994877, 0.08437551445, -0.9963774331, -1.271145576, -0.3608254252, 0.929622769, 0.07487669769, 0.3397872176, 0.9325729083, 0.3587231243, 0.04031737705, -0.749896989, 0, 0, 0, 1]
    },
    {
      "index": 5,
      "name": "input_5",
      "image_topic": "/drivers/zed_camera/rear_left/left/image_rect_color",
      "camera_info_topic": "/drivers/zed_camera/rear_left/left/camera_info",
      "frame_id": "rear_left_left_camera_optical_frame",
      "intrinsics": {
        "fx": 488.573761,
        "fy": 488.573761,
        "cx": 648.9766846,
        "cy": 341.7672119,
        "width": 1280,
        "height": 720
      },
      "planar_transform": [-0.7128496617, -0.05715469264, 0.6989840491, 1.658967001, 0.2364513458, 0.918731364, 0.3162648287, 0.8521728306, -0.6602545879, 0.3907249953, -0.641403038, -2.266268509, 0, 0, 0, 1],
      "equirectangular_transform": [-0.7128496617, -0.05715469264, 0.6989840491, 1.658967001, 0.2364513458, 0.918731364, 0.3162648287, 0.8521728306, -0.6602545879, 0.3907249953, -0.641403038, -2.266268509, 0, 0, 0, 1]
    },
    {
      "index": 6,
      "name": "input_6",
      "image_topic": "/drivers/zed_camera/rear_right/left/image_rect_color",
      "camera_info_topic": "/drivers/zed_camera/rear_right/left/camera_info",
      "frame_id": "rear_right_left_camera_optical_frame",
      "intrinsics": {
        "fx": 496.3605042,
        "fy": 496.3605042,
        "cx": 652.3399048,
        "cy": 355.5737,
        "width": 1280,
        "height": 720
      },
      "planar_transform": [-0.7232397536, 0.05252587248, -0.6885966102, -1.383099375, -0.2423295842, 0.9143985113, 0.3242710828, 0.9261129408, 0.6466843368, 0.4013930682, -0.6486007812, -2.406698339, 0, 0, 0, 1],
      "equirectangular_transform": [-0.7232397536, 0.05252587248, -0.6885966102, -1.383099375, -0.2423295842, 0.9143985113, 0.3242710828, 0.9261129408, 0.6466843368, 0.4013930682, -0.6486007812, -2.406698339, 0, 0, 0, 1]
    },
    {
      "index": 7,
      "name": "input_7",
      "image_topic": "/drivers/zed_camera/rear_center/left/image_rect_color",
      "camera_info_topic": "/drivers/zed_camera/rear_center/left/camera_info",
      "frame_id": "rear_center_left_camera_optical_frame",
      "intrinsics": {
        "fx": 494.2579651,
        "fy": 494.2579651,
        "cx": 647.6151733,
        "cy": 351.9077148,
        "width": 1280,
        "height": 720
      },
      "planar_transform": [-0.9999469676, -0.01006741982, 0.002170038852, 0.1730240376, -0.009930620571, 0.9983914513, 0.055820182, 0.03150858637, -0.002728513446, 0.05579567189, -0.9984384799, -2.939043181, 0, 0, 0, 1],
      "equirectangular_transform": [-0.9999469676, -0.01006741982, 0.002170038852, 0.1730240376, -0.009930620571, 0.9983914513, 0.055820182, 0.03150858637, -0.002728513446, 0.05579567189, -0.9984384799, -2.939043181, 0, 0, 0, 1]
    }
  ],
  "planar": {
    "enabled": true,
    "frame_id": "front_center_left_camera_optical_frame",
    "width": 2560,
    "height": 720,
    "fx": 739.0083446,
    "fy": 739.0083446,
    "cx": 1280,
    "cy": 360,
    "depth": 10,
    "blend_factor": 0
  },
  "equirectangular": {
    "enabled": true,
    "frame_id": "front_center_left_camera_optical_frame",
    "width": 2560,
    "height": 640,
    "hfov_rad": 6.283185307,
    "vfov_rad": 1.570796327,
    "radius": 10,
    "blend_factor": 0
  }
}
```

</details>

```bash
PROJECTION_MODE="equirectangular"
CONFIG_PATH="./config.json"
OUTPUT_PATH="./output.mp4"
gst-launch-1.0 -e \
    imagereprojection name=proj config-path=${CONFIG_PATH} projection-mode=${PROJECTION_MODE} ! \
        videoconvert ! \
        x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 key-int-max=30 ! \
        h264parse ! \
        mp4mux faststart=true ! \
        filesink location=${OUTPUT_PATH} \
    videotestsrc pattern=smpte is-live=true num-buffers=150 ! \
        video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! \
        queue ! \
        proj.sink_0 \
    videotestsrc pattern=ball is-live=true num-buffers=150 ! \
        video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! \
        queue ! \
        proj.sink_1 \
    videotestsrc pattern=checkers-8 is-live=true num-buffers=150 ! \
        video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! \
        queue ! \
        proj.sink_2 \
    videotestsrc pattern=zone-plate is-live=true num-buffers=150 ! \
        video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! \
        queue ! \
        proj.sink_3 \
    videotestsrc pattern=snow is-live=true num-buffers=150 ! \
        video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! \
        queue ! \
        proj.sink_4 \
    videotestsrc pattern=red is-live=true num-buffers=150 ! \
        video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! \
        queue ! \
        proj.sink_5 \
    videotestsrc pattern=green is-live=true num-buffers=150 ! \
        video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! \
        queue ! \
        proj.sink_6 \
    videotestsrc pattern=blue is-live=true num-buffers=150 ! \
        video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 ! \
        queue ! \
        proj.sink_7
```
