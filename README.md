# image_reprojection

```bash
sudo apt install -y ros-jazzy-rmw-zenoh-cpp ros-jazzy-image-transport-plugins

# optional
ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/drivers/zed_camera/front_center/left/image_rect_color/compressed -r out:=/drivers/zed_camera/front_center/left/image_rect_color

ros2 launch image_reprojection image_reprojection_launch.py
```

## TODO

- image_transport publisher
- distortion model "equirectangular" is unknown; does it even make sense to publish a CameraInfo for equirectangular projection?
- allow to specify arbitrary number of arbitrary projections, e.g., one planar to front, one planar for BEV
- performance
  - multi-threaded (caution: are callbacks thread-safe?)
  - OpenCV? (cv_bridge?)
  - GPU?
