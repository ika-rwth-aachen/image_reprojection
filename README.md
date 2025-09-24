# image_reprojection

```bash
sudo apt install -y ros-jazzy-rmw-zenoh-cpp ros-jazzy-image-transport-plugins

# optional
ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/drivers/zed_camera/front_center/left/image_rect_color/compressed -r out:=/drivers/zed_camera/front_center/left/image_rect_color

ros2 launch image_reprojection image_reprojection_launch.py
```
