# Charuco ROS2

ROS 2 wrapper around OpenCV aruco detection and pose estimation.

## Build instruction
```bash
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO
# parameter --symlink-install is optional
colcon build --symlink-install
source install/setup.bash
```

## Launch
### Charuco detection node
```bash
ros2 launch charuco_ros2 charuco_ros2.launch.py launch_rviz:=true
```
### (Optional) Realsense node
```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```
#### Publishers

- tf2 transform from the camera frame (frame_id of the <camera_name>/color/image_raw topic) to the charuco marker frame specified by the `charuco_id` parameter
- **\<camera name\>/charuco/debug_image**: debug image with overlaid detections. Published if `publish_debug_images` is true.
- **\<camera name\>/charuco/camera_info**: debug image info (copy of **\<camera name\>/color/camera_info**). Published if `publish_debug_images` is true.


#### Subscribers
- **\<camera name\>/color/image_raw** [sensor_msgs/msg/Image]: Video stream where charuco board detection is done.

- **\<camera name\>/color/camera_info** [sensor_msgs/msg/CameraInfo] Video stream camera info.
