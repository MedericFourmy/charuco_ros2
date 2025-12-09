# Charuco ROS2

## Run node
`ros2 run charuco_ros2 charuco_ros2_node`

## Realsense Demo
`ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true pointcloud.enable:=true`  

Higher resolution (low frequency):  
- rs435:  
`ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true pointcloud.enable:=true rgb_camera.color_profile:=1280x720x6 depth_module.depth_profile:=1280x720x6 depth_module.infra_profile:=1280x720x6`
- rs455:  
`ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true pointcloud.enable:=true rgb_camera.color_profile:=1280x720x5 depth_module.depth_profile:=1280x720x5 depth_module.infra_profile:=1280x720x5`
