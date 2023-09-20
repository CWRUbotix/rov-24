# realsense

## Overview

This package interfaces with the [Realsense](https://www.intelrealsense.com/depth-camera-d415/) D415 depth camera.

[Realsense docs](https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide)

## Usage

Launch the package with:

```bash
ros2 launch realsense realsense_launch.py
```

## Launch files

* **realsense_launch.py:** launches the Realsense interface and starts publishing frames

## Published Topics

* **`/depth_cam/image_raw`** ([sensor_msgs/Image])

    Regular color camera frames

* **`/depth_cam/camera_info`** ([sensor_msgs/CameraInfo])

    Details about the camera (frame size, etc.)

* **`/depth_cam/depth_to_color`** ([realsense2_camera_msgs/Extrinsincs])

    Orientation of the camera??? It's magic

[sensor_msgs/Image]: http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
[realsense2_camera_msgs/Extrinsincs]: https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_camera_msgs/msg/Extrinsics.msg
