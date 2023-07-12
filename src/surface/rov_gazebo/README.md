# Gazebo Simulation

## 1. Build

```bash
cd ~/rov-24
colcon build
```

## 2. Run launch file

```bash
ros2 launch rov_gazebo sim_launch.py
```

## 3. Play simulation

Press play button or space bar at gazebo window

## 4. Move ROV

Don't forget to arm the ROV

See help message from terminal or use PS5 controller

## Reference

[Thruster map](https://www.ardusub.com/introduction/features.html)

[Simulation](https://www.ardusub.com/developers/sitl.html)

### Published Topics

/bottom_cam/image_raw

/front_cam/image_raw

/manip_cam/image_raw

/depth_cam/image_raw

/depth_cam/points

## Subscription Topics

/manual_control

/arm

## Notes

Might need to run this

```bash
sudo rm -rf /home/rmc/.local/lib/python3.10/site-packages/cv2/qt/plugins/
```
