# ros2_video_streamer

A ROS2 python node for streaming video files or images to a topic.
Tailored launch file for 2024 MATE ROV competition.

## CWRUBotix MATE ROV 2024

Use the `three_camera_launch.py` launch file without any parameters to launch
three streams from the files `bottom_cam.mp4`, `front_cam.mp4`, and
`manip_cam.mp4`. These files are not included in the Git repo to save space,
so you need to manually place three video files with those names in
`src/ros2_video_streamer`.

```bash
ros2 launch ros2_video_streamer three_camera_launch.py
```

This launcher will publish the streams on `bottom_cam/image_raw`,
`front_cam/image_raw`, and `manip_cam/image_raw`.

If greater control is desired, see __Custom Usage__.

## Custom Usage

Minimally, you need to provide the `ros2_video_streamer_node_launch.py` launch
file a `type` (`video` or `image`) and `path`, or `type:=video` and `camera_name`.
`path` is the path to the content to stream (relative to the ros2_video_streamer
package directory), and `camera_name` is the name of a camera (e.g. `front_cam`)
which the launch file uses to autofill various fields, including path, which it
sets to `<camera_name>.mp4`.

```bash
ros2 launch ros2_video_streamer ros2_video_streamer_node_launch.py type:=video camera_name:=<name>
```

```bash
ros2 launch ros2_video_streamer ros2_video_streamer_node_launch.py type:=<type> path:=<path>
```

The content is published on the `/simulated_cam/image_raw` topic by default,
but specifying `camera_name` will switch the topic to `/<camera_name>/image_raw`.

## All Settings
These are all of the parameters which the launch file accepts.

* __camera_name__ - name of the camera (defaults to `simulated_cam`) which is
    used as the node namespace and autopopulates the following fields:
    * __image_topic_name__: `/<camera_name>/image_raw`
    * __info_topic_name__: `/<camera_name>/camera_info`
    * __path__: `<camera_name>.mp4`

* __node_name__ - Override the default name of the node.

* __image_topic_name__ - Override default name of the topic to publish images to

* __info_topic_name__ - Override default name of the topic to publish camera info to

* __config_file_name__ - Name of YAML file in the `config` folder. `CameraInfo` messages are published on the `~/camera_info` topic based on the content of the config file. By default, nothing is published.

* __loop__ (_true_ or _false_; defaults to _true_) - Continuously publish the source on loop

* __frame_id__ - Frame id string in the `CameraInfo` messages.

* __start__ (_int_) - Location to start publishing the source.
