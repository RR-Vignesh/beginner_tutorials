[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

# beginner_tutorials

## Overview

There is a publisher node and service node here. The publisher node will publish a custom string message and the service wil modify the message in the topic.

## Build/run steps

Go to the root of the workspace "ros2_ws" then type the following in the terminal: -

```
cd ~/ros2_ws/src
git clone <repo>
cd .. 
colcon build
. install/setup.bash
```

After making the required changes in the package (if any)
```
colcon build --packages-select cpp_pubsub
. install/setup.bash
```

Run Publisher and subscriber separately: -

```
. install/setup.bash
ros2 run cpp_pubsub talker --ros-args --log-level debug
```

Open new terminal: -

```
. install/setup.bash
ros2 run cpp_pubsub listener --ros-args --log-level debug
```

Run with launch file: -

```
. install/setup.bash
ros2 launch cpp_pubsub custom_launch.yaml frequency:=1
```

## Dependencies

1. ROS humble
2. rclcpp
3. std_msgs
4. rosidl_default_generators
5. rosidl_default_runtime