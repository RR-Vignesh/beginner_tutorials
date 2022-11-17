# beginner_tutorials

## Overview

There is a publisher node and service node here. The publisher node will publish a custom string message and the service wil modify the message and send a new message.

## Build/run steps

Go to the root of the workspace "ros2_ws" then type the following in the terminal: -

```
colcon build --packages-select cpp_pubsub
```
open a new terminal and type: -

```
. install/setup.bash
ros2 run cpp_pubsub talker
```

open a new terminal and type: -

```
. install/setup.bash
ros2 run cpp_pubsub server_client
```


## Dependencies

1. ROS humble
2. rclcpp
3. std_msgs
4. rosidl_default_generators
5. rosidl_default_runtime