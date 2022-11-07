# beginner_tutorials

## Overview

There is a publisher node and subscriber node here. The publisher node publishes the following string "Hello, world! This is Vignesh here from UMD" to a topic and the subscriber listens to the string published by the publisher node from the same topic.

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
ros2 run cpp_pubsub listener
```


## Dependencies

1. ROS humble
2. rclcpp
3. std_msgs