# FTG

Sample ROS2 node implementing Follow-the-Gap

This node already works with the F1tenth Gym simulator (uses specific ROS2 topics)

## Dependencies

Surely, ROS2 foxy running on Ubuntu 20.04.
Then, you shall install the following packages


```
sudo apt install cmake ros-foxy-ackermann-msgs libyaml-cpp-dev
```

## Build and run


First, open a terminal, and source the ROS2 environment/default underlay.
```
$ source /opt/ros/foxy/setup.bash
```

Then, you can enter ROS WS, and build
```
$ colcon build
```

...re-source your environment, and run!
```
$ source install/local_setup.bash
$ ros2 launch ftg ftg_node.launch.xml
```