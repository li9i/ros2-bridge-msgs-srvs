![alt text](https://github.com/li9i/ros2-bridge-msgs-srvs/blob/master/architecture.png?raw=true)

Contents:

- `0.ros1_bridge_foxy_source` The source code of the bridge we will be using
- `1.ros_vanilla_bridge_test` Communication tests with built-in types of messages between two nodes
- `2.ros_custom_bridge_test` Communication tests with custom messages between two nodes
- `3.ros_custom_bridge_and_vanilla_setup_test_msgs` Tests with custom messages between four nodes. Each is running a different ROS version+distribution, as seen in the diagram above.
- `4.ros_custom_bridge_and_vanilla_setup_test_srvs` Tests with custom services between four nodes. Each is running a different ROS version+distribution, as seen in the diagram above.

Instructions on running the last two setups may be found in their `play instructions` text file.
