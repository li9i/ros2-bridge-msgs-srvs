![alt text](https://github.com/li9i/ros2-bridge-msgs-srvs/blob/master/architecture.png?raw=true)

# Contents:

- `0.ros1_bridge_foxy_source` The source code of the bridge we will be using
- `1.ros_vanilla_bridge_test` Communication tests with built-in types of messages between two nodes
- `2.ros_custom_bridge_test` Communication tests with custom messages between two nodes
- `3.ros_custom_bridge_and_vanilla_setup_test_msgs` Tests with custom messages between four nodes. Each is running a different ROS version+distribution, as seen in the diagram above.
- `4.ros_custom_bridge_and_vanilla_setup_test_srvs` Tests with custom services between four nodes. Each is running a different ROS version+distribution, as seen in the diagram above.

Instructions on running the last two setups may be found in their `play instructions` text file.

# Rationale

- [Bridge communication between ROS 1 and ROS 2](https://github.com/ros2/ros1_bridge/blob/master/README.md)
- Problem:
  - `ros1_bridge` needs ROS 1 and ROS 2 in the same machine [but ROS 1 is not supported in Ubuntu 22.04](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html?highlight=bridge#:\~:text=the%20release%20of%20ros%202%20humble%20(and%20rolling)%20on%20ubuntu%2022.04%20jammy%20jellyfish%20marks%20the%20first%20ros%202%20release%20on%20a%20platform%20with%20no%20official%20ros%201%20release.)
  - `ros1_bridge` cannot be installed via package manager in Ubuntu 22.04.
  - `ros1_bridge` [must be compiled from source for support of custom messages](https://github.com/ros2/ros1_bridge/blob/master/README.md#:\~:text=if%20you%20would%20like%20to%20use%20a%20bridge%20with%20other%20interfaces%20(including%20your%20own%20custom%20types)%2C%20you%20will%20have%20to%20build%20the%20bridge%20from%20source)
- Solution: Use the `osrf/ros:foxy-ros1-bridge` docker image.
- By launching the bridge all publishers to- and subscribers of- built-in topic types should be able to publish and subscribe to topics of built-in types.
- When you need [support for custom messages](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst#example-workspace-setup) clone the `foxy` branch of [the bridge](https://github.com/ros2/ros1_bridge) when building the Dockerfile. CAUTION: ROS 1 and ROS 2 packages which require custom messages need to reside in the same Docker image as the bridge but on separate workspaces (ROS 1 packages with ROS 1 packages, ROS 2 packages with ROS 2 packages, and the bridge in a workspace of its own): this because the bridge needs to source the `setup.bash` files of both workspace [(source)](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst#example-workspace-setup:~:text=Then%20build%20the%20bridge%3A).

* In the case of packages with custom messages
  * all ROS 1 packages should be `noetic` packages
  * all ROS 2 packages should be `foxy` packages
  * If a ROS 1 package can not be/is not converted to `noetic` and needs to communicate with a ROS 2 package then the bridge's ROS 2 version should be the one that corresponds to the Ubuntu distribution that corresponds to the ROS 1 version of that particular package (e.g. ROS 1 [`melodic`](https://wiki.ros.org/melodic) -> `18.04` -> ROS 2 [`dashing`](https://docs.ros.org/en/dashing/Installation.html)).
