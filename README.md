# Universal Robot CB2 ROS2 Driver

ROS 2 node for Universal Robot CB2 Controller (https://github.com/jhu-bigss/Universal_Robots_CB2_ROS2_Driver).

You can use the ROS 2 VCS python-based tool (`sudo apt install python3-vcstool`) to download this package and its specific dependencies. Look for the `.repos` file in this repository and find the link to the raw content. Then in your ROS 2 workspace, under the `src` directory, use:
```bash
vcs import --input https://raw.githubusercontent.com/jhu-bigss/Universal_Robots_CB2_ROS2_Driver/bigss_util_v2/ur_cb2_driver.repos
```
Then go back to the root of your ROS 2 workspace and build using:
```bash
colcon build
```

Once the code is compiled, you can launch the driver using:
```bash
ros2 launch ur_bringup ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=true
```