# Universal Robot CB2 ROS2 Driver

ROS 2 node for Universal Robot CB2 Controller (https://github.com/jhu-bigss/Universal_Robots_CB2_ROS2_Driver).

You can use the ROS 2 VCS python-based tool (`sudo apt install python3-vcstool`) to download this package and its specific dependencies. Look for the `.repos` file in this repository and find the link to the raw content. Then in your ROS 2 workspace, under the `src` directory, use:
```bash
vcs import --input https://raw.githubusercontent.com/jhu-bigss/Universal_Robots_CB2_ROS2_Driver/main/ur_cb2_driver.repos
```
Make sure all the dependencies are met by running:

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "reflexxes_type2"
```

Make sure fortran is installed in order for `cisstNetlib` to be build. See cisstNetlib [Issue#5](https://github.com/jhu-cisst/cisstNetlib/issues/5#issuecomment-1169452231)

At the root of your ROS 2 workspace, build using:
```bash
colcon build
```

Once the code is compiled, you can launch the driver using:
```bash
ros2 launch ur_cb2_bringup ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT>
```

## Install URSim 1.8.16941
Dowload [URSIM 1.8.16941](https://www.universal-robots.com/download/software-cb-series/simulator-non-linux/offline-simulator-cb-series-non-linux-ursim-1816941/) and run it on a virtual machine. Open a terminal and run `ifconfig`, then you should see the robot's IP.
