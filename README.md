# Universal_Robots_CB2_ROS2_Driver

## You can use this driver with the official [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), which you can install with the following command:

```bash
sudo apt-get install ros-<distro>-ur-description
```

However, you need to modify the URDF files within the package:

```
colcon_cd ur_description

# go to the directory
urdf
├── ur_macro.xacro
├── ur.ros2_control.xacro
└── ur.urdf.xacro

```
### Modify `ur.ros2_control.xacro`

Change the plugin name from ur_robot_driver to ur_robot_driver_cb2 as the following line:

```xml
<plugin>ur_robot_driver_cb2/URPositionHardwareInterface</plugin>
```

Delete the `<sensor>` and `gpio` tages:

```xml
<xacro:unless value="${sim_gazebo or sim_ignition}">
  <sensor name="${tf_prefix}tcp_fts_sensor">
    ...
  </sensor>

  <gpio name="${tf_prefix}speed_scaling">
    ...
  </gpio>

  <gpio name="${tf_prefix}gpio">
    ...
    ...
    ...
  </gpio>

</xacro:unless>
```

Because current CB2 driver doesn't support TCP sensor and gpio interface yet. Without deleting the lines above in the `ur.ros2_control.xacro` file, the driver will complain that these required states/command interfaces have not been provided.

### launch file

Now you can launch the driver using the provided launch file.

## Cartesian Motion Control

Clone the [cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers) (ros2) package into your src folder and build it in release mode for faster performance. (you may skip the simulation and tests)

```bash
colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```