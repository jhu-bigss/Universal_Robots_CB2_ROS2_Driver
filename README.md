# Universal_Robots_CB2_ROS2_Driver

## To use this driver with the official [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), which you can install with the following command:

```bash
sudo apt-get install ros-<distro>-ur-description
```
You need to modify the URDF file within the package:

```
colcon_cd ur_description

urdf
├── ur_macro.xacro
├── ur.ros2_control.xacro
└── ur.urdf.xacro
```

### `ur.urdf.xacro` and `ur_macro.xacro`

add `cb2_controller` as a xacro input parameter argument:

```xml
    <xacro:arg name="cb2_controller" default="false"/>

    <xacro:ur_robot
      name="$(arg name)"
      tf_prefix="$(arg tf_prefix)"
      ...
      cb2_controller="$(arg cb2_controller)"
      ...
```

```xml
  <xacro:macro name="ur_robot" params="
    name
    tf_prefix
    ...
    cb2_controller:=false
    ...
    <xacro:if value="${generate_ros2_control_tag}">
      <xacro:include filename="$(find ur_description)/urdf/ur.ros2_control.xacro" />
      <xacro:ur_ros2_control
        name="${name}"
        use_fake_hardware="${use_fake_hardware}"
        ...
        cb2_controller="${cb2_controller}"
        ...
```


### `ur.ros2_control.xacro`

change the plugin name from `ur_robot_driver` to `ur_robot_driver_cb2` as the following line:

```xml
    <plugin>ur_robot_driver_cb2/URPositionHardwareInterface</plugin>
```

add the `cb2_controller` argument to `sensor` and `gpio` interface xacro:

```xml
 <xacro:unless value="${sim_gazebo or sim_ignition or cb2_controller}">
 ...
```
so that you can use `cb2_controller` argument in your launch file to disable TCP sensor and gpio interface. Otherwise, the driver will complain that these required states/command interfaces have not been provided.

### launch file

Set `cb2_controller:=true` when generating the robot description:

```python
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=",
            "ur5",
            " ",
            "ur_type:=",
            "ur5",
            " ",
            "prefix:=",
            "",
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "cb2_controller:=",
            "true",
        ]
    )
```
## Cartesian Motion Control

Clone the [cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers) (ros2) package into your src folder and build it in release mode for faster performance. (skip the simulation and tests)

```bash
colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```