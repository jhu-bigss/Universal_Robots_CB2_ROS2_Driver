# UR_WEB_GUI

This package depends on [nicegui](https://nicegui.io/) and [kdl_parser_py](https://wiki.ros.org/kdl_parser_py).
`kdl_parser_py` depends on [urdf_parser_py](https://github.com/ros/urdf_parser_py/tree/ros2).

```bash
pip3 install nicegui urdf_parser_py
```

## Requirements

* clone [kdl_parser_py](https://github.com/ros/kdl_parser.git)
    * The **kdl_parser_py** package is not yet available in ROS2 at this moment. There is some efforts proting it to ROS2, but it is not yet merged. So, we are using this [pull request #55](https://github.com/ros/kdl_parser/pull/55).

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/ros/kdl_parser.git
git fetch origin pull/55/head:pr55
git checkout pr55
```
## Convert `dae` meshes to `glb` using free online convertor

`nicegui` 3D viewer only supports ``stl` or `glb` files. You can convert `dae` files to `glb` using [this online convertor](https://imagetostl.com/convert/file/dae/to/glb).

Locate your ur_descriptiopn package and convert the files under `/meshes` to `glb` format. Otherwise, you will get an error like this:

```bash
[WARN] [ur_web_gui]: File not found: /opt/ros/<>your-ros-distro>/share/ur_description/meshes/urXX/visual/mesh_name.glb
```