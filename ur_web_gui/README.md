# UR_WEB_GUI

This package depends on [nicegui](https://nicegui.io/).

```bash
pip3 install nicegui
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
