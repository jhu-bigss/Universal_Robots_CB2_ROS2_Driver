import argparse
import sys
import xml.dom.minidom

import math
import threading
from pathlib import Path
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser

import rclpy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from ament_index_python.packages import get_package_share_directory

from nicegui import Client, app, ui, ui_run


def _convert_to_float(name, jtype, limit_name, input_string):
    try:
        return float(input_string)
    except ValueError as e:
        raise Exception(
            f'"{limit_name}" limit must be a float for joint "{name}" of type "{jtype}"') from e

def _convert_to_deg(value):
    return value * 180.0 / math.pi

def _convert_to_rad(value):
    return value * math.pi / 180.0

class WebGuiNode(Node):

    def init_urdf(self, xmldom):
        free_joints = {}
        joint_list = []
        links = {}
        visual_link_list = ['base_link_inertia',
                     'shoulder_link',
                     'upper_arm_link',
                     'forearm_link',
                     'wrist_1_link',
                     'wrist_2_link',
                     'wrist_3_link'] # temporary hard-coded list

        robot_list = xmldom.getElementsByTagName('robot')
        if not robot_list:
            raise Exception('URDF must have a "robot" tag')
        robot = robot_list[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'link':
                if child.getAttribute('name') not in visual_link_list:
                    continue
                links[child.getAttribute('name')] = {'xyz': [float(m) for m in child.getElementsByTagName('visual').item(0).getElementsByTagName('origin').item(0).getAttribute('xyz').split(" ")],
                                                     'rpy': [float(n) for n in child.getElementsByTagName('visual').item(0).getElementsByTagName('origin').item(0).getAttribute('rpy').split(" ")]}
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype in ('fixed', 'floating', 'planar'):
                    continue
                name = child.getAttribute('name')
                parent_link = child.getElementsByTagName('parent').item(0).getAttribute('link')
                child_link = child.getElementsByTagName('child').item(0).getAttribute('link')
                if jtype == 'continuous':
                    minval = -math.pi * 2
                    maxval = math.pi * 2
                else:
                    # Limits are required, and required to be floats.
                    limit_list = child.getElementsByTagName('limit')
                    if not limit_list:
                        raise Exception(
                            f'Limits must be specified for joint "{name}" of type "{jtype}"')

                    limit = limit_list[0]

                    if not limit.hasAttribute('lower'):
                        raise Exception(
                            f'"lower" limit must be specified for joint "{name}" of type "{jtype}"')
                    minval = _convert_to_float(name, jtype, 'lower', limit.getAttribute('lower'))

                    if not limit.hasAttribute('upper'):
                        raise Exception(
                            f'"upper" limit must be specified for joint "{name}" of type "{jtype}"')
                    maxval = _convert_to_float(name, jtype, 'upper', limit.getAttribute('upper'))

                joint_list.append(name)

                joint = {'min': _convert_to_deg(minval),
                         'max': _convert_to_deg(maxval),
                         'current_position_in_deg': 0.0,
                         'desired_position_in_deg': 0.0,
                         'parent_link': parent_link,
                         'child_link': child_link,
                         }

                if jtype == 'continuous':
                    joint['continuous'] = True
                free_joints[name] = joint

        return (free_joints, joint_list, links, visual_link_list)

    def configure_robot(self, description):
        self.get_logger().debug('Got description, configuring robot')
        xmldom = xml.dom.minidom.parseString(description)
        (self.free_joints, self.joint_list, self.links, self.link_list) = self.init_urdf(xmldom)

        # configure the robot kinematics using PyKDL
        flag, kdl_tree = kdl_parser.treeFromString(description)
        self.kdl_chain = kdl_tree.getChain(self.link_list[0], self.link_list[-1])
        self.kdl_joints = kdl.JntArray(self.kdl_chain.getNrOfJoints())
        self.kdl_fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)

        # configure the UI
        self.configure_ui()

    def declare_ros_parameter(self, name, default, descriptor):
        # When the automatically_declare_parameters_from_overrides parameter to
        # rclpy.create_node() is True, then any parameters passed in on the
        # command-line are automatically declared.  In that case, calling
        # node.declare_parameter() will raise an exception.  However, in the case
        # where a parameter is *not* overridden, we still want to declare it
        # (so it shows up in "ros2 param list", for instance).  Thus we always do
        # a declaration and just ignore ParameterAlreadyDeclaredException.

        try:
            self.declare_parameter(name, default, descriptor)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

    def robot_description_cb(self, msg):
        try:
            self.configure_robot(msg.data)
        except Exception as e:
            self.get_logger().warn(str(e))

    def __init__(self, description_file ):
        super().__init__('ur_web_gui', automatically_declare_parameters_from_overrides=True)

        self.declare_ros_parameter('ur_type', 'ur10e', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='robot type'))
        self.declare_ros_parameter('robot_description_pkg_name', 'ur_description', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='robot description package name'))
        package_share_directory = get_package_share_directory(self.get_parameter('robot_description_pkg_name').value)
        app.add_static_files('/static', package_share_directory + '/meshes/' + self.get_parameter('ur_type').value + '/visual')

        self.free_joints = {}
        self.joint_list = []
        self.links = {}
        self.link_list = []
        self.kdl_chain = None
        self.kdl_joints = kdl.JntArray(6) # 6 joints by default
        self.T = [kdl.Frame() for i in range(7)] # 7 links by default including the base_link

        if description_file is not None:
            # If we were given a URDF file on the command-line, use that.
            with open(description_file, 'r') as infp:
                description = infp.read()
            self.configure_robot(description)
        else:
            self.get_logger().info(
                'Waiting for robot_description to be published on the robot_description topic...')

        qos = rclpy.qos.QoSProfile(depth=1,
                                   durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_robot_description = self.create_subscription(String,
                                 'robot_description',
                                 lambda msg: self.robot_description_cb(msg),
                                 qos)
        self.sub_jnt_states = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, qos)
        self.pub_cmd_fwd_pos_controller = self.create_publisher(Float64MultiArray, 'forward_position_controller/commands', qos)

    def configure_ui(self):
        # Web GUI
        with Client.auto_index_client:
            # Joint Control
            with ui.row().classes('items-stretch'):
                with ui.card().classes('w-300 text-center items-center'):
                    ui.label('Joint Control').classes('text-2xl')
                    self.control_on_switch = ui.switch('Remote Control', on_change=lambda e: self.control_on_switch_cb(e))
                    self.realtime_jog_switch = ui.switch('Real-time')
                    ui.label('Real-time jogging is ON! Please be careful!').classes('text-xs text-red-500').bind_visibility_from(self.realtime_jog_switch, 'value')
                    for j in self.joint_list:
                        joint = self.free_joints[j]
                        ui.label(j).classes('text-l text-bold')
                        joint["ui_label"] = ui.label().classes('mb-[-1.4em]').bind_text_from(joint, 'current_position_in_deg', lambda x: "%.2f" % x + "°")
                        joint["ui_slider"] = ui.slider(min=joint['min'], max=joint['max'], step=0.1, on_change=self.set_desired_joint_positions) \
                            .props('label').on('update:model-value', throttle=1.0, leading_events=False) \
                            .bind_value(joint, 'desired_position_in_deg').bind_enabled_from(self.control_on_switch, 'value')

                    # Buttons
                    ui.button('Reset', on_click=self.reset_desired_joint_positions).bind_enabled_from(self.control_on_switch, 'value')
                    ui.button('Send Commands', on_click=self.publish_fwd_pos_controller).bind_enabled_from(self.control_on_switch, 'value')

                # Cartesian Control
                # TODO: Implement Cartesian Control

                # Visualization
                with ui.card().classes('w-300 h-300 items-center'):
                    ui.label('Visualization').classes('text-2xl')
                    with ui.scene(500, 500) as scene:
                        self.links['base_link_inertia']['stl'] = scene.stl('/static/base.stl')
                        self.links['shoulder_link']['stl'] = scene.stl('/static/shoulder.stl')
                        self.links['upper_arm_link']['stl'] = scene.stl('/static/upperarm.stl')
                        self.links['forearm_link']['stl'] = scene.stl('/static/forearm.stl')
                        self.links['wrist_1_link']['stl'] = scene.stl('/static/wrist1.stl')
                        self.links['wrist_2_link']['stl'] = scene.stl('/static/wrist2.stl')
                        self.links['wrist_3_link']['stl'] = scene.stl('/static/wrist3.stl')

    def joint_states_callback(self, msg):
        if len(self.joint_list) == len(msg.name):
            for i, name in enumerate(msg.name):
                self.free_joints[name]['current_position_in_deg'] = _convert_to_deg(msg.position[i])

        if self.kdl_chain is not None:
            self.update_visualization()

    def control_on_switch_cb(self, e):
        if e.value:
            self.reset_desired_joint_positions()

    def reset_desired_joint_positions(self):
        for joint in self.joint_list:
            self.free_joints[joint]['desired_position_in_deg'] = self.free_joints[joint]['current_position_in_deg']

    def set_desired_joint_positions(self):
        for i, joint in enumerate(self.joint_list):
            self.kdl_joints[i] = _convert_to_rad(self.free_joints[joint]['desired_position_in_deg'])

        # if real-time jogging is ON, publish the joint positions right away
        if self.realtime_jog_switch.value:
            self.publish_fwd_pos_controller()

    def publish_fwd_pos_controller(self):
        msg = Float64MultiArray()
        for joint in self.joint_list:
            msg.data.append(_convert_to_rad(self.free_joints[joint]['desired_position_in_deg']))

        self.pub_cmd_fwd_pos_controller.publish(msg)

    def update_visualization(self):
        for i, link in enumerate(self.link_list):
            self.kdl_fk_solver.JntToCart(self.kdl_joints, self.T[i], i)
            T = kdl.Frame(kdl.Rotation.RPY(*self.links[link]['rpy']), kdl.Vector(*self.links[link]['xyz']))
            T = self.T[i] * T # where magic happens
            self.links[link]['stl'].rotate(*T.M.GetRPY())
            self.links[link]['stl'].move(*T.p)

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'description_file', help='Robot description file to use', nargs='?', default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])
    web_gui_node = WebGuiNode(parsed_args.description_file)

    try:
        rclpy.spin(web_gui_node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')