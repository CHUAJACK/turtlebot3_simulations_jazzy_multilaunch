# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# gz element tags whose text is a topic name -> made absolute and namespaced
# e.g. <topic>scan</topic> becomes <topic>/tb3_1/scan</topic>
TOPIC_TAGS = {'topic', 'odom_topic', 'tf_topic', 'camera_info_topic'}
# gz element tags whose text is a TF frame -> prefixed to match frame_prefix
# e.g. <frame_id>odom</frame_id> becomes <frame_id>tb3_1/odom</frame_id>
FRAME_TAGS = {'frame_id', 'child_frame_id', 'gz_frame_id'}


def _tmp_dir():
    path = os.path.join(tempfile.gettempdir(), 'tb3_multirobot')
    os.makedirs(path, exist_ok=True)
    return path


def _namespaced_sdf(sdf_path, namespace):
    """Rewrite the model SDF so every gz topic/frame is unique to ``namespace``.

    The stock turtlebot3 SDF hard-codes global topics (``cmd_vel``, ``scan``,
    ``/tf`` ...) and global frames (``odom``, ``base_footprint`` ...). Spawning
    more than one of those would make every robot publish/subscribe on the same
    gz topics, so we prefix them with the namespace and set the model name to
    the namespace (which is also the gz entity name passed to ``create``).
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    model = root.find('model')
    if model is not None:
        model.set('name', namespace)

    for elem in root.iter():
        text = (elem.text or '').strip()
        if not text:
            continue
        if elem.tag in TOPIC_TAGS:
            elem.text = '/' + namespace + '/' + text.lstrip('/')
        elif elem.tag in FRAME_TAGS:
            elem.text = namespace + '/' + text.lstrip('/')

    out_path = os.path.join(_tmp_dir(), namespace + '_model.sdf')
    with open(out_path, 'w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(ET.tostring(root, encoding='unicode'))
    return out_path


def _namespaced_bridge_config(bridge_path, namespace):
    """Prefix every ros/gz topic in the bridge config with ``namespace``.

    ``clock`` is skipped: it is a single global topic bridged once by the top
    level launch file, not per robot.
    """
    with open(bridge_path) as f:
        entries = yaml.safe_load(f)

    out = []
    for entry in entries:
        if 'clock' in (entry.get('ros_topic_name'), entry.get('gz_topic_name')):
            continue
        entry = dict(entry)
        entry['ros_topic_name'] = '/' + namespace + '/' + entry['ros_topic_name'].lstrip('/')
        entry['gz_topic_name'] = '/' + namespace + '/' + entry['gz_topic_name'].lstrip('/')
        out.append(entry)

    out_path = os.path.join(_tmp_dir(), namespace + '_bridge.yaml')
    with open(out_path, 'w') as f:
        yaml.safe_dump(out, f)
    return out_path


def launch_setup(context, *args, **kwargs):
    turtlebot3_model = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + turtlebot3_model
    pkg_share = get_package_share_directory('turtlebot3_gazebo')

    sdf_path = os.path.join(pkg_share, 'models', model_folder, 'model.sdf')
    bridge_path = os.path.join(pkg_share, 'params', model_folder + '_bridge.yaml')

    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)

    # Fall back to the plain single-robot behaviour when no namespace is given.
    if namespace:
        entity_name = namespace
        spawn_sdf = _namespaced_sdf(sdf_path, namespace)
        bridge_config = _namespaced_bridge_config(bridge_path, namespace)
        camera_topic = '/' + namespace + '/camera/image_raw'
        suffix = '_' + namespace
    else:
        entity_name = turtlebot3_model
        spawn_sdf = sdf_path
        bridge_config = bridge_path
        camera_topic = '/camera/image_raw'
        suffix = ''

    actions = []

    actions.append(Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_turtlebot3' + suffix,
        arguments=[
            '-name', entity_name,
            '-file', spawn_sdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
        output='screen',
    ))

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge' + suffix,
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}',
        ],
        output='screen',
    ))

    # The burger has no camera; only bridge the image when the model provides one.
    if turtlebot3_model != 'burger':
        actions.append(Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='image_bridge' + suffix,
            arguments=[camera_topic],
            output='screen',
        ))

    return actions


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace applied to the robot, its gz topics and its TF frames'))
    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Spawn x position of the robot'))
    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Spawn y position of the robot'))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
