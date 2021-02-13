# Copyright 2021 Open Rise Robotics
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
import shutil
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.node import Node
from watchdog.events import FileSystemEventHandler
from xacro_live import XacroObserver


class RobotDescriptionServer(Node):

    def __init__(self):
        super().__init__('robot_state_publisher')
        self.declare_parameter('robot_description', str())


@pytest.fixture
def xacro_dir(tmp_path):
    shutil.copytree('test/urdf', tmp_path, dirs_exist_ok=True)
    return tmp_path


@pytest.fixture
def xacro_file(xacro_dir):
    return os.path.join(xacro_dir, 'robot.xacro')


@pytest.fixture
def xacro_observer(xacro_file):
    return XacroObserver(xacro_file)


@pytest.fixture
def event_handler_mock():
    event_handler = FileSystemEventHandler()
    event_handler.on_modified = MagicMock()
    return event_handler


@pytest.fixture(scope='module')
def canonicalize_xml():
    try:
        import xml.etree.ElementTree
        canonicalize_fn = xml.etree.ElementTree.canonicalize
    except AttributeError:
        import io

        import lxml.etree

        def canonicalize_fn(xml_string: str):
            et = lxml.etree.parse(io.StringIO(xml_string))
            return lxml.etree.tostring(et, method='c14n', with_comments=False)

    return canonicalize_fn


@pytest.fixture
def test_node():

    def test_node_fn(node_name: str):
        rclpy.init()
        return rclpy.create_node(node_name)

    yield test_node_fn
    rclpy.shutdown()


@pytest.fixture
def robot_description_server(test_node):
    return RobotDescriptionServer(test_node('robot_state_publisher'))
