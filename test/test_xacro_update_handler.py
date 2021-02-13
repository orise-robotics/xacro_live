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

import pathlib

import pytest
import rclpy
from xacro_live import RobotDescriptionClient
from xacro_live import XacroUpdateHandler


@pytest.fixture
def robot_description_client(init_node):
    return RobotDescriptionClient(rclpy.create_node('robot_description_client'))


@pytest.fixture
def handler(xacro_observer, robot_description_client):
    handler_obj = XacroUpdateHandler(xacro_observer, robot_description_client)
    xacro_observer.start(handler_obj)
    return handler_obj


@pytest.fixture
def xacro_path(xacro_file):
    return pathlib.Path(xacro_file)


def test_init(handler, xacro_observer, robot_description_client):
    assert handler.xacro_observer == xacro_observer
    assert handler.client == robot_description_client
    assert handler.logger.name == 'xacro_live'


def test_on_modify_valid(handler, xacro_path, canonicalize_xml, robot_description_server):
    xacro_path.touch()

    while robot_description_server.get_parameter('robot_description').value == '':
        rclpy.spin_once(robot_description_server)

    expected = canonicalize_xml(handler.xacro_observer.xacro_tree.xml_string())
    actual = canonicalize_xml(robot_description_server.get_parameter('robot_description').value)

    assert expected == actual


def test_on_modify_invalid(handler, xacro_path, robot_description_server):
    xacro_path.write_text('/')

    rclpy.spin_once(robot_description_server, timeout_sec=1)

    assert robot_description_server.get_parameter('robot_description').value == ''
