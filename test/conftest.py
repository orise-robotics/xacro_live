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
import time
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


class AsyncTimeout:

    def __init__(self, duration: float, auto_start=True):
        self.duration = duration
        if auto_start:
            self.start()

    def start(self):
        self.start_time = time.clock_gettime(time.CLOCK_MONOTONIC)

    def running(self) -> bool:
        return not self.finished()

    def finished(self) -> bool:
        return (time.clock_gettime(time.CLOCK_MONOTONIC) - self.start_time) >= self.duration


@pytest.fixture(scope='session')
def async_timeout():

    def timeout_fn(duration, auto_start=True):
        return AsyncTimeout(duration, auto_start)

    return timeout_fn


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


@pytest.fixture(scope='session')
def canonicalize_xml():
    import xml.etree.ElementTree
    return xml.etree.ElementTree.canonicalize


@pytest.fixture
def init_node():
    yield rclpy.init()
    rclpy.shutdown()


@pytest.fixture
def robot_description_server(init_node):
    return RobotDescriptionServer()
