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

import rclpy
import rclpy.logging
import rclpy.node
import rclpy.utilities as rosutil

from .xacro_observer import XacroObserver
from .xacro_update_handler import RobotDescriptionClient
from .xacro_update_handler import XacroUpdateHandler


def main():

    rclpy.init()
    node = rclpy.create_node('xacro_live')

    args = rosutil.remove_ros_args()

    # TODO: improve argparsing
    # assert (len(args) == 2 and os.path.isfile(args[1]))

    observer = XacroObserver(args[1])
    client = RobotDescriptionClient(node, 'robot_state_publisher')
    event_handler = XacroUpdateHandler(observer, client)

    try:
        observer.start(event_handler)
    except Exception as ex:  # noqa [flake8(B902)] TODO: specify exception types
        rclpy.logging.get_logger('xacro_live').error('Invalid startup robot_description!')
        rclpy.logging.get_logger('xacro_live').error(str(ex))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('xacro_live').info('Exited by KeyboardInterrupt')
    finally:
        rclpy.logging.get_logger('xacro_live').info('Stop xacro observer!')
        observer.stop()
