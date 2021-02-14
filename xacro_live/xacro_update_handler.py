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

import rclpy.logging as roslog
from rclpy.task import Future
from watchdog.events import EVENT_TYPE_MODIFIED
from watchdog.events import FileSystemEventHandler

from .robot_description_client import RobotDescriptionClient
from .xacro_observer import XacroObserver


class XacroUpdateHandler(FileSystemEventHandler):

    def __init__(self, xacro_observer: XacroObserver, client: RobotDescriptionClient):
        self.xacro_observer = xacro_observer
        self.logger = roslog.get_logger('xacro_live')
        self.client = client
        self.future = Future()

    def on_modified(self, event):
        """Look for updates on xacro model and publish the them to the clients."""
        if event.event_type == EVENT_TYPE_MODIFIED and not event.is_directory:
            if self.xacro_observer.xacro_tree.is_file_member(event.src_path):
                self.logger.info("File '{}' modified!".format(event.src_path))
                try:
                    self.xacro_observer.update(self)
                    self.future = self.client.call_async(
                        self.xacro_observer.xacro_tree.xml_string()
                    )
                except Exception as ex:
                    self.future = Future()
                    self.logger.warn('Invalid update!')
                    self.logger.warn(str(ex))
