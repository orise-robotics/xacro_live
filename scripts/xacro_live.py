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
import time

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
import rclpy
import rclpy.logging as roslog
import rclpy.utilities as rosutil
from watchdog.events import EVENT_TYPE_MODIFIED
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer
import xacro


class XacroEventHandler(FileSystemEventHandler):

    def __init__(self, xacro_observer):
        self.xacro_observer = xacro_observer
        self.logger = roslog.get_logger('XacroEventHandler')

    def on_modified(self, event):
        if event.event_type == EVENT_TYPE_MODIFIED and not event.is_directory:
            if self.xacro_observer.is_file_member(event.src_path):
                self.logger.info("File '{}' modified!".format(event.src_path))
                self.xacro_observer.update()


class XacroObserver:

    def __init__(self, root_file, node):
        self.root_file = os.path.realpath(root_file)
        self.dirs = {os.path.dirname(self.root_file)}
        self.files = {self.root_file}
        self.doc = None
        self.observer = Observer()
        self.client = node.create_client(SetParameters, 'robot_state_publisher/set_parameters')

        self.logger = roslog.get_logger('XacroObserver')

        ready = self.client.wait_for_service(timeout_sec=5.0)

        if not ready:
            raise RuntimeError('Wait for service timed out')

        self.request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = 'robot_description'
        parameter.value.type = ParameterType.PARAMETER_STRING
        self.request.parameters = [parameter]

        self.event_handler = XacroEventHandler(self)

    def xml_string(self):
        return self.doc.toprettyxml(indent='  ')

    def watched_dirs(self):
        return [emitter.watch.path for emitter in self.observer.emitters]

    def start(self):
        self.observer.start()
        self.update()

    def stop(self):
        self.observer.stop()
        self.observer.join()

    def is_file_member(self, path):
        return os.path.realpath(path) in self.files

    def update(self) -> None:
        try:
            # process xacro file
            self.doc = xacro.process_file(
                self.root_file, **{
                    'output': None,
                    'just_deps': False,
                    'xacro_ns': True,
                    'verbosity': 1,
                    'mappings': {}
                }
            )
            # compute include files and dirs
            self.files.update([os.path.realpath(file) for file in xacro.all_includes])
            self.dirs.update([os.path.dirname(file) for file in self.files])

            # watch all include files
            watched_dirs = self.watched_dirs()
            new_dirs = [xdir for xdir in self.dirs if xdir not in watched_dirs]
            for new_dir in new_dirs:
                self.observer.schedule(self.event_handler, path=new_dir, recursive=False)

            self.request.parameters[0].value.string_value = self.xml_string()
            self.client.call_async(self.request)
        except Exception as ex:
            self.logger.warn('Invalid update!')
            self.logger.warn(str(ex))


if __name__ == '__main__':

    rclpy.init()
    node = rclpy.create_node('xacro_live')

    args = rosutil.remove_ros_args()

    # TODO: improve argparsing
    assert (len(args) == 2 and os.path.isfile(args[1]))

    observer = XacroObserver(args[1], node)
    observer.start()

    try:
        while True:
            time.sleep(1)
    finally:
        observer.stop()
        observer.join()
