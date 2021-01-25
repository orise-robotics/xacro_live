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
import typing

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
import rclpy
import rclpy.logging as roslog
import rclpy.node
import rclpy.utilities as rosutil
from watchdog.events import EVENT_TYPE_MODIFIED
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer
import xacro


class RobotDescriptionClient:

    def __init__(
        self,
        client_node: rclpy.node.Node,
        server_node_name='robot_state_publisher',
        param_name='robot_description'
    ):
        self.request = SetParameters.Request()

        parameter = Parameter()
        parameter.name = param_name
        parameter.value.type = ParameterType.PARAMETER_STRING

        self.request.parameters = [parameter]
        self.client = node.create_client(SetParameters, server_node_name + '/set_parameters')

    def wait_for_service(self, timeout_sec=5.) -> None:
        if not self.client.wait_for_service(timeout_sec):
            raise RuntimeError('Wait for service timed out')

    def call_async(self, robot_description_str) -> None:
        self.request.parameters[0].value.string_value = robot_description_str
        self.client.call_async(self.request)


class XacroTree:

    def __init__(self, root_file):
        self.root_file = os.path.realpath(root_file)
        self.dirs = {os.path.dirname(self.root_file)}
        self.files = {self.root_file}
        self.doc = None

    def xml_string(self):
        """Get current urdf string output of the current version of the target file."""
        return self.doc.toprettyxml(indent='  ')

    def is_file_member(self, path):
        """Check if path is a member of the target xacro file tree."""
        return os.path.realpath(path) in self.files

    def update(self):
        """Process the xacro file and update files & directories."""
        self._process_file()
        self._update_file_dir_lists()

    def _process_file(self):
        """Process the xacro file."""
        self.doc = xacro.process_file(
            self.root_file, **{
                'output': None,
                'just_deps': False,
                'xacro_ns': True,
                'verbosity': 1,
                'mappings': {}
            }
        )
        return self.doc

    def _update_file_dir_lists(self):
        """Compute include files and their directories."""
        self.files.update([os.path.realpath(file) for file in xacro.all_includes])
        self.dirs.update([os.path.dirname(file) for file in self.files])


class XacroObserver:

    def __init__(self, root_file):
        self.xacro_tree = XacroTree(root_file)
        self.observer = Observer()
        self.logger = roslog.get_logger('XacroObserver')

    def watched_dirs(self):
        """Get the list of directories being watched."""
        return [emitter.watch.path for emitter in self.observer.emitters]

    def start(self, event_handler):
        """Start tracking."""
        self.observer.start()
        self.update(event_handler)

    def stop(self):
        """Stop tracking."""
        self.observer.stop()
        self.observer.join()

    def update_watchlist(self, event_handler):
        """Update list of directories tracked."""
        watched_dirs = self.watched_dirs()
        new_dirs = [xdir for xdir in self.xacro_tree.dirs if xdir not in watched_dirs]
        for new_dir in new_dirs:
            self.observer.schedule(event_handler, path=new_dir, recursive=False)

    def update(self, event_handler) -> None:
        try:
            self.xacro_tree.update()
            self.update_watchlist(event_handler)
        except Exception as ex:
            self.logger.warn('Invalid update!')
            self.logger.warn(str(ex))


class XacroEventHandler(FileSystemEventHandler):

    def __init__(self, xacro_observer, clients: typing.List[RobotDescriptionClient]):
        self.xacro_observer = xacro_observer
        self.logger = roslog.get_logger('XacroEventHandler')
        self.clients = clients

    def on_modified(self, event):
        if event.event_type == EVENT_TYPE_MODIFIED and not event.is_directory:
            if self.xacro_observer.xacro_tree.is_file_member(event.src_path):
                self.logger.info("File '{}' modified!".format(event.src_path))
                self.xacro_observer.update(self)
                for client in self.clients:
                    client.call_async(self.xacro_observer.xacro_tree.xml_string())


if __name__ == '__main__':

    rclpy.init()
    node = rclpy.create_node('xacro_live')

    args = rosutil.remove_ros_args()

    # TODO: improve argparsing
    assert (len(args) == 2 and os.path.isfile(args[1]))

    observer = XacroObserver(args[1])
    clients = [
        RobotDescriptionClient(node, 'robot_state_publisher'),
        RobotDescriptionClient(node, 'joint_state_publisher')
    ]
    event_handler = XacroEventHandler(observer, clients)
    observer.start(event_handler)

    try:
        rclpy.spin(node)
    finally:
        observer.stop()
