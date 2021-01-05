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

import rclpy
import rclpy.utilities as rosutil
from std_msgs.msg import String
from watchdog.events import EVENT_TYPE_MODIFIED
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer
import xacro


class XacroTree:

    def __init__(self, root_file):
        self.root_file = os.path.realpath(root_file)
        self.dirs = set(os.path.dirname(self.root_file))
        self.files = set(self.root_file)
        self.doc = None

    def update(self) -> None:
        self.doc = xacro.process_file(
            self.root_file, **{
                'output': None,
                'just_deps': False,
                'xacro_ns': True,
                'verbosity': 1,
                'mappings': {}
            }
        )
        self.files.update(xacro.all_includes)


class XacroEventHandler(FileSystemEventHandler):

    def __init__(self, xacro_observer, publisher):
        self.xacro_observer = xacro_observer
        self.publisher = publisher

    def on_modified(self, event):
        if event.event_type == EVENT_TYPE_MODIFIED and not event.is_directory:
            print(f'event type: {event.event_type}  path : {event.src_path}')  # TODO: remove

            if self.xacro_observer.is_file_member(event.src_path):
                print(
                    'process!'
                )  # TODO: replace by calling service to update the robot description
                self.xacro_observer.update()
                self.publisher.publish(String(data=self.xacro_observer.xml_string()))
                print(self.xacro_observer.xml_string())


class XacroObserver:

    def __init__(self, root_file, node):
        self.root_file = os.path.realpath(root_file)
        self.dirs = {os.path.dirname(self.root_file)}
        self.files = {self.root_file}
        self.doc = None
        self.observer = Observer()
        self.publisher = node.create_publisher(String, 'topic', 10)
        self.event_handler = XacroEventHandler(self, self.publisher)

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
        except Exception as ex:
            print('Exception2')
            print(ex)


if __name__ == '__main__':

    rclpy.init()
    args = rosutil.remove_ros_args()

    # TODO: improve argparsing (add msgs, consider multiple files and folders)
    assert (len(args) == 2 and os.path.isfile(args[1]))

    filepath = os.path.abspath(args[1])
    filedir = os.path.dirname(filepath)

    node = rclpy.create_node('node')
    observer = XacroObserver(args[1], node)

    observer.start()

    # TODO: consider multiple observers given the xacro include tree scructure
    # observer = Observer()
    # event_handler = XacroEventHandler(filepath)

    # observer.schedule(event_handler, path=filedir, recursive=True)
    # observer._watches
    # observer.start()

    try:
        while True:
            time.sleep(1)
    finally:
        observer.stop()
        observer.join()
