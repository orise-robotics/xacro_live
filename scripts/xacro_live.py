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
from watchdog.events import EVENT_TYPE_MODIFIED
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer
import xacro


class XacroEventHandler(FileSystemEventHandler):

    def __init__(self, root_file):
        super().__init__()
        self.root_file = root_file
        try:
            self.doc, self.all_files = self.__process_file()
        except Exception as ex:  # TODO: specify exceptions to be handled
            print('Invalid xacro file!')
            print(ex)

    def on_modified(self, event):
        if event.event_type == EVENT_TYPE_MODIFIED and not event.is_directory:
            print(f'event type: {event.event_type}  path : {event.src_path}')  # TODO: remove

            if any(True for file in self.all_files if event.src_path.endswith(file)):
                try:
                    self.doc, self.all_files = self.__process_file()
                except Exception as ex:
                    print(ex)

            print('process!')  # TODO: replace by calling service to update the robot description

    def __process_file(self):
        opts = {
            'output': None,
            'just_deps': False,
            'xacro_ns': True,
            'verbosity': 1,
            'mappings': {}
        }
        doc = xacro.process_file(self.root_file, **opts)
        return doc, xacro.all_includes + [self.root_file]


if __name__ == '__main__':

    rclpy.init()
    args = rosutil.remove_ros_args()

    # TODO: improve argparsing (add msgs, consider multiple files and folders)
    assert (len(args) == 2 and os.path.isfile(args[1]))

    filepath = os.path.abspath(args[1])
    filedir = os.path.dirname(filepath)

    # TODO: consider multiple observers given the xacro include tree scructure
    observer = Observer()
    event_handler = XacroEventHandler(filepath)

    observer.schedule(event_handler, path=filedir, recursive=True)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()

    observer.join()

    observer.schedule()
