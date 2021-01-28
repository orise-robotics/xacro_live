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

import typing

import rclpy.logging as roslog
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer

from .xacro_tree import XacroTree


class XacroObserver:

    def __init__(self, root_file: str):
        self.xacro_tree = XacroTree(root_file)
        self.observer = Observer()
        self.logger = roslog.get_logger('xacro_live')

    def watched_dirs(self) -> typing.List[str]:
        """Get the list of directories being watched."""
        return [emitter.watch.path for emitter in self.observer.emitters]

    def start(self, event_handler: FileSystemEventHandler) -> None:
        """Start tracking."""
        self.observer.start()
        self.update(event_handler)

    def stop(self) -> None:
        """Stop tracking."""
        self.observer.stop()
        self.observer.join()

    def update_watchlist(self, event_handler: FileSystemEventHandler) -> None:
        """Update list of directories tracked."""
        watched_dirs = self.watched_dirs()
        new_dirs = [xdir for xdir in self.xacro_tree.dirs if xdir not in watched_dirs]
        for new_dir in new_dirs:
            self.observer.schedule(event_handler, path=new_dir, recursive=False)

    def update(self, event_handler: FileSystemEventHandler) -> None:
        """Update xacro_tree and dir watchlist."""
        self.xacro_tree.update()
        self.update_watchlist(event_handler)
