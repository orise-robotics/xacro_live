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

import xacro


class XacroTree:

    def __init__(self, root_file: str):
        self.root_file = root_file

    @property
    def dirs(self):
        return self._dirs.copy()

    @property
    def files(self):
        return self._files.copy()

    @property
    def root_file(self):
        return self._root_file

    @root_file.setter
    def root_file(self, filename):
        self._root_file = os.path.realpath(filename)
        self._dirs = {os.path.dirname(self.root_file)}
        self._files = {self.root_file}
        self._doc = None

    def xml_string(self) -> str:
        """Get current urdf string output of the current version of the target file."""
        return self._doc.toprettyxml(indent='  ')

    def is_file_member(self, path: str) -> bool:
        """Check if path is a member of the target xacro file tree."""
        return os.path.realpath(path) in self._files

    def update(self) -> None:
        """Process the xacro file and update files & directories."""
        xacro.all_includes = []
        self._doc = xacro.process_file(
            self.root_file, **{
                'output': None,
                'just_deps': False,
                'xacro_ns': True,
                'verbosity': 1,
                'mappings': {}
            }
        )
        self._files = {os.path.realpath(file) for file in xacro.all_includes + [self.root_file]}
        self._dirs = {os.path.dirname(file) for file in self._files}
