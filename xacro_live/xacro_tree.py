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
        self.root_file = os.path.realpath(root_file)
        self.dirs = {os.path.dirname(self.root_file)}
        self.files = {self.root_file}
        self.doc = None

    def xml_string(self) -> str:
        """Get current urdf string output of the current version of the target file."""
        return self.doc.toprettyxml(indent='  ')

    def is_file_member(self, path: str) -> bool:
        """Check if path is a member of the target xacro file tree."""
        return os.path.realpath(path) in self.files

    def update(self) -> None:
        """Process the xacro file and update files & directories."""
        self.doc = xacro.process_file(
            self.root_file, **{
                'output': None,
                'just_deps': False,
                'xacro_ns': True,
                'verbosity': 1,
                'mappings': {}
            }
        )
        self.files.update([os.path.realpath(file) for file in xacro.all_includes])
        self.dirs.update([os.path.dirname(file) for file in self.files])
