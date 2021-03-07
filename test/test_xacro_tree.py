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
from xacro_live import XacroTree


def test_init(xacro_file):
    tree = XacroTree(xacro_file)

    # check root_file attribute
    assert tree.root_file.endswith(xacro_file)
    assert os.path.isabs(tree.root_file)
    assert os.path.isfile(tree.root_file)
    assert os.path.exists(tree.root_file)
    assert len(tree.files) == 1
    assert len(tree.dirs) == 1

    # check files and dirs attributes
    assert tree.root_file in tree.files
    assert os.path.dirname(tree.root_file) in tree.dirs


def test_update(xacro_file, canonicalize_xml):
    tree = XacroTree(xacro_file)
    tree.update()

    assert canonicalize_xml(tree.xml_string()) == canonicalize_xml(xacro.process(xacro_file))

    # check update of 'files' and 'dirs' attributes
    assert len(tree.files) == 2
    assert len(tree.dirs) == 2
    assert tree.root_file in tree.files
    assert os.path.dirname(tree.root_file) in tree.dirs
    assert any(xfile.endswith('snippets/wheel.xacro') for xfile in tree.files)
    assert any(xdir.endswith('snippets') for xdir in tree.dirs)
