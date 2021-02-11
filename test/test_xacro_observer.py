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

import xacro
from xacro_live import XacroTree


def test_init(xacro_file, xacro_observer):
    # check logger
    assert xacro_observer.logger.name == 'xacro_live'

    # check observer
    assert not xacro_observer.observer.is_alive()

    # check xacro_tree
    expected_xacro_tree = XacroTree(xacro_file)
    assert xacro_observer.xacro_tree.root_file == expected_xacro_tree.root_file
    assert xacro_observer.xacro_tree.dirs == expected_xacro_tree.dirs
    assert xacro_observer.xacro_tree.files == expected_xacro_tree.files


def test_start_stop(xacro_observer, xacro_file, event_handler_mock, canonicalize):
    event_handler_mock.on_modified.assert_not_called()
    xacro_observer.start(event_handler_mock)

    assert xacro_observer.observer.is_alive()
    assert len(xacro_observer.observer.emitters) == 2
    assert canonicalize(xacro_observer.xacro_tree.xml_string()
                        ) == canonicalize(xacro.process(xacro_file))

    xacro_observer.stop()

    assert not xacro_observer.observer.is_alive()
    assert len(xacro_observer.observer.emitters) == 0
