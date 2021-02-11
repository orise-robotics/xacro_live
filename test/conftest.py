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

from unittest.mock import MagicMock

import pytest
from watchdog.events import FileSystemEventHandler
from xacro_live import XacroObserver


@pytest.fixture
def xacro_file():
    return 'test/urdf/robot.xacro'


@pytest.fixture
def xacro_observer(xacro_file):
    return XacroObserver(xacro_file)


@pytest.fixture
def event_handler_mock():
    event_handler = FileSystemEventHandler()
    event_handler.on_modified = MagicMock()
    return event_handler


@pytest.fixture
def canonicalize():
    try:
        import xml.etree.ElementTree
        canonicalize_fn = xml.etree.ElementTree.canonicalize
    except AttributeError:
        import io

        import lxml.etree

        def canonicalize_fn(xml_string: str):
            et = lxml.etree.parse(io.StringIO(xml_string))
            return lxml.etree.tostring(et, method='c14n', with_comments=False)

    return canonicalize_fn
