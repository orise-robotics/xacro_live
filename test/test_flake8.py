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

import ament_flake8.main
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    try:
        rc, errors = ament_flake8.main.main_with_errors(argv=[])
        assert rc == 0, \
            'Found %d code style errors / warnings:\n' % len(errors) + \
            '\n'.join(errors)
    except AttributeError:  # main_with_errors is not available
        rc = ament_flake8.main.main(argv=[])
        assert rc == 0, 'Found code style errors / warnings'
