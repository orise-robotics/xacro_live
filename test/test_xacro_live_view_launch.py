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

from pathlib import Path
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
import rclpy.node


@pytest.mark.launch_test
def generate_test_description():
    share_dir = FindPackageShare('xacro_live')

    spawn_launch_path = PathJoinSubstitution([share_dir, 'launch/xacro_live_view.launch.py'])
    xacro_file_path = str(Path(__file__).parent / 'urdf/robot.xacro')

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_path),
        launch_arguments={'xacro_file': xacro_file_path}.items(),
    )

    return (
        launch.LaunchDescription([
            spawn_launch,
            TimerAction(period=5., actions=[ReadyToTest()]),
        ]),
        {
            'spawn_launch': spawn_launch
        }
    )


class TestSpawnLaunchInterface(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.node.Node('test_node')

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def test_node_names(self, proc_info, spawn_launch):
        self.assertEqual(1, len(spawn_launch.get_sub_entities()))
        nodes = set(
            filter(
                lambda entity: isinstance(entity, rclpy.node.Node),
                spawn_launch.get_sub_entities()[0].entities
            )
        )

        expected_nodes = [  # in terms of regex expresions
            r'/robot_state_publisher',
            r'/robot_state_publisher',
            r'/joint_state_publisher_gui',
            r'/xacro_live',
            r'/rviz',
            r'/transform_listener_impl_\w{12}'
        ]

        # Make a regex that matches if any of our regexes match.
        combined_regex = '(' + ')|('.join(expected_nodes) + ')'

        for node in nodes:
            proc_info.assertWaitForStartup(node, timeout=5)
            self.assertTrue(node.is_node_name_fully_specified())
            self.assertRegex(node.node_name, combined_regex)

    def test_topics(self):
        topics = self.node.get_topic_names_and_types()
        self.assertIn(('/joint_states', ['sensor_msgs/msg/JointState']), topics)
        self.assertIn(('/robot_description', ['std_msgs/msg/String']), topics)
        self.assertIn(('/tf', ['tf2_msgs/msg/TFMessage']), topics)
        self.assertIn(('/tf_static', ['tf2_msgs/msg/TFMessage']), topics)

    def test_robot_description(self):
        client = self.node.create_client(GetParameters, 'robot_state_publisher/get_parameters')
        assert client.wait_for_service(5.)
        request = GetParameters.Request(names=['robot_description'])
        recv_robot_description = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, recv_robot_description, timeout_sec=5.)
        assert recv_robot_description.done()


@launch_testing.post_shutdown_test()
class TestProcessTermination(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # rclpy does not exit normally on SIGINT signal (https://github.com/ros2/rclpy/issues/527)
        launch_testing.asserts.assertExitCodes(proc_info, [launch_testing.asserts.EXIT_OK, -2])
