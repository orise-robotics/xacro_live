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

import rcl_interfaces.msg
import rcl_interfaces.srv
import rclpy
import rclpy.node


class RobotDescriptionClient:

    def __init__(
        self,
        client_node: rclpy.node.Node,
        server_node_name='robot_state_publisher',
        param_name='robot_description'
    ):
        self.request = rcl_interfaces.srv.SetParameters.Request()

        parameter = rcl_interfaces.msg.Parameter()
        parameter.name = param_name
        parameter.value.type = rcl_interfaces.msg.ParameterType.PARAMETER_STRING

        self.request.parameters = [parameter]
        self.client = client_node.create_client(
            rcl_interfaces.srv.SetParameters, server_node_name + '/set_parameters'
        )

    def wait_for_service(self, timeout_sec=5.) -> None:
        if not self.client.wait_for_service(timeout_sec):
            raise RuntimeError('Wait for service timed out')

    def call_async(self, robot_description_str: str) -> None:
        self.request.parameters[0].value.string_value = robot_description_str
        self.client.call_async(self.request)
