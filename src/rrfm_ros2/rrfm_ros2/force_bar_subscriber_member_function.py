# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench


class ForceBarSubscriber(Node):

    def __init__(self, P:int):
        super().__init__(f'force_bar_P{P}_subscriber')
        self.P = P
        self.subscription = self.create_subscription(
            Wrench, f'/scout/force_bar_P{P}',
            self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"""
    force:
        x: {msg.force.x}
        y: {msg.force.y}
        z: {msg.force.z}
    torque:
        x: {msg.torque.x}
        y: {msg.torque.y}
        z: {msg.torque.z}""")


def main(args=None):
    rclpy.init(args=args)
    subscriber = ForceBarSubscriber(1)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
