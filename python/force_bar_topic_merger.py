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

from sys import argv

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
from std_msgs.msg import String


class ForceBarTopicMerger(Node):
    """merge force bar segment topcis into a single topic"""

    def __init__(self, sn:int, dt:float):
        """
        sn.... segment number
        dt.... timer interval
        """
        super().__init__(f'force_bar_topic_merger_{sn}')
        self.sn = sn
        self.dt = dt
        self.val = 0.0
        self.subscription = self.create_subscription(
            msg_type=Wrench,
            topic=f'/scout/force_bar/segment_{self.sn}',
            callback=self.listener_callback,
            qos_profile=10)
        self.publisher = self.create_publisher(
            msg_type=String,
            topic="/merged_force_topic",
            qos_profile=10)
        self.timer = self.create_timer(
            timer_period_sec=self.dt,
            callback=self.timer_callback)

    def listener_callback(self, msg):
        self.val = msg.force.x

    def timer_callback(self):
        msg = String()
        msg.data = f"{self.sn} {self.val}"
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    subscriber = ForceBarTopicMerger(
        sn=int(argv[1]),
        dt=0.01)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
