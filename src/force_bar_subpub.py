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


class ForceBarSubPub(Node):
    """merge force bar segment topcis into a single topic"""

    def __init__(self, sn:int, dt:float, *, ft:str='f', xyz:str='x'):
        """
        sn.... segment number (mendatory)
        dt.... timer interval (mendatory)
        ft.... force 'f' or torque 't' sel. for merged pub
        xyz... 'x', 'y' or 'z' selection for merged pub
        """
        super().__init__(f'force_bar_segment_{sn}_subpub')
        self.sn = sn
        self.dt = dt
        self.ft = ft
        self.xyz = xyz
        self.val = 0.0
        self.subscription = self.create_subscription(
            Wrench,
            f'/scout/force_bar/segment_{self.sn}',
            self.listener_callback,
            10)
        self.pub_topic = f"/scout/force_bar/joined_{'torque' if ft == 't' else 'force'}_{xyz}"
        self.publisher = self.create_publisher(
            String,
            self.pub_topic,
            10)
        self.timer = self.create_timer(
            self.dt,
            self.timer_callback)

    def listener_callback(self, msg):
        if self.ft == 'f':
            if self.xyz == 'x':
                self.val = msg.force.x
            if self.xyz == 'y':
                self.val = msg.force.y
            if self.xyz == 'z':
                self.val = msg.force.z
        if self.ft == 't':
            if self.xyz == 'x':
                self.val = msg.torque.x
            if self.xyz == 'y':
                self.val = msg.torque.y
            if self.xyz == 'z':
                self.val = msg.torque.z

    def timer_callback(self):
        msg = String()
        msg.data = f"{self.sn} {self.val}"
        self.publisher.publish(msg)
#-LOG-BLOCK-START---------------------------------------------------------------#
        self.get_logger().info(f"""
    publishing:
        topic: {self.pub_topic}
        sensor_n: {self.sn:4d}
        {'torque' if self.ft == 't' else 'force'}_{self.xyz}: {self.val:8.2f}""")
#-LOG-BLOCK-END-----------------------------------------------------------------#


def main(args=None):
    rclpy.init(args=args)

    sn = int(argv[1])
    subscriber = ForceBarSubPub(sn, dt=0.01)

    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
