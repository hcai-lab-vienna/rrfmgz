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

    def __init__(self, sn:int, *, ft:str='f', xyz:str='x',
                 echo:bool=False, dt:float=1.0, p:int=5):
        """
        sn.... segment number (mendatory)
        ft.... force 'f' or torque 't' sel. for merged pub
        xyz... 'x', 'y' or 'z' selection for merged pub
        echo.. print logging to stdout
        dt.... timer interval
        p..... precision
        """
        super().__init__(f'force_bar_segment_{sn}_subpub')
        self.sn = sn
        self.ft = ft
        self.xyz = xyz
        self.echo = echo
        self.dt = dt
        self.p = p
        self.val = 0.0
        self.subscription = self.create_subscription(
            Wrench, f'/scout/force_bar/segment_{self.sn}',
            self.listener_callback, 10)
        self.publisher = self.create_publisher(
            String, '/scout/force_bar/joined_force_x', 10)
        self.timer = self.create_timer(
            self.dt, self.timer_callback)

    def listener_callback(self, msg):
        if self.ft == 'f':
            if self.xyz == 'x':
                self.val = round(msg.force.x, self.p)
            if self.xyz == 'y':
                self.val = round(msg.force.y, self.p)
            if self.xyz == 'z':
                self.val = round(msg.force.z, self.p)
        if self.ft == 't':
            if self.xyz == 'x':
                self.val = round(msg.torque.x, self.p)
            if self.xyz == 'y':
                self.val = round(msg.torque.y, self.p)
            if self.xyz == 'z':
                self.val = round(msg.torque.z, self.p)
        if self.echo:
            self.get_logger().info(f"""
    force:
        x: {msg.force.x}
        y: {msg.force.y}
        z: {msg.force.z}
    torque:
        x: {msg.torque.x}
        y: {msg.torque.y}
        z: {msg.torque.z}""")

    def timer_callback(self):
        msg = String()
        msg.data = f"{self.sn},{self.ft},{self.xyz},{self.val}"
        self.publisher.publish(msg)
        if self.echo:
            self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)

    sn = int(argv[1])
    subscriber = ForceBarSubPub(sn)

    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
