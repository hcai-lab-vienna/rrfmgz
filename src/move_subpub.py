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

import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"


class MoveSubPub(Node):

    def __init__(self, dt:float, *, ft:str='f', xyz:str='x'):
        """
        dt.... timer interval (mendatory)
        ft.... force 'f' or torque 't' sel. for merged pub
        xyz... 'x', 'y' or 'z' selection for merged pub
        """
        super().__init__('scout_move_subpub')
        self.dt = dt
        self.ft = ft
        self.xyz = xyz
        self.linear = 0.0
        self.angular = 0.0
        self.collision = 0
        self.fps = int(1/self.dt)
        self.subscription = self.create_subscription(
            String,
            f"/scout/force_bar/joined_{'torque' if ft == 't' else 'force'}_{xyz}",
            self.listener_callback,
            10)
        self.pub_topic = '/cmd_vel'
        self.publisher = self.create_publisher(
            Twist,
            self.pub_topic,
            10)
        self.timer = self.create_timer(
            dt,
            self.timer_callback)

    def listener_callback(self, msg):
        sn, val = msg.data.split(' ')
        val = float(val)
        r = random.randint(2, 5)
        collision_condition = val < -300
        if collision_condition:
            self.collision = 7 * self.fps
        if 6*self.fps < self.collision < 7*self.fps:
            self.linear = -3.0
            self.angular = 0.0
        if r*self.fps < self.collision < 6*self.fps:
            self.linear = 0
            self.angular = 1.0
        if 1*self.fps < self.collision < r*self.fps:
            self.linear = 3.0
            self.angular = 0
#-LOG-BLOCK-START-------------------------------------------------------#
        if collision_condition:
            self.get_logger().info(f"""
    collision detected for:
        sensor_n: {int(sn):4d}
        {'torque' if self.ft == 't' else ' force'}_{self.xyz}: {val:8.2f}""")
#-LOG-BLOCK-END---------------------------------------------------------#

    def timer_callback(self):
        msg = Twist()
        if self.collision:
            self.collision -= 1
            msg.linear.x = float(self.linear)
            msg.angular.z = float(self.angular)
            self.publisher.publish(msg)
#-LOG-BLOCK-START-----------------------------------------------------------------#
            if self.collision % self.fps == 0:
                self.get_logger().info(f"""
    publishing:
        topic: {self.pub_topic}
         data: {{linear: {{x: {self.linear}}}, angular: {{z: {self.angular}}}}}""")
#-LOG-BLOCK-END-------------------------------------------------------------------#


def main(args=None):
    rclpy.init(args=args)
    subscriber = MoveSubPub(dt=0.01)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
