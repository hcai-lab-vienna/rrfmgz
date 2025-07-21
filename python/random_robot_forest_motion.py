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


class RandomRobotForestMotion(Node):

    def __init__(self, dt:float):
        """
        dt.... timer interval
        """
        super().__init__('random_robot_forest_motion')
        self.dt = dt
        self.lin = 0.0
        self.ang = 0.0
        self.cf = 0  # control frames
        self.fps = int(1/self.dt)
        self.subscription = self.create_subscription(
            String,
            "/scout/force_bar/joined_force_x",
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.timer = self.create_timer(
            dt,
            self.timer_callback)

    def listener_callback(self, msg):
        sn, val = msg.data.split(' ')
        val = float(val)
        r = random.randint(2, 5)
        cc = val < -300  # collision condition
        if cc:
            self.cf = 7 * self.fps
        if 6*self.fps < self.cf < 7*self.fps:
            self.lin = -3.0
            self.ang = 0.0
        if r*self.fps < self.cf < 6*self.fps:
            self.lin = 0
            self.ang = 1.0
        if 1*self.fps < self.cf < r*self.fps:
            self.lin = 3.0
            self.ang = 0
        if cc:
            self.get_logger().info(f"COL sn:{sn} val:{val:.2f}")

    def timer_callback(self):
        msg = Twist()
        if self.cf:
            self.cf -= 1
            msg.linear.x = float(self.lin)
            msg.angular.z = float(self.ang)
            self.publisher.publish(msg)
            if self.cf % self.fps == 0:
                self.get_logger().info(f"MOV lin:{self.lin:.1f} ang:{self.ang:.1f}")


def main(args=None):
    rclpy.init(args=args)
    subscriber = RandomRobotForestMotion(
        dt=0.01)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
