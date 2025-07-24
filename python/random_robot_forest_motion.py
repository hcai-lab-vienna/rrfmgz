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

    def __init__(self, hz:float):
        """
        dt.... timer interval
        """
        super().__init__('random_robot_forest_motion')
        self.dt = 1.0/float(hz)
        # fps in combination with frames required to make logic
        # independent of physics calculation frame rate
        self.fps:float = float(1/self.dt)
        self.frames:int = 0
        self.linear:float = 0.0
        self.angular:float = 0.0
        self.collision:bool = False
        self.subscription = self.create_subscription(
            msg_type=String,
            topic="/merged_force_topic",
            callback=self.listener_callback,
            qos_profile=10)
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=10)
        self.timer = self.create_timer(
            timer_period_sec=self.dt,
            callback=self.timer_callback)

    def calculate_movement(self, sn:int, val:float):
        self.collision = val < -300  # collision condition
        lin_speed:float = 1.0
        ang_speed:float = 0.5
        td:float = 7.0  # total duration
        bd:float = 1.0  # backwards drive duration
        rd:float = random.uniform(2.0, td-bd-1.0)  # rotation duration
        if self.collision:
            self.frames = int(td * self.fps) + 1
        if (td-bd)*self.fps < self.frames <= td*self.fps:
            self.linear = -1.0 * lin_speed
            self.angular = 0.0 * ang_speed
        elif self.angular == 0:  # only set angular direction once per collision
            if sn == 3:
                ang_speed = random.choice([1.0, -1.0]) * ang_speed
            elif 3 < sn:
                ang_speed = -1.0 * ang_speed
        if rd*self.fps < self.frames <= (td-bd)*self.fps:
            self.linear = 0.0 * lin_speed
            self.angular = 1.0 * ang_speed
        if 0.0 < self.frames <= rd*self.fps:
            self.linear = 1.0 * lin_speed
            self.angular = 0.0 * ang_speed

    def listener_callback(self, msg):
        sn, val = msg.data.split(' ')
        sn = int(sn)
        val = float(val)
        self.calculate_movement(sn, val)
        if self.collision:
            self.get_logger().info(f"COL sn:{sn} val:{val:.2f}")

    def timer_callback(self):
        msg = Twist()
        if self.frames:
            self.frames -= 1
            msg.linear.x = float(self.linear)
            msg.angular.z = float(self.angular)
            self.publisher.publish(msg)
            if self.frames % self.fps == 0:
                self.get_logger().info(f"MOV lin:{self.linear:.1f} ang:{self.angular:.1f}")


def main(args=None):
    rclpy.init(args=args)
    subscriber = RandomRobotForestMotion(
        hz=10)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
