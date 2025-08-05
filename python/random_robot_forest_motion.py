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
from datetime import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String


# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"


class RandomRobotForestMotion(Node):

    def __init__(self, collision_force:float, hz:float, qos_profile:int=10):
        """
        dt.... timer interval
        """
        super().__init__('random_robot_forest_motion')
        self.dt = 1.0/float(hz)
        # fps in combination with frames required to make logic
        # independent of physics calculation frame rate
        self.fps:float = float(1/self.dt)
        self.frames:int = 0
        self.cf:float = abs(collision_force)
        self.linear:float = 0.0
        self.angular:float = 0.0
        self.collision:bool = False
        self.merged_force_topic_sub = self.create_subscription(
            msg_type=String,
            topic="/merged_force_topic",
            callback=self.merged_force_topic_callback,
            qos_profile=qos_profile)
        self.pose_sub = self.create_subscription(
            msg_type=Pose,
            topic="/scout/pose",
            callback=self.pose_sub_callback,
            qos_profile=qos_profile)
        self.cmd_vel_pub = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=qos_profile)
        self.timer = self.create_timer(
            timer_period_sec=self.dt,
            callback=self.timer_callback)

    def calculate_movement(self, sn:int, val:float):
        self.collision = abs(val) >= self.cf  # collision condition
        lin_speed:float = 0.5
        ang_speed:float = 0.5
        td:float = 7.0  # total duration
        bd:float = 1.5  # backwards drive duration
        rd:float = random.uniform(2.0, td-bd-1.0)  # rotation duration
        if self.collision:
            self.frames = int(td * self.fps) + 1
        if (td-bd)*self.fps < self.frames <= td*self.fps:
            self.linear = -lin_speed
            self.angular = 0.0
        elif self.angular == 0:  # only set angular direction once per collision
            if sn == 3:
                ang_speed = random.choice([1.0, -1.0]) * ang_speed
            elif 3 < sn:
                ang_speed = -ang_speed
        if rd*self.fps < self.frames <= (td-bd)*self.fps:
            self.linear = 0.0
            self.angular = ang_speed
        if 0.0 < self.frames <= rd*self.fps:
            self.linear = lin_speed
            self.angular = 0.0

    def merged_force_topic_callback(self, msg):
        sn, val = msg.data.split(' ')
        sn = int(sn)
        val = float(val)
        self.calculate_movement(sn, val)
        if self.collision:
            self.get_logger().info(f"COL sn:{sn} val:{val:.2f}")
    
    def pose_sub_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        # z = msg.position.z
        # now = datetime.now().strftime('%Y%m%d%H%M:%S%f')
        # with open(f'data/recored_positions_{now}.csv', 'w') as f:
        with open(f'data/recored_positions.csv', 'a') as f:
            f.write(f"{x},{y},{int(bool(self.frames))}\n")

    def timer_callback(self):
        if self.frames:
            self.frames -= 1
            msg = Twist()
            msg.linear.x = float(self.linear)
            msg.angular.z = float(self.angular)
            self.cmd_vel_pub.publish(msg)
            if self.frames % self.fps == 0:
                self.get_logger().info(f"MOV lin:{self.linear:.1f} ang:{self.angular:.1f}")


def main(args=None):
    rclpy.init(args=args)
    subscriber = RandomRobotForestMotion(
        collision_force=100,
        hz=1000)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
