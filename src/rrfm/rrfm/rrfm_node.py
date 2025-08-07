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


HZ = 1000


import random
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String


# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"


class RandomRobotForestMotion(Node):

    def __init__(self, hz:int, qos_profile:int=10):
        super().__init__('random_robot_forest_motion')
        # --- timing ---
        # fps in combination with frames required to make logic
        # independent of physics calculation frame rate
        self.dt:float = 1.0/float(hz)
        self.fps:int = hz
        self.maneuver_frames:int = 1
        # --- motion ---
        self.linear:float = 0.5
        self.angular:float = 0.0
        self.prev_linear:float = 0.0
        self.prev_angular:float = 0.0
        # --- save file ---
        base_file_str = r'data/recored_positions_' + datetime.now().strftime(r"%y%m%d") + r'_{}.csv'
        for i in range(1, 1000):
            new_base_file_str = base_file_str.format(f"{i:03d}")
            if not Path(new_base_file_str).exists():
                base_file_str = new_base_file_str
                break
        self.save_file:str = base_file_str
        # --- ros ---
        self.merged_force_topic_sub = self.create_subscription(
            msg_type=String,
            topic="/merged_force_topic",
            callback=self.merged_force_topic_callback,
            qos_profile=qos_profile)
        self.cmd_vel_pub = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=qos_profile)
        self.timer = self.create_timer(
            timer_period_sec=self.dt,
            callback=self.timer_callback)
        self.pose_sub = self.create_subscription(
            msg_type=Pose,
            topic="/scout/pose",
            callback=self.pose_sub_callback,
            qos_profile=qos_profile)

    def detect_collision(self, sn:int, val:float, threshold:float) -> bool:
        "Returns True if val exceeds threshold and no maneuver is in progress, False otherwise."
        if abs(val) > threshold:
            log_msg = f"COL sn:{sn} val:{val:.2f}"
            if self.maneuver_frames:
                log_msg += " (discarding, maneuver in progress)"
            self.get_logger().info(log_msg)
            if not self.maneuver_frames:
                return True
        return False

    def calculate_movement(self, sn:int, val:float):
        self.maneuver_frames = 10*self.fps
        self.linear = -0.5

    def merged_force_topic_callback(self, msg):
        "detect collisions and calculate movement"
        sn, val = msg.data.split(' ')
        sn = int(sn)
        val = float(val)
        if self.detect_collision(sn, val, 200):
            self.calculate_movement(sn, val)

    def pub_to_cmd_vel(self):
        "Move the robot with current values of self.linear & self.angular respectively."
        self.get_logger().info(f"MOV lin:{self.linear:.1f} ang:{self.angular:.1f}")
        msg = Twist()
        msg.linear.x = float(self.linear)
        msg.angular.z = float(self.angular)
        self.cmd_vel_pub.publish(msg)

    def timer_callback(self):
        "Send commands to robot if there are frames left."
        if self.maneuver_frames:
            self.maneuver_frames -= 1
            if self.linear != self.prev_linear or self.angular != self.prev_angular:
                self.prev_linear = self.linear
                self.prev_angular = self.angular
                self.pub_to_cmd_vel()
        if self.maneuver_frames == 1:
            self.linear = 0.5

    def pose_sub_callback(self, msg):
        "record position data"
        with open(self.save_file, 'a') as f:
            f.write(f"{msg.position.x},{msg.position.y},{self.maneuver_frames}\n")


def main(args=None):
    rclpy.init(args=args)
    subscriber = RandomRobotForestMotion(HZ)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
