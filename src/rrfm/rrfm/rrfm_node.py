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
import math
from glob import glob
from datetime import datetime

import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose, Quaternion
from std_msgs.msg import String


class RandomRobotForestMotion(Node):

    segment_thresholds:list[float] = [200, 200, 200, 200, 200]
    starting_velocity:float = 0.5
    box_size:tuple[float, float] = (40.0, 40.0)
    record:bool = True

    @staticmethod
    def quaternion_to_yaw(q:Quaternion) -> float:
        "Converts a quaternion into yaw (rotation about Z axis). Returns yaw in radians."
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y**2 +q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def angle_to_center(pos_x:float, pos_y:float, yaw:float) -> float:
        "Compute the angle pointing towards the center of the box."
        direction_to_center = math.atan2(-pos_x, -pos_y)  # vector from robot to center
        heading = direction_to_center - yaw
        heading = (heading + math.pi)%(2*math.pi) - math.pi  # normalize
        return heading  # in radians

    def __init__(self):
        super().__init__('random_robot_forest_motion')
        self.command_stack = []
        self.out_of_bounds = False
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.timer = self.create_timer(0, lambda *args, **kwargs: None)
        self.timer.cancel()
        self.cmd_vel = self.create_publisher(Twist,'/cmd_vel', 10)
        self.collision = self.create_subscription(String, '/merged_force_topic', self.collision_callback, 10)
        self.pose = self.create_subscription(Pose, '/scout/pose', self.pose_callback, 10)
        # for each run a day create a new save file
        if self.record:
            base_str = f'data/recored_positions_{datetime.now().strftime(r"%y%m%d")}_'
            last_file = sorted(glob(base_str + '*'))[-1]
            file_nr = int(last_file.replace(base_str, '').split('.')[0])
            file_nr += 1
            self.save_file = base_str + f'{file_nr:03d}.csv'
        self.move(self.starting_velocity, 0)

    def collision_commands(self, sn:int=3) -> list[tuple]:
        "motion routine when collision happens"
        a2 = 0.5 * (-1 if sn < 3 else random.choice([-1, 1]) if sn == 3 else 1)
        d2 = random.uniform(2, 6)
        return [
            # linear, angular,   duration
            (   -0.5,     0.0,        1.5),
            (    0.0,      a2,         d2),
            (    0.5,     0.0,        0.1)
        ].copy()

    def wall_commands(self) -> list[tuple]:
        "motion routine when out of bounds"
        heading = self.angle_to_center(self.pose_x, self.pose_y, self.yaw)
        sign = heading // abs(heading)
        heading = abs(heading)
        l1 = -0.5 if heading > math.pi/2 else 0.5
        a2 = sign*0.5
        d2 = 6*heading/math.pi
        return [
            # linear, angular,   duration
            (     l1,     0.0,        1.5),
            (    0.0,      a2,         d2),
            (    0.5,       0,        0.1)
        ].copy()

    def move(self, linear:float, angular:float):
        "Move the robot with current values of self.linear & self.angular respectively."
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel.publish(msg)
        self.get_logger().info(f"MOVE lin:{linear:.1f} ang:{angular:.1f}")

    def stop(self):
        "does the stopping"
        self.command_stack = []
        if self.timer:
            self.timer.cancel()
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel.publish(msg)
        self.get_logger().info("STOP")

    def detect_collision(self, sn:int, val:float, threshold:float) -> bool:
        "Returns True if val exceeds threshold and no maneuver is in progress, False otherwise."
        return_val = False
        if abs(val) > threshold:
            log_msg = f"COLL sn:{sn} val:{val:.2f}"
            if self.command_stack:
                log_msg += ' (ignored)'
            else:
                return_val = True
            self.get_logger().info(log_msg)
        return return_val

    def detect_wall(self) -> bool:
        "Returns True if outside of BOX_SIZE."
        return_val = False
        if abs(self.pose_x) >= self.box_size[0]/2 or abs(self.pose_y) >= self.box_size[1]/2:
            log_msg = f"WALL pose_x:{self.pose_x:.2f} pose_y:{self.pose_y:.2f} yaw:{self.yaw:.2f}"
            if self.command_stack:
                log_msg += ' (ignored)'
            else:
                return_val = True
            self.get_logger().info(log_msg)
            # set out of bounds check
            self.out_of_bounds = True
        else:
            self.out_of_bounds = False
        return return_val

    def command_callback(self):
        "Schedules commands from command_stack and waits duration for next command (recursive)."
        self.timer.cancel()
        if self.command_stack:
            linear, angular, duration = map(float, self.command_stack.pop(0))
            self.move(linear, angular)
            if duration:
                self.timer = self.create_timer(duration, self.command_callback)
                self.get_logger().info(f"WAIT sec:{duration:.2f}")

    def collision_callback(self, msg:String):
        "Detect collisions and calculates movement."
        sn, val = msg.data.split(' ')
        sn = int(sn)
        val = float(val)
        if self.detect_collision(sn, val, self.segment_thresholds[sn]):
            self.stop()
            self.command_stack = self.collision_commands(sn)
        if self.timer.is_canceled():
            self.command_callback()

    def pose_callback(self, msg:Pose):
        "Enforce map boundaries and record position data."
        self.pose_x = msg.position.x
        self.pose_y = msg.position.y
        self.yaw = self.quaternion_to_yaw(msg.orientation)
        if self.detect_wall() and not self.command_stack:
            self.stop()
            self.command_stack = self.wall_commands()
        if self.record:
            with open(self.save_file, 'a') as f:
                f.write(f"{self.pose_x},{self.pose_y},{len(self.command_stack)},{self.yaw}\n")


def main(args=None):
    rclpy.init(args=args)
    node = RandomRobotForestMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
