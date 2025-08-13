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
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String


SEGMENTS:int = 5
SEGMENT_THRESHOLDS:list[float] = [200, 200, 200, 200, 200]
assert len(SEGMENT_THRESHOLDS) == SEGMENTS
STARTING_VELOCITY:float = 0.5
BOX_SIZE:tuple[float, float] = (50.0, 50.0)


def COMMANDS(sn:int=3, val:float=0.0) -> list[tuple]:
    "motion routine when collision happens"
    a1 = 0.5 * (-1 if sn < 3 else random.choice([-1, 1]) if sn == 3 else 1)
    d1 = random.uniform(2, 6)
    #     linear, angular,   duration
    return [
        (   -0.5,       0,        1.5),
        (      0,      a1,         d1),
        (    0.5,       0,          1)
    ].copy()


class RandomRobotForestMotion(Node):

    def __init__(self):
        super().__init__('random_robot_forest_motion')
        self.starting_position = ()
        self.command_stack = []
        self.timer = self.create_timer(0, lambda *args, **kwargs: None)
        self.timer.cancel()
        self.cmd_vel = self.create_publisher(Twist,'/cmd_vel', 10)
        self.collision = self.create_subscription(String, '/merged_force_topic', self.collision_callback, 10)
        self.pose = self.create_subscription(Pose, '/scout/pose', self.pose_callback, 10)
        # for each run a day create a new save file
        self.save_file = ''
        base_str = r'data/recored_positions_' + datetime.now().strftime(r"%y%m%d") + r'_{}.csv'
        for i in range(1, 1000):
            self.save_file = base_str.format(f"{i:03d}")
            if not Path(self.save_file).exists():
                break

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

    def detect_wall(self, pos_x:float, pos_y:float) -> bool:
        return_val = False
        if not self.starting_position:
            self.starting_position = (pos_x, pos_y)
            self.get_logger().info(f"START: x:{pos_x:.2f} y:{pos_y:.2f}")
        else:
            pos_x = self.starting_position[0] - pos_x
            pos_y = self.starting_position[1] - pos_y
            if abs(pos_x) >= BOX_SIZE[0]/2 or abs(pos_y) >= BOX_SIZE[1]/2:
                log_msg = f"WALL x:{pos_x:.2f} y:{pos_y:.2f}"
                if self.command_stack:
                    log_msg += ' (ignored)'
                else:
                    return_val = True
                self.get_logger().info(log_msg)
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
        if self.detect_collision(sn, val, SEGMENT_THRESHOLDS[sn]):
            self.stop()
            self.command_stack = COMMANDS(sn, val)
        if self.timer.is_canceled():
            self.command_callback()

    def pose_callback(self, msg:Pose):
        "Enforce map boundaries and record position data."
        if self.detect_wall(msg.position.x, msg.position.y):
            self.stop()
            self.command_stack = COMMANDS()
        with open(self.save_file, 'a') as f:
            f.write(f"{msg.position.x},{msg.position.y},{len(self.command_stack)}\n")


def main(args=None):
    rclpy.init(args=args)
    node = RandomRobotForestMotion()
    node.move(STARTING_VELOCITY, 0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
