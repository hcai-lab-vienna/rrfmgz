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


import math
import os
import random
from datetime import datetime
from glob import glob

import rclpy

# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Quaternion, Twist
from rclpy.node import Node
from std_msgs.msg import String


def quaternion_to_yaw(q: Quaternion) -> float:
    "Converts a quaternion into yaw (rotation about Z axis). Returns yaw in radians."
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_to_center(pose: tuple) -> float:
    """
    Angle to the origin.
    Why -y and -x?
    Because we are going from (x, y) to (0, 0),
    which is a vector pointing from the robot toward the origin
    """
    return math.atan2(-pose[1], -pose[0])


def normalize_angle(angle: float) -> float:
    "To prevent yaw angle wrapping from -pi to pi, causes discontinuities in rotation"
    return (angle + math.pi) % (2 * math.pi) - math.pi


class RandomRobotForestMotion(Node):
    segment_thresholds: list[float] = [200, 200, 200, 200, 200]
    box_size: tuple[float, float] = (40.0, 40.0)
    record: bool = True
    recover_from_stand_still: bool = True
    always_random_rotation: bool = True

    def __init__(self):
        super().__init__("random_robot_forest_motion")
        # command_stack will be executed by the robot from top to bottom,
        # each command is a tuple of (linear speed, angular speed, duration in seconds).
        self.command_stack = []
        # out_of_bounds will be set to True if the robot is OUT OF BOUNDS (if this isn't obvious oO)
        self.out_of_bounds = False
        # internal pose states of the robot will be continuously updated.
        self.pose = 0.0, 0.0
        self.yaw = 0.0
        self.theta = 0.0  # angle from center of map to robot
        # timer for each command executed by command_stack, last filed of each command_stack tuple will be the duration.
        self.command_timer = self.create_timer(0, lambda: None)
        self.command_timer.cancel()
        # a timer to prevent multiple collision commands in a short amount of time
        self.immediate_collision_timer = self.create_timer(0, lambda: None)
        self.immediate_collision_timer.cancel()
        self.immediate_collision = False
        # ros callbacks
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.collision = self.create_subscription(
            String, "/merged_force_topic", self.collision_callback, 10
        )
        self.pose = self.create_subscription(
            Pose, "/scout/pose", self.pose_callback, 10
        )
        # for each run a day create a new save file
        if self.record:
            base_str = f"data/recored_positions_{datetime.now().strftime(r'%y%m%d')}_"
            file_list = sorted(glob(base_str + "*"))
            if file_list:
                file_nr = 1 + int(file_list[-1].replace(base_str, "").split(".")[0])
            else:
                file_nr = 1
            self.save_file = base_str + f"{file_nr:03d}.csv"

    def collision_commands(self, sn: int = 3) -> list[tuple]:
        "motion routine when collision happens"
        if self.always_random_rotation:
            a2 = 0.5 * random.choice([-1, 1])
        else:
            a2 = 0.5 * (-1 if sn < 3 else random.choice([-1, 1]) if sn == 3 else 1)
        d2 = random.uniform(2, 6)
        return [
            # linear, angular, duration
            (-0.5, 0.0, 1.5),
            (0.0, a2, d2),
            (0.5, 0.0, 0.1),
        ].copy()

    def wall_commands(self) -> list[tuple]:
        "motion routine when out of bounds"
        target_yaw = self.theta - self.yaw
        angle = normalize_angle(target_yaw)
        try:
            sign = angle // abs(angle)
        except ZeroDivisionError:
            sign = 1
        heading = abs(angle) / math.pi
        if heading > 0.1:
            d1 = 5 * heading
            l2 = 0.0
        else:
            d1 = 0.0
            l2 = 0.5
        return [
            # linear,  angular, duration
            (0.0, sign * 0.5, d1),
            (l2, 0.0, 0.1),
        ].copy()

    def move(self, linear: float, angular: float):
        "Move the robot with current values of self.linear & self.angular respectively."
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel.publish(msg)
        self.get_logger().info(f"MOVE lin:{linear:.1f} ang:{angular:.1f}")

    def stop(self):
        "does the stopping"
        self.command_stack = []
        if self.command_timer:
            self.command_timer.cancel()
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel.publish(msg)
        self.get_logger().info("STOP")

    def set_immediate_collision(self):
        "Prevent to many collision detections in a short amount of time."
        if not self.immediate_collision:
            self.immediate_collision = True
            self.immediate_collision_timer = self.create_timer(
                0.5, self.reset_immediate_collision
            )

    def reset_immediate_collision(self):
        "Allows collision detection again, should only be called by set_immediate_collision."
        self.immediate_collision_timer.cancel()
        self.immediate_collision = False

    def detect_collision(self, sn: int, val: float, threshold: float) -> bool:
        "Returns True if val exceeds threshold and no maneuver is in progress, False otherwise."
        return_val = False
        if abs(val) > threshold:
            log_msg = f"COLL sn:{sn} val:{val:.2f}"
            if self.immediate_collision:
                log_msg += " (ignored)"
            else:
                self.set_immediate_collision()
                return_val = True
            self.get_logger().info(log_msg)
        return return_val

    def detect_wall(self) -> bool:
        "Returns True if outside of BOX_SIZE."
        return_val = False
        if (
            abs(self.pose[0]) >= self.box_size[0] / 2
            or abs(self.pose[1]) >= self.box_size[1] / 2
        ):
            log_msg = f"WALL pose_x:{self.pose[0]:.2f} pose_y:{self.pose[1]:.2f} yaw:{self.yaw:.2f}"
            if self.command_stack:
                log_msg += " (ignored)"
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
        self.command_timer.cancel()
        if self.command_stack:
            linear, angular, duration = map(float, self.command_stack.pop(0))
            self.move(linear, angular)
            if duration:
                self.command_timer = self.create_timer(duration, self.command_callback)
                self.get_logger().info(f"WAIT sec:{duration:.2f}")
        elif self.recover_from_stand_still:
            # stand still prevention
            self.command_stack = [(0.5, 0.0, 10.0)]

    def collision_callback(self, msg: String):
        "Detect collisions and calculates movement."
        sn, val = msg.data.split(" ")
        sn = int(sn)
        val = float(val)
        if self.detect_collision(sn, val, self.segment_thresholds[sn]):
            self.stop()
            self.command_stack = self.collision_commands(sn)
        if self.command_timer.is_canceled():
            self.command_callback()

    def pose_callback(self, msg: Pose):
        "Enforce map boundaries and record position data."
        self.pose = msg.position.x, msg.position.y
        self.yaw = quaternion_to_yaw(msg.orientation)
        self.theta = angle_to_center(self.pose)
        if self.detect_wall() and not self.command_stack:
            self.stop()
            self.command_stack = self.wall_commands()
        if self.record:
            with open(self.save_file, "a") as f:
                f.write(
                    f"{self.pose[0]},{self.pose[1]},{self.yaw},{int(self.immediate_collision)},{int(self.out_of_bounds)}\n"
                )


def main(args=None):
    random.seed(os.environ.get("RRFM_SEED", default=None))
    rclpy.init(args=args)
    node = RandomRobotForestMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
