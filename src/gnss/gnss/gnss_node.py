#!/usr/bin/env python3

from gnss.A7670E import AT, latlong_displacement_to_xy_meter

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix


class GNSS(Node):

    hz:float = 1.0

    def __init__(self, dat_port='/dev/AMA0', cmd_port=None, baud_rate=115200):
        super().__init__('gnss_serial')
        self.publisher = self.create_publisher(NavSatFix, '/gnss/nav_sat_fix', 10)
        self.timer = self.create_timer(1.0/self.hz, self.timer_callback)
        self.ser_dat = AT(dat_port, baud_rate)
        if cmd_port == dat_port:
            self.ser_cmd = self.ser_dat
        elif cmd_port:
            self.ser_cmd = AT(cmd_port, baud_rate)
        else:
            self.ser_cmd = None
        if self.ser_cmd:
            self.ser_cmd.gnss_up()
        self.latlong = None
        self.old_latlong = None
        self.frame_id = 0

    def __del__(self):
        if self.ser_dat:
            self.ser_dat.close()
        if self.ser_cmd:
            self.ser_cmd.close()

    def timer_callback(self):
        latlong = self.ser_dat.gnss_latlong
        if latlong:
            self.latlong = latlong
        if self.latlong and self.latlong != self.old_latlong:
            self.old_latlong = self.latlong
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = str(self.frame_id)
            self.frame_id += 1
            msg.latitude, msg.longitude = self.latlong
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GNSS('/dev/ttyUSB3', '/dev/ttyUSB2')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()