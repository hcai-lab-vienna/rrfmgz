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
        self._ser_dat_port = AT(dat_port, baud_rate)
        if cmd_port == dat_port:
            self._ser_cmd_port = self._ser_dat_port
        elif cmd_port:
            self._ser_cmd_port = AT(cmd_port, baud_rate)
        else:
            self._ser_cmd_port = None
        if self._ser_cmd_port:
            self._ser_cmd_port.gnss_up()

    def __del__(self):
        if self._ser_dat_port:
            self._ser_dat_port.close()
        if self._ser_cmd_port:
            self._ser_cmd_port.close()

    def timer_callback(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        latlong = self._ser_dat_port.gnss_latlong
        if latlong:
            msg.latitude, msg.longitude = latlong
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GNSS('/dev/ttyUSB3', '/dev/ttyUSB2')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()