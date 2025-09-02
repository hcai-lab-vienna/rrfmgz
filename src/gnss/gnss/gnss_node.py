#!/usr/bin/env python3

from A7670E import AT

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
# from nmea_msgs.msg import Sentence


class GNSS(Node):

    hz:float = 1.0

    def __init__(self):
        super().__init__('gnss_serial')
        self.publisher = self.create_publisher(NavSatFix, '/gnss/nav_sat_fix', 10)
        # self.publisher = self.create_publisher(Sentence, '/gnss/sentence', 10)
        self.timer = self.create_timer(1.0/self.hz, self.timer_callback)
        self._ser_cmd_port = None
        self._ser_dat_port = None
    
    def __enter__(self):
        self._ser_cmd_port = AT(port='/dev/ttyUSB2')
        self._ser_dat_port = AT(port='/dev/ttyUSB3')
        self._ser_cmd_port.gnss_up()
    
    def __exit__(self, exc_type, exc_value, traceback):
        if self._ser_cmd_port:
            self._ser_cmd_port.gnss_down()
            self._ser_cmd_port.close()
        if self._ser_dat_port:
            self._ser_dat_port.close()
    
    def timer_callback(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude, msg.longitude = self._ser_dat_port.gnss_latlong
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    with GNSS() as node:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()