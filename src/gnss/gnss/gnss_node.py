#!/usr/bin/env python3

from gnss.A7670E import AT

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix


class GNSS(Node):

    def __init__(self, dat_port:str='/dev/AMA0', cmd_port:str|bool=False,
                 baud_rate:int=115200, hz:float=1.0):
        super().__init__('gnss_serial')
        self.publisher = self.create_publisher(NavSatFix, '/gnss/nav_sat_fix', 10)
        self.timer = self.create_timer(1.0/hz, self.timer_callback)
        self.ser_dat = AT(dat_port, baud_rate)
        # Only connect to command port and start GPS if cmd_port is defined.
        # It's possible to use the same port for commands and data,
        # but in practice it doesn't work that great.
        if cmd_port == dat_port or cmd_port == True:
            self.ser_cmd = self.ser_dat
        elif cmd_port:
            self.ser_cmd = AT(cmd_port, baud_rate)
        else:
            self.ser_cmd = None
        if self.ser_cmd:
            self.ser_cmd.gnss_up()
        # Store last successful position measurement.
        self.latlong = AT.GNSS_STARTING_POSITION
        # Increment frame_id on a successful measurement.
        self.frame_id = 0

    def __del__(self):
        if self.ser_dat:
            self.ser_dat.close()
        if self.ser_cmd:
            self.ser_cmd.close()

    def timer_callback(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.frame_id)
        latlong = self.ser_dat.gnss_latlong
        # if successful measurement: update self data
        if latlong:
            self.latlong = latlong
            self.frame_id += 1
        # only publish self data
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