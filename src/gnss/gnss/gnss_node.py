#!/usr/bin/env python3

from A7670E import AT, latlong_displacement_to_xy_meter

import rclpy
from rclpy.node import Node


class GNSS(Node):

    def __init__(self):
        super().__init__('gnss_serial')


def main(args=None):
    rclpy.init(args=args)
    node = GNSS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()