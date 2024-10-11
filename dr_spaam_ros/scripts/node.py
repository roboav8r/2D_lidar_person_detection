#!/usr/bin/env python

import rclpy
from dr_spaam_ros.dr_spaam_ros import DrSpaamROS

def main(args=None):
    rclpy.init(args=args)

    dr_spaam_ros = DrSpaamROS()

    rclpy.spin(dr_spaam_ros)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dr_spaam_ros.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()