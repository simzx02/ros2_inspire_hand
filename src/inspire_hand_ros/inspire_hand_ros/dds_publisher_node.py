#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#The original code here: ~/workspace/inspire_hand/inspire_hand_ws/inspire_hand_sdk/example/dds_publish.py
#A copy of the original code is dds_publisher_485_r.py in this package.

import rclpy
from rclpy.node import Node
import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds

class DDSPublisherNode(Node):
    def __init__(self):
        super().__init__('dds_publisher_node')

        # Initialize Cyclone DDS
        ChannelFactoryInitialize(0)
        self.get_logger().info("Cyclone DDS initialized.")

        # Publishers for left and right hand
        self.pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
        self.pub_r.Init()
        self.pub_l = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
        self.pub_l.Init()
        self.get_logger().info("DDS publishers initialized.")

        # Default command
        self.cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
        self.short_value = 1000

        # Start the periodic publishing timer
        self.timer = self.create_timer(0.1, self.publish_dds_callback)

        # Counter
        self.cnd = 0

    def publish_dds_callback(self):
        """Publish commands periodically, mimicking original loop."""
        try:
            # Flip short_value every 10 iterations
            if (self.cnd + 1) % 10 == 0:
                self.short_value = 1000 - self.short_value

            # Generate values
            values_to_write = [self.short_value] * 6
            values_to_write[-1] = 1000 - values_to_write[-1]
            values_to_write[-2] = 1000 - values_to_write[-2]

            value_np = np.array(values_to_write)
            value_np = np.clip(value_np, 200, 800)

            # Fill DDS command
            self.cmd.angle_set = value_np.tolist()
            self.cmd.mode = 0b0001

            # Publish
            self.pub_l.Write(self.cmd)
            self.pub_r.Write(self.cmd)

            self.cnd += 1

        except Exception as e:
            self.get_logger().warn(f"Failed to publish DDS message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DDSPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
