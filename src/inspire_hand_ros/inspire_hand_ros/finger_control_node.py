#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#Modified from inspire_hand_ros/dds_publisher_node.py to control individual fingers in a sequence.

import rclpy
from rclpy.node import Node
import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds

class FingerControlNode(Node):
    def __init__(self):
        super().__init__('finger_control_node')

        # Initialize Cyclone DDS
        ChannelFactoryInitialize(0)
        self.get_logger().info("Cyclone DDS initialized.")

        # Publisher for right hand
        self.pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
        self.pub_r.Init()
        self.get_logger().info("DDS publisher initialized for right hand.")

        # Default command
        self.cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
        self.short_value = 1000

        # Start the periodic publishing timer
        self.timer = self.create_timer(0.1, self.publish_dds_callback)

        # Counter
        self.cnd = 0
        self.finger_closed = False   # <-- NEW

    def publish_dds_callback(self):
        try:
            # Define constants
            #Use 200–800 for normal operation. Not good to use full scale 0 -1000.
            OPEN = 900
            CLOSE = 0

            # Define the hard-coded sequence of finger states
            if not hasattr(self, "sequence"):
                # Each sublist = one "frame" of motion
                # Thumb is always open (index 0)
                # Index finger = joint 1, middle = 2, ring = 3, little = 4, wrist/extra = 5
                self.sequence = [
                #    [CLOSE, OPEN, OPEN, OPEN, OPEN, OPEN],  # 4th finger closed
                #    [OPEN, CLOSE, OPEN, OPEN, OPEN, OPEN], # 3rd finger closed
                #    [OPEN, OPEN, CLOSE, OPEN, OPEN, OPEN], # 2nd finger closed
                #    [OPEN, OPEN, OPEN, CLOSE, OPEN, OPEN], # 1st finger closed
                #    [OPEN, OPEN, OPEN, OPEN, CLOSE, OPEN], # thumb closed
                    [CLOSE, OPEN, OPEN, OPEN, OPEN, CLOSE],  # thumb move to center
                #    [OPEN, OPEN, OPEN, OPEN, OPEN, OPEN],  # open all again
                ]
                self.seq_idx = 0

            # Select current state
            target = self.sequence[self.seq_idx]

            # Fill DDS command
            self.cmd.angle_set = target
            self.cmd.mode = 0b0001

            # Publish to right hand only
            self.pub_r.Write(self.cmd)

            # Advance to next state every 10 cycles (~1 sec)
            if (self.cnd + 1) % 10 == 0:
                self.seq_idx = (self.seq_idx + 1) % len(self.sequence)

            self.cnd += 1

        except Exception as e:
            self.get_logger().warn(f"Failed to publish DDS message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FingerControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
