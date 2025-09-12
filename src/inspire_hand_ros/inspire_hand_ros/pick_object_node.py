#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from inspire_sdkpy import inspire_hand_defaut, inspire_dds


class PickObjectNode(Node):
    def __init__(self):
        super().__init__('pick_object_node')

        # Init Unitree SDK channel factory
        if len(sys.argv) > 1:
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0)

        # Publisher & subscriber for right hand
        self.pubr = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
        self.pubr.Init()
        self.subr = ChannelSubscriber("rt/inspire_hand/state/r", inspire_dds.inspire_hand_state)
        self.subr.Init()

        # Prepare command
        self.cmd = inspire_hand_defaut.get_inspire_hand_ctrl()

        # Force limits (grams)
        self.force_limits = [300] * 6
        if hasattr(self.cmd, "force_set"):
            self.cmd.force_set = self.force_limits
        else:
            self.get_logger().warn("cmd.force_set not found in inspire_hand_ctrl")

        # Grip parameters
        self.current_angles = [850] * 6
        self.min_angle = 100
        self.step_size = 10
        self.reached = [False] * 6
        self.final_forces = [0.0] * 6
        self.final_angles = [None] * 6

        # Open all fingers first
        self.open_all_fingers()
        time.sleep(2.0)

        self.get_logger().info("Closing fingers...")
        self.progressive_close()

        self.print_summary()

        self.get_logger().info("Press Ctrl+C to open all fingers and exit.")
        try:
            while rclpy.ok():
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt detected — opening all fingers.")
            self.open_all_fingers()
            time.sleep(1.0)

    def open_all_fingers(self):
        self.cmd.angle_set = [850] * 6
        self.cmd.mode = 0b0001  # angle mode for opening
        self.pubr.Write(self.cmd)
        self.get_logger().info("Opening fingers...")

    def progressive_close(self):
        while not all(self.reached) and any(a >= self.min_angle for a in self.current_angles):
            prev_angles = self.current_angles.copy()

            self.cmd.angle_set = self.current_angles
            self.cmd.mode = 0b0101  # angle + force control
            self.pubr.Write(self.cmd)

            time.sleep(0.1)

            state_r = self.subr.Read(0.05)
            if state_r is not None:
                forces = list(state_r.force_act)
                self.final_forces = forces
                actual_angles = getattr(state_r, "angle_act", self.current_angles)

                for i in range(6):
                    if not self.reached[i]:
                        if forces[i] >= self.force_limits[i]:
                            self.reached[i] = True
                            self.final_angles[i] = actual_angles[i]
                        else:
                            if self.current_angles[i] - self.step_size >= self.min_angle:
                                self.current_angles[i] -= self.step_size

            if self.current_angles == prev_angles:
                self.get_logger().info("No angle changes detected — motion stopped.")
                break

    def print_summary(self):
        self.get_logger().info("=== Per-Finger Progressive Grip Summary (Right Hand) ===")
        for i in range(6):
            status = "Reached" if self.reached[i] else "Not reached"
            stop_angle = self.final_angles[i] if self.final_angles[i] is not None else self.current_angles[i]
            self.get_logger().info(
                f"Finger {i+1}: {self.final_forces[i]:.1f} g "
                f"(limit {self.force_limits[i]} g) -> {status}, stop angle: {stop_angle}"
            )


def main(args=None):
    rclpy.init(args=args)
    PickObjectNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
