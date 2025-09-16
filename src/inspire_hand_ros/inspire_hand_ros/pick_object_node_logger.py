import csv
import datetime
import sys
import time
import rclpy
import os
from rclpy.node import Node

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from inspire_sdkpy import inspire_hand_defaut, inspire_dds


class PickObjectNode(Node):
    def __init__(self):
        super().__init__('pick_object_node_logger')

        # Init Unitree SDK channel factory
        if len(sys.argv) > 1 and not sys.argv[1].startswith("--"):
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
        self.peak_forces = [0.0] * 6
        self.finger_times = [0.0] * 6

        # Open all fingers first
        self.open_all_fingers()
        time.sleep(2.0)

        self.get_logger().info("Closing fingers...")
        self.progressive_close() #OR Other closing method

        self.print_summary()
        self.log_to_csv()

        self.get_logger().info("Press Ctrl+C to open all fingers and exit.")
        try:
            while rclpy.ok():
                time.sleep(0.1)

        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt detected — opening all fingers.")
            self.open_all_fingers()
            time.sleep(1.0)

    #def open_all_fingers(self):
    #    self.cmd.angle_set = [850] * 6
    #    self.cmd.mode = 0b0001  # angle mode for opening
    #    self.pubr.Write(self.cmd)
    #    self.get_logger().info("Opening fingers...")

    def open_all_fingers(self, speed=200): #200 is acceptable speed
        """
        Gradually open all fingers with speed control using mode 9 (angle + speed).

        :param speed: The speed value to control the release (higher value = faster release).
        """
        self.get_logger().info("Opening fingers with angle + speed control...")

        # Start from the current angles and gradually increase to the fully open position
        target_angles = [850] * 6  # Fully open position
        while any(current < target for current, target in zip(self.current_angles, target_angles)):
            for i in range(6):
                if self.current_angles[i] < target_angles[i]:
                    self.current_angles[i] = min(self.current_angles[i] + speed, target_angles[i])

            # Send the updated angles and speed to the hand
            self.cmd.angle_set = self.current_angles
            self.cmd.speed_set = [speed] * 6  # Set the speed for all fingers
            self.cmd.mode = 0b1001  # Mode 9: angle + speed
            self.pubr.Write(self.cmd)

            time.sleep(0.1)  # Small delay to control the speed of opening

        self.get_logger().info("Fingers fully opened.")

    def progressive_close(self):
        start_time = time.time()  # Record the start time of the closing process

        while not all(self.reached) and any(a >= self.min_angle for a in self.current_angles):
            prev_angles = self.current_angles.copy()

            self.cmd.angle_set = self.current_angles
            self.cmd.mode = 0b0101  # angle + force control
            self.pubr.Write(self.cmd)

            time.sleep(0.1)

            state_r = self.subr.Read(0.05)
            if state_r is not None:
                forces = list(state_r.force_act)
                actual_angles = getattr(state_r, "angle_act", self.current_angles)

                for i in range(6):
                    # Update peak force
                    if forces[i] > self.peak_forces[i]:
                        self.peak_forces[i] = forces[i]

                    # Check if the finger has reached its stopping condition
                    if not self.reached[i]:
                        if forces[i] >= self.force_limits[i] or self.current_angles[i] <= self.min_angle:
                            self.reached[i] = True
                            self.final_angles[i] = actual_angles[i]
                            self.final_forces[i] = forces[i]
                            self.finger_times[i] = time.time() - start_time  # Record the time it took to stop
                        else:
                            if self.current_angles[i] - self.step_size >= self.min_angle:
                                self.current_angles[i] -= self.step_size

            if self.current_angles == prev_angles:
                self.get_logger().info("No angle changes detected — motion stopped.")
                break

    def print_summary(self):
        self.get_logger().info("=== Per-Finger Progressive Grip Summary (Right Hand) ===")
        for i in range(6):
            #status = "Reached" if self.reached[i] else "Not reached"
            stop_angle = self.final_angles[i] if self.final_angles[i] is not None else self.current_angles[i]
            self.get_logger().info(
                f"Finger {i+1}: {self.final_forces[i]:.1f} g "
                f"(limit {self.force_limits[i]} g), stop angle: {stop_angle}"
            )

    import os  # Add this import at the top of the file

    def log_to_csv(self):
        csv_file = "grip_log.csv"
        file_exists = os.path.exists(csv_file)

        # Define the header
        header = [
            "Timestamp",
            "Finger1 Peak Force", "Finger2 Peak Force", "Finger3 Peak Force",
            "Finger4 Peak Force", "Finger5 Peak Force", "Finger6 Peak Force",
            "Finger1 Final Force", "Finger2 Final Force", "Finger3 Final Force",
            "Finger4 Final Force", "Finger5 Final Force", "Finger6 Final Force",
            "Finger1 Final Angle", "Finger2 Final Angle", "Finger3 Final Angle",
            "Finger4 Final Angle", "Finger5 Final Angle", "Finger6 Final Angle",
            "Finger1 Time", "Finger2 Time", "Finger3 Time",
            "Finger4 Time", "Finger5 Time", "Finger6 Time"
        ]

        # Prepare the data row
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        csv_data = [timestamp]

        # Record peak forces, final forces, final angles, and times
        csv_data.extend(self.peak_forces)  # Peak forces
        csv_data.extend(self.final_forces)  # Final forces
        csv_data.extend(self.final_angles)  # Final angles
        csv_data.extend(self.finger_times)  # Time for each finger to stop

        # Write to CSV
        with open(csv_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            if not file_exists:  # Write the header only if the file doesn't exist
                writer.writerow(header)
            writer.writerow(csv_data)

        self.get_logger().info(f"Logged data to {csv_file}")

def main(args=None):
    rclpy.init(args=args)
    PickObjectNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()