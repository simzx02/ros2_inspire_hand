#!/usr/bin/env python3

import rospy
import time
import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from inspire_sdkpy import inspire_hand_defaut, inspire_dds
from std_msgs.msg import String

class PickObjectNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pick_object_node_listener')

        # Init Unitree SDK channel factory
        if len(sys.argv) > 1 and not sys.argv[1].startswith("--"):
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0)

        # Publisher & Subscriber for right hand
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
            rospy.logwarn("cmd.force_set not found in inspire_hand_ctrl")

        # Initialize grip parameters
        self.reset_grip_data()

        # Subscribe to /finger_command topic
        rospy.Subscriber("/finger_command", String, self.command_callback)

        rospy.loginfo("PickObjectNode ready. Awaiting finger commands...")
        rospy.spin()  # Keep the node alive and listening for commands

    def reset_grip_data(self):
        """Reset grip variables and initialize current angles from the robot."""
        self.min_angle = 50
        self.step_size = 10
        self.reached = [False] * 6
        self.final_forces = [0.0] * 6
        self.final_angles = [None] * 6
        self.peak_forces = [0.0] * 6
        self.finger_times = [0.0] * 6

        # Try reading the current angles from the robot
        state_r = self.subr.Read(0.1)
        if state_r is not None and hasattr(state_r, "angle_act"):
            self.current_angles = list(state_r.angle_act)
            rospy.loginfo(f"Initialized current angles from robot state: {self.current_angles}")
        else:
            # Fallback to default open angles on first run or error
            self.current_angles = [850] * 6
            rospy.logwarn("Failed to read current angles from robot; defaulting to 850.")



    def open_all_fingers(self, speed=350):
        """
        Gradually open all fingers with speed control using mode 9 (angle + speed).
        :param speed: The speed value to control the release (higher value = faster release).
        """
        rospy.loginfo("Opening fingers with angle + speed control...")

        target_angles = [850] * 6  # Fully open position
        while any(current < target for current, target in zip(self.current_angles, target_angles)):
            for i in range(6):
                if self.current_angles[i] < target_angles[i]:
                    self.current_angles[i] = min(self.current_angles[i] + speed, target_angles[i])

            self.cmd.angle_set = self.current_angles
            self.cmd.speed_set = [speed] * 6
            self.cmd.mode = 0b1001  # Mode 9: angle + speed
            self.pubr.Write(self.cmd)

            time.sleep(0.1)  # Small delay to control the speed of opening

        rospy.loginfo("Fingers fully opened.")

    def pid_close(self):
        """
        Close fingers using a PID-like control mechanism for dynamic adjustment of angle, force, and speed.
        The fingers move faster initially and slow down as they approach the force threshold or minimum angle.
        """
        start_time = time.time()  # Record the start time of the closing process

        # PID parameters
        Kp = 1  # Proportional gain
        Ki = 0.05  # Integral gain
        Kd = 0.05  # Derivative gain

        integral = [0.0] * 6  # Integral term for each finger
        previous_error = [0.0] * 6  # Previous error for derivative calculation

        while not all(self.reached) and any(a >= self.min_angle for a in self.current_angles):
            prev_angles = self.current_angles.copy()

            self.cmd.angle_set = self.current_angles
            self.cmd.mode = 0b1101  # Mode 13: angle + force + speed control
            self.pubr.Write(self.cmd)

            time.sleep(0.1)

            state_r = self.subr.Read(0.1)
            if state_r is not None:
                forces = list(state_r.force_act)
                actual_angles = getattr(state_r, "angle_act", self.current_angles)

                for i in range(6):
                    # Update peak force
                    if forces[i] > self.peak_forces[i]:
                        self.peak_forces[i] = forces[i]

                    # Calculate error
                    error = self.force_limits[i] - forces[i]

                    # Update PID terms
                    integral[i] += error * 0.1  # Accumulate integral term
                    derivative = (error - previous_error[i]) / 0.1  # Calculate derivative term
                    previous_error[i] = error

                    # Calculate dynamic step size using PID control
                    step_size = int(Kp * error + Ki * integral[i] + Kd * derivative)
                    step_size = max(self.step_size, min(step_size, 50))  # Clamp step size between self.step_size and 50

                    # Dynamically adjust speed based on proximity to the threshold
                    speed = max(150, 500 - abs(error))  # Higher speed for larger errors, slower as error decreases

                    # Check if the finger has reached its stopping condition
                    if not self.reached[i]:
                        if forces[i] >= self.force_limits[i] or self.current_angles[i] <= self.min_angle:
                            self.reached[i] = True
                            self.final_angles[i] = actual_angles[i]
                            self.final_forces[i] = forces[i]
                            self.finger_times[i] = time.time() - start_time  # Record the time it took to stop
                        else:
                            if self.current_angles[i] - step_size >= self.min_angle:
                                self.current_angles[i] -= step_size

                    # Update the command with dynamic speed
                    self.cmd.speed_set = [speed] * 6  # Set the speed for all fingers

            if self.current_angles == prev_angles:
                rospy.loginfo("No angle changes detected — motion stopped.")
                break

        rospy.loginfo("PID close completed.")

    def print_summary(self):
        rospy.loginfo("=== Per-Finger Grip Summary (Right Hand) ===")
        for i in range(6):
            stop_angle = self.final_angles[i] if self.final_angles[i] is not None else self.current_angles[i]
            rospy.loginfo(
                f"Finger {i+1}: {self.peak_forces[i]:.1f} g "
                f"(limit {self.force_limits[i]} g), stop angle: {stop_angle}"
            )

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd == "open":
            self.open_all_fingers()
        elif cmd == "close":
            self.reset_grip_data()
            self.pid_close()
            self.print_summary()
        else:
            rospy.logwarn(f"Unknown command: '{cmd}' (expected 'open' or 'close')")

def main():
    try:
        # Start the PickObjectNode
        node = PickObjectNode()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


##rostopic pub --once /finger_command std_msgs/String "close" or open
