#!/usr/bin/env python3

import rospy
import time
import sys
import threading
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds
from std_msgs.msg import String, Float32MultiArray

class PickObjectNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pick_object_node_listener')

        # Init Unitree SDK channel factory for DDS control
        if len(sys.argv) > 1 and not sys.argv[1].startswith("--"):
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0)

        # Publisher for right hand (DDS for control only)
        self.pubr = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
        self.pubr.Init()

        # Prepare command
        self.cmd = inspire_hand_defaut.get_inspire_hand_ctrl()

        # Force limits (grams)
        self.force_limits = [300] * 6
        if hasattr(self.cmd, "force_set"):
            self.cmd.force_set = self.force_limits
        else:
            rospy.logwarn("cmd.force_set not found in inspire_hand_ctrl")

        # Global speed parameters
        self.open_speed = 350  # Speed for opening fingers
        self.close_speed = 200  # Base speed for closing fingers

        # Thread-safe storage for sensor data from ROS topics
        self.data_lock = threading.Lock()
        self.current_angles = [850] * 6
        self.current_forces = [0] * 6
        self.current_status = [0] * 6
        self.current_currents = [0] * 6
        self.data_received = False

        # Subscribe to ROS topics published by driver_ros1.py
        rospy.Subscriber("/inspire_hand/angle", Float32MultiArray, self.angle_callback)
        rospy.Subscriber("/inspire_hand/force", Float32MultiArray, self.force_callback)
        rospy.Subscriber("/inspire_hand/status", Float32MultiArray, self.status_callback)
        rospy.Subscriber("/inspire_hand/current", Float32MultiArray, self.current_callback)

        # Subscribe to /finger_command topic for control
        rospy.Subscriber("/finger_command", String, self.command_callback)

        # Initialize grip parameters
        self.reset_grip_data()

        # Wait for initial data
        rospy.loginfo("Waiting for sensor data from driver_ros1.py...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while not self.data_received and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        
        if self.data_received:
            rospy.loginfo(f"Initial sensor data received: Angles={self.current_angles}")
        else:
            rospy.logwarn("No sensor data received after 5 seconds. Using default values.")

        rospy.loginfo("PickObjectNode ready. Awaiting finger commands...")
        rospy.spin()  # Keep the node alive and listening for commands

    def angle_callback(self, msg):
        """Callback for angle data from driver."""
        with self.data_lock:
            # Convert to int for DDS compatibility
            self.current_angles = [int(x) for x in msg.data]
            self.data_received = True

    def force_callback(self, msg):
        """Callback for force data from driver."""
        with self.data_lock:
            # Keep as int for DDS compatibility
            self.current_forces = [int(x) for x in msg.data]

    def status_callback(self, msg):
        """Callback for status data from driver."""
        with self.data_lock:
            # Keep as int for DDS compatibility
            self.current_status = [int(x) for x in msg.data]

    def current_callback(self, msg):
        """Callback for current data from driver."""
        with self.data_lock:
            # Keep as int for DDS compatibility
            self.current_currents = [int(x) for x in msg.data]

    def get_sensor_data(self):
        """Thread-safe method to get current sensor data."""
        with self.data_lock:
            return {
                'angles': self.current_angles.copy(),
                'forces': self.current_forces.copy(),
                'status': self.current_status.copy(),
                'currents': self.current_currents.copy()
            }

    def reset_grip_data(self):
        """Reset grip variables and initialize current angles from sensor data."""
        self.min_angle = 50
        self.step_size = 10
        self.reached = [False] * 6
        self.final_forces = [0.0] * 6
        self.final_angles = [None] * 6
        self.peak_forces = [0.0] * 6
        self.finger_times = [0.0] * 6

        # Get current angles from ROS topic data
        sensor_data = self.get_sensor_data()
        self.target_angles = sensor_data['angles'].copy()
        rospy.loginfo(f"Reset grip data. Current angles: {self.target_angles}")

    def open_all_fingers(self):
        """
        Gradually open all fingers with speed control using mode 9 (angle + speed).
        """
        rospy.loginfo(f"Opening fingers with speed={self.open_speed}...")

        target_angles = [850] * 6  # Fully open position
        sensor_data = self.get_sensor_data()
        current = sensor_data['angles'].copy()

        while any(curr < target for curr, target in zip(current, target_angles)):
            for i in range(6):
                if current[i] < target_angles[i]:
                    current[i] = min(current[i] + self.open_speed, target_angles[i])

            # Ensure integers for DDS
            self.cmd.angle_set = [int(x) for x in current]
            self.cmd.speed_set = [int(self.open_speed)] * 6
            self.cmd.mode = 0b1001  # Mode 9: angle + speed
            self.pubr.Write(self.cmd)

            time.sleep(0.1)
            
            # Update current from sensor feedback
            sensor_data = self.get_sensor_data()
            current = sensor_data['angles'].copy()

        rospy.loginfo("Fingers fully opened.")

    def pid_close(self):
        """
        Close fingers using a PID-like control mechanism for dynamic adjustment of angle, force, and speed.
        Uses ROS topic data for real-time feedback.
        """
        start_time = time.time()

        # PID parameters
        Kp = 5  # Proportional gain
        Ki = 0.05  # Integral gain
        Kd = 0.05  # Derivative gain

        integral = [0.0] * 6
        previous_error = [0.0] * 6

        # Get initial angles from sensor
        sensor_data = self.get_sensor_data()
        command_angles = sensor_data['angles'].copy()

        rospy.loginfo(f"Starting PID close from angles: {command_angles} with base speed={self.close_speed}")

        while not all(self.reached) and any(a >= self.min_angle for a in command_angles):
            prev_angles = command_angles.copy()

            # Send command - ensure all values are integers for DDS
            self.cmd.angle_set = [int(x) for x in command_angles]
            self.cmd.speed_set = [int(self.close_speed)] * 6
            self.cmd.force_set = [int(x) for x in self.force_limits]
            self.cmd.mode = 0b1101  # Mode 13: angle + force + speed control
            self.pubr.Write(self.cmd)

            time.sleep(0.1)

            # Get current sensor data from ROS topics
            sensor_data = self.get_sensor_data()
            forces = sensor_data['forces']
            actual_angles = sensor_data['angles']

            rospy.loginfo_throttle(1.0, f"Forces: {forces}, Angles: {actual_angles}")

            for i in range(6):
                # Update peak force
                if forces[i] > self.peak_forces[i]:
                    self.peak_forces[i] = forces[i]

                # Calculate error
                error = self.force_limits[i] - forces[i]

                # Update PID terms
                integral[i] += error * 0.1
                derivative = (error - previous_error[i]) / 0.1
                previous_error[i] = error

                # Calculate dynamic step size using PID control
                step_size = int(Kp * error + Ki * integral[i] + Kd * derivative)
                step_size = max(self.step_size, min(step_size, 70))

                # Check if the finger has reached its stopping condition
                if not self.reached[i]:
                    if forces[i] >= self.force_limits[i] or command_angles[i] <= self.min_angle:
                        self.reached[i] = True
                        self.final_angles[i] = actual_angles[i]
                        self.final_forces[i] = forces[i]
                        self.finger_times[i] = time.time() - start_time
                        rospy.loginfo(f"Finger {i+1} reached target: Angle={actual_angles[i]}, Force={forces[i]}")
                    else:
                        if command_angles[i] - step_size >= self.min_angle:
                            command_angles[i] -= step_size

            if command_angles == prev_angles:
                rospy.loginfo("No angle changes detected - motion stopped.")
                break

        rospy.loginfo("PID close completed.")

    def print_summary(self):
        """Print summary of grip operation."""
        rospy.loginfo("=== Per-Finger Grip Summary (Right Hand) ===")
        sensor_data = self.get_sensor_data()
        for i in range(6):
            stop_angle = self.final_angles[i] if self.final_angles[i] is not None else sensor_data['angles'][i]
            rospy.loginfo(
                f"Finger {i+1}: "
                f"Stop Angle: {stop_angle:.1f}, "
                f"Final Force: {self.final_forces[i]:.1f}, "
                f"Peak Force: {self.peak_forces[i]:.1f}, "
                f"Time: {self.finger_times[i]:.2f}s"
            )

    def command_callback(self, msg):
        """Handle incoming finger commands."""
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
        node = PickObjectNode()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


##rostopic pub --once /finger_command std_msgs/String "close"
##rostopic pub --once /finger_command std_msgs/String "open"
##rostopic pub --once /finger_command std_msgs/String "close"
##rostopic pub --once /finger_command std_msgs/String "open"