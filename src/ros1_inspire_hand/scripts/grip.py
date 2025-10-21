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

        # Control parameters
        self.force_limits = [300] * 6  # Force limit in grams
        self.min_angle = 0  # Fully closed
        self.max_angle = 850  # Fully open
        self.open_speed = 350
        self.close_speed = 150
        self.step_size = 10  # Angle step per iteration
        
        # Fixed finger 6 (thumb abduction)
        self.finger6_fixed = True
        self.finger6_angle = 850

        # Thread-safe storage for sensor data
        self.data_lock = threading.Lock()
        self.current_angles = [850] * 6
        self.current_forces = [0] * 6
        self.data_received = False

        # Subscribe to ROS topics
        rospy.Subscriber("/inspire_hand/angle", Float32MultiArray, self.angle_callback)
        rospy.Subscriber("/inspire_hand/force", Float32MultiArray, self.force_callback)
        rospy.Subscriber("/finger_command", String, self.command_callback)
        
        # Slip detection parameters
        self.slip_detection_active = False
        self.slip_detection_thread = None
        self.stop_slip_detection = False

        # Wait for initial data
        rospy.loginfo("Waiting for sensor data...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while not self.data_received and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        
        if self.data_received:
            rospy.loginfo(f"Ready. Initial angles: {self.current_angles}")
        else:
            rospy.logwarn("No sensor data received. Using defaults.")

        # Auto-open fingers on startup
        rospy.loginfo("Auto-opening fingers on startup...")
        self.open_all_fingers()
        rospy.loginfo("Initialization complete. Ready for commands.")

        rospy.spin()

    def angle_callback(self, msg):
        with self.data_lock:
            self.current_angles = [int(x) for x in msg.data]
            self.data_received = True

    def force_callback(self, msg):
        with self.data_lock:
            self.current_forces = [int(x) for x in msg.data]

    def get_sensor_data(self):
        with self.data_lock:
            return self.current_angles.copy(), self.current_forces.copy()

    def open_all_fingers(self):
        """Open all fingers to max angle."""
        rospy.loginfo("Opening fingers...")
        
        target = [self.max_angle] * 6
        if self.finger6_fixed:
            target[5] = self.finger6_angle
        
        self.cmd.angle_set = target
        self.cmd.speed_set = [self.open_speed] * 6
        self.cmd.mode = 0b1001  # Angle + speed
        self.pubr.Write(self.cmd)
        
        # Wait for movement to complete (with sensor lag tolerance)
        time.sleep(2.0)
        
        angles, _ = self.get_sensor_data()
        rospy.loginfo(f"Opened. Final angles: {angles}")

    def close_with_force_control(self):
        """
        Close fingers with PID control until force limit or minimum angle reached.
        PID adjusts step size dynamically based on force feedback.
        """
        start_time = time.time()
        
        # PID parameters
        Kp = 0.8  # Proportional gain (step size response to error)
        Ki = 0.02  # Integral gain (accumulates error over time)
        Kd = 0.1  # Derivative gain (responds to rate of change)
        
        # Initialize tracking
        reached = [False] * 6
        final_angles = [None] * 6
        final_forces = [None] * 6
        
        # PID state variables
        integral = [0.0] * 6
        previous_error = [0.0] * 6
        
        # Get starting position
        angles, forces = self.get_sensor_data()
        command_angles = angles.copy()
        
        if self.finger6_fixed:
            command_angles[5] = self.finger6_angle
            reached[5] = True
        
        rospy.loginfo(f"Closing from angles: {command_angles}")
        
        iteration = 0
        max_iterations = 100
        stabilization_counter = [0] * 6
        
        while not all(reached) and iteration < max_iterations:
            iteration += 1
            
            # Update command angles with PID for unreached fingers
            for i in range(6):
                if not reached[i]:
                    # Calculate error (how much more force we want)
                    force_error = self.force_limits[i] - forces[i]
                    
                    # Update PID terms
                    integral[i] += force_error * 0.15
                    integral[i] = max(-500, min(500, integral[i]))  # Anti-windup
                    
                    derivative = (force_error - previous_error[i]) / 0.15
                    previous_error[i] = force_error
                    
                    # Calculate dynamic step size using PID
                    pid_output = Kp * force_error + Ki * integral[i] + Kd * derivative
                    dynamic_step = int(max(5, min(pid_output, 100)))  # Clamp between 5-100
                    
                    # Apply step
                    command_angles[i] = max(command_angles[i] - dynamic_step, self.min_angle)
            
            # Keep finger 6 fixed
            if self.finger6_fixed:
                command_angles[5] = self.finger6_angle
            
            # Send command
            self.cmd.angle_set = command_angles
            self.cmd.speed_set = [self.close_speed] * 6
            self.cmd.force_set = self.force_limits
            self.cmd.mode = 0b1101  # Angle + force + speed
            self.pubr.Write(self.cmd)
            
            # Wait for movement
            time.sleep(0.15)
            
            # Read actual state
            actual_angles, actual_forces = self.get_sensor_data()
            forces = actual_forces  # Update for next PID calculation
            
            # Check each finger
            for i in range(6):
                if reached[i]:
                    continue
                
                # Stop condition 1: Force limit reached
                if actual_forces[i] >= self.force_limits[i]:
                    reached[i] = True
                    final_angles[i] = actual_angles[i]
                    final_forces[i] = actual_forces[i]
                    rospy.loginfo(f"F{i+1}: Force limit reached ({actual_forces[i]}g at {actual_angles[i]}°)")
                    continue
                
                # Stop condition 2: Reached minimum angle (with tolerance)
                if actual_angles[i] <= (self.min_angle + 15):
                    stabilization_counter[i] += 1
                    if stabilization_counter[i] >= 3:  # Wait 3 iterations to confirm
                        reached[i] = True
                        final_angles[i] = actual_angles[i]
                        final_forces[i] = actual_forces[i]
                        rospy.loginfo(f"F{i+1}: Min angle reached ({actual_angles[i]}°, Force={actual_forces[i]}g)")
                else:
                    stabilization_counter[i] = 0
                
                # Stop condition 3: Stuck (no movement for several iterations)
                if iteration > 5:  # Allow initial movement
                    if hasattr(self, '_prev_angles'):
                        movement = abs(actual_angles[i] - self._prev_angles[i])
                        if movement < 3:  # Less than 3 degrees movement
                            stabilization_counter[i] += 1
                            if stabilization_counter[i] >= 5:  # Stuck for 5 iterations
                                reached[i] = True
                                final_angles[i] = actual_angles[i]
                                final_forces[i] = actual_forces[i]
                                rospy.loginfo(f"F{i+1}: Stopped (stuck at {actual_angles[i]}°, Force={actual_forces[i]}g)")
                        else:
                            stabilization_counter[i] = 0
            
            # Save current state for next iteration
            self._prev_angles = actual_angles.copy()
            
            # Log progress every second
            if iteration % 7 == 0:
                rospy.loginfo(f"Iter {iteration}: Cmd={[int(x) for x in command_angles[:5]]}, "
                             f"Actual={actual_angles[:5]}, Forces={actual_forces[:5]}")
        
        # Summary
        elapsed = time.time() - start_time
        
        # Allow forces to settle and take average of multiple readings
        rospy.loginfo("Grip complete. Reading settled forces...")
        time.sleep(0.3)
        
        # Take 5 force readings and average them for stability
        force_samples = []
        for _ in range(5):
            _, sample_forces = self.get_sensor_data()
            force_samples.append(sample_forces)
            time.sleep(0.05)
        
        # Calculate average forces
        avg_forces = [int(sum(f[i] for f in force_samples) / len(force_samples)) for i in range(6)]
        
        # Update final forces with averaged values
        for i in range(6):
            if not (i == 5 and self.finger6_fixed) and final_angles[i] is not None:
                final_forces[i] = avg_forces[i]
        
        rospy.loginfo("=== Grip Complete ===")
        for i in range(6):
            if i == 5 and self.finger6_fixed:
                rospy.loginfo(f"F6 (Thumb Abd): FIXED at {self.finger6_angle}")
            else:
                rospy.loginfo(f"F{i+1}: Angle={final_angles[i]}, Force={final_forces[i]}g")
        rospy.loginfo(f"Total time: {elapsed:.2f}s, Iterations: {iteration}")
        
        # Start slip detection after successful grip
        self.start_slip_detection()

    def detect_slip(self):
        """
        Monitor force fluctuations to detect object slipping.
        If 2+ fingers show 3+ fluctuations of ±10g within 2 seconds, open fingers.
        """
        rospy.loginfo("Slip detection: ACTIVE")
        
        # History buffer: stores last N force readings per finger
        history_size = 20  # 20 readings * 0.1s = 2 seconds
        force_history = [[] for _ in range(6)]
        
        while not self.stop_slip_detection and not rospy.is_shutdown():
            # Read current forces
            _, current_forces = self.get_sensor_data()
            
            # Update history for each finger
            for i in range(6):
                if i == 5 and self.finger6_fixed:
                    continue
                    
                force_history[i].append(current_forces[i])
                
                # Keep only last 2 seconds of data
                if len(force_history[i]) > history_size:
                    force_history[i].pop(0)
            
            # Check for fluctuations (need at least 2 seconds of data)
            if all(len(h) >= history_size for h in force_history[:5]):
                fingers_slipping = 0
                
                for i in range(5):  # Check first 5 fingers (not thumb abduction)
                    if len(force_history[i]) < 4:
                        continue
                    
                    # Count fluctuations: force changes of ±10g or more
                    fluctuation_count = 0
                    for j in range(1, len(force_history[i])):
                        change = abs(force_history[i][j] - force_history[i][j-1])
                        if change >= 10:
                            fluctuation_count += 1
                    
                    # If this finger has 3+ fluctuations, it's slipping
                    if fluctuation_count >= 3:
                        fingers_slipping += 1
                        rospy.logwarn(f"Slip detected on F{i+1}: {fluctuation_count} fluctuations")
                
                # If 2+ fingers are slipping, open and wait for next command
                if fingers_slipping >= 2:
                    rospy.logwarn(f"SLIP DETECTED on {fingers_slipping} fingers! Opening fingers...")
                    
                    # Stop slip detection before opening
                    self.stop_slip_detection = True
                    
                    # Open fingers
                    self.open_all_fingers()
                    
                    rospy.loginfo("Fingers opened due to slip. Waiting for next command (open/close)...")
                    
                    # Exit slip detection loop - wait for user command
                    return
            
            time.sleep(0.1)  # Sample rate: 10 Hz
        
        rospy.loginfo("Slip detection: STOPPED")
    
    def start_slip_detection(self):
        """Start slip detection in a separate thread."""
        if not self.slip_detection_active:
            self.stop_slip_detection = False
            self.slip_detection_active = True
            self.slip_detection_thread = threading.Thread(target=self.detect_slip, daemon=True)
            self.slip_detection_thread.start()
    
    def stop_slip_detection_func(self):
        """Stop slip detection thread."""
        if self.slip_detection_active:
            self.stop_slip_detection = True
            if self.slip_detection_thread:
                self.slip_detection_thread.join(timeout=2.0)
            self.slip_detection_active = False

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        
        if cmd == "open":
            # Stop slip detection if active
            self.stop_slip_detection_func()
            self.open_all_fingers()
        elif cmd == "close":
            # Stop any existing slip detection before new grip
            self.stop_slip_detection_func()
            self.close_with_force_control()
        else:
            rospy.logwarn(f"Unknown command: '{cmd}'")

def main():
    try:
        node = PickObjectNode()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

##rostopic pub --once /finger_command std_msgs/String "close"
##rostopic pub --once /finger_command std_msgs/String "open"