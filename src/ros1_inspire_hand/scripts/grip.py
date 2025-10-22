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
        
        # ROS publisher for calibrated forces
        self.calibrated_force_pub = rospy.Publisher('/inspire_hand/force_calibrated', Float32MultiArray, queue_size=10)

        # Prepare command
        self.cmd = inspire_hand_defaut.get_inspire_hand_ctrl()

        # Control parameters
        self.force_limits = [300] * 6  # Force limit in grams
        self.min_angle = 0  # Fully closed
        self.max_angle = 850  # Fully open
        self.open_speed = 350
        self.close_speed = 250
        self.step_size = 5  # Angle step per iteration
        
        # Fixed finger 6 (thumb abduction)
        self.finger6_fixed = True
        self.finger6_angle = 850

        # Thread-safe storage for sensor data
        self.data_lock = threading.Lock()
        self.current_angles = [850] * 6
        self.current_forces = [0] * 6
        self.force_offsets = [0] * 6  # Calibration offsets
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
        
        # Calibrate force sensors in open position
        rospy.loginfo("Calibrating force sensors...")
        time.sleep(1.0)  # Wait for settling
        self.calibrate_forces()
        
        rospy.loginfo("Initialization complete. Ready for commands.")

        rospy.spin()

    def angle_callback(self, msg):
        with self.data_lock:
            self.current_angles = [int(x) for x in msg.data]
            self.data_received = True

    def force_callback(self, msg):
        with self.data_lock:
            raw_forces = [int(x) for x in msg.data]
            # Apply calibration offset
            self.current_forces = [raw_forces[i] - self.force_offsets[i] for i in range(6)]
        
        # Publish calibrated forces for GUI (outside lock to avoid deadlock)
        try:
            calibrated_msg = Float32MultiArray()
            calibrated_msg.data = [float(f) for f in self.current_forces]
            self.calibrated_force_pub.publish(calibrated_msg)
        except:
            pass  # Ignore publishing errors during shutdown

    def calibrate_forces(self):
        """Calibrate force sensors to remove zero-offset (baseline readings)."""
        samples = []
        for _ in range(10):
            _, forces = self.get_sensor_data()
            samples.append(forces)
            time.sleep(0.05)
        
        # Calculate average offset for each finger
        for i in range(6):
            avg_offset = sum(sample[i] for sample in samples) / len(samples)
            self.force_offsets[i] = int(avg_offset)

    def get_sensor_data(self):
        with self.data_lock:
            return self.current_angles.copy(), self.current_forces.copy()

    def calibrate_forces(self):
        """
        Calibrate force sensors to zero in open position.
        Samples raw force readings and stores offsets.
        """
        rospy.loginfo("Sampling force sensors (20 samples over 1 second)...")
        raw_samples = []
        
        # Take 20 samples
        for _ in range(20):
            try:
                msg = rospy.wait_for_message("/inspire_hand/force", Float32MultiArray, timeout=1.0)
                raw_forces = [int(x) for x in msg.data]
                raw_samples.append(raw_forces)
                time.sleep(0.05)
            except:
                rospy.logwarn("Timeout reading force during calibration")
                continue
        
        if len(raw_samples) < 10:
            rospy.logwarn("Insufficient samples, using zero offsets")
            self.force_offsets = [0] * 6
            return
        
        # Calculate average offset for each finger
        for i in range(6):
            avg_offset = sum(sample[i] for sample in raw_samples) / len(raw_samples)
            self.force_offsets[i] = int(round(avg_offset))
        
        rospy.loginfo(f"✓ Calibration complete!")
        rospy.loginfo(f"  Offsets: F1={self.force_offsets[0]:+3d}g, F2={self.force_offsets[1]:+3d}g, "
                     f"F3={self.force_offsets[2]:+3d}g, F4={self.force_offsets[3]:+3d}g, "
                     f"F5={self.force_offsets[4]:+3d}g, F6={self.force_offsets[5]:+3d}g")
        rospy.loginfo("  All forces now read 0g in open position")

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
        
        # Improved PID parameters for smoother motion
        Kp = 2  # Reduced for gentler response
        Ki = 0.05  # Reduced to prevent integral windup
        Kd = 0.5  # Increased for better damping
        
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
        
        # Smoothing: Track velocity to avoid jerky movements
        velocity = [0.0] * 6
        max_velocity_change = 30  # Limit acceleration
        
        while not all(reached) and iteration < max_iterations:
            iteration += 1
            
            # Update command angles with PID for unreached fingers
            for i in range(6):
                if not reached[i]:
                    # Calculate error (how much more force we want)
                    force_error = self.force_limits[i] - forces[i]
                    
                    # Update PID terms
                    integral[i] += force_error * 0.1  # Reduced integration time
                    integral[i] = max(-300, min(300, integral[i]))  # Tighter anti-windup
                    
                    derivative = (force_error - previous_error[i]) / 0.1
                    previous_error[i] = force_error
                    
                    # Calculate dynamic step size using PID
                    pid_output = Kp * force_error + Ki * integral[i] + Kd * derivative
                    
                    # Target velocity based on PID
                    target_velocity = max(3, min(pid_output, 60))  # Clamp between 3-60 (smoother range)
                    
                    # Smooth velocity changes (limit acceleration)
                    velocity_change = target_velocity - velocity[i]
                    if abs(velocity_change) > max_velocity_change:
                        velocity_change = max_velocity_change if velocity_change > 0 else -max_velocity_change
                    
                    velocity[i] += velocity_change
                    velocity[i] = max(3, min(velocity[i], 60))  # Keep within bounds
                    
                    # Apply smoothed step
                    dynamic_step = int(velocity[i])
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
            
            # Shorter wait for more responsive control
            time.sleep(0.1)
            
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
                        if movement < 2:  # Less than 2 degrees movement (tighter tolerance)
                            stabilization_counter[i] += 1
                            if stabilization_counter[i] >= 7:  # Stuck for longer to avoid false positives
                                reached[i] = True
                                final_angles[i] = actual_angles[i]
                                final_forces[i] = actual_forces[i]
                                rospy.loginfo(f"F{i+1}: Stopped (stuck at {actual_angles[i]}°, Force={actual_forces[i]}g)")
                        else:
                            stabilization_counter[i] = 0
            
            # Save current state for next iteration
            self._prev_angles = actual_angles.copy()
            
            # Log progress every second
            if iteration % 10 == 0:
                rospy.loginfo(f"Iter {iteration}: Cmd={[int(x) for x in command_angles[:5]]}, "
                             f"Actual={actual_angles[:5]}, Forces={actual_forces[:5]}, "
                             f"Vel={[int(v) for v in velocity[:5]]}")
        
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
                        if change >= 20:
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