#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from inspire_sdkpy import inspire_sdk
import time

class InspireHandNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('headless_driver_node')

        # Create publishers
        self.angle_pub = rospy.Publisher('inspire_hand/angle', Float32MultiArray, queue_size=10)
        self.force_pub = rospy.Publisher('inspire_hand/force', Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher('inspire_hand/status', Float32MultiArray, queue_size=10)
        self.current_pub = rospy.Publisher('inspire_hand/current', Float32MultiArray, queue_size=10)  # New publisher
        self.speed_pub = rospy.Publisher('inspire_hand/speed', Float32MultiArray, queue_size=10)    # New publisher

        # Define the structure for the hand data
        self.states_structure = [
            ('angle_act', 1546, 6, 'short'),
            ('force_act', 1582, 6, 'short'),
            ('status', 1612, 3, 'byte'),
            ('current', 1594, 6, 'short')  # New state for current
        ]
    
        # Initialize the hand interface
        self.handler = inspire_sdk.ModbusDataHandler(
            LR='r',
            device_id=1,
            use_serial=True,
            serial_port='/dev/ttyUSB0',
            states_structure=self.states_structure
        )
        
        # Create a timer for data publishing (100Hz)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        rospy.loginfo('Inspire Hand Node started')
        
        # Statistics
        self.call_count = 0
        self.start_time = time.perf_counter()

    def timer_callback(self, event):
        try:
            data_dict = self.handler.read()
            self.call_count += 1

            if data_dict is None:
                raise Exception("Failed to read data from hand")

            # Publish angle data
            if 'angle_act' in data_dict:
                angle_msg = Float32MultiArray()
                angle_msg.data = [float(x) for x in data_dict['angle_act']]
                self.angle_pub.publish(angle_msg)

            # Publish force data
            if 'force_act' in data_dict:
                force_msg = Float32MultiArray()
                force_msg.data = [float(x) for x in data_dict['force_act']]
                self.force_pub.publish(force_msg)

            # Publish status data
            if 'status' in data_dict:
                status_msg = Float32MultiArray()
                status_msg.data = [float(x) for x in data_dict['status']]
                self.status_pub.publish(status_msg)

            # Publish current data (new)
            if 'current' in data_dict:
                current_msg = Float32MultiArray()
                current_msg.data = [float(x) for x in data_dict['current']]
                self.current_pub.publish(current_msg)

            # Log statistics every second
            if self.call_count % 100 == 0:  # Every 100 calls (approximately 1 second)
                elapsed_time = time.perf_counter() - self.start_time
                frequency = self.call_count / elapsed_time
                rospy.loginfo(
                    f'Current frequency: {frequency:.2f} Hz, '
                    f'Calls: {self.call_count/100}, '
                    f'Elapsed Time: {elapsed_time:.2f} s'
                )

        except Exception as e:
            rospy.logerr(f'Error reading hand data: {str(e)}')
            if 'data_dict' in locals():
                rospy.logerr(f'Available data keys: {data_dict.keys() if data_dict else "None"}')
                rospy.logerr(f'Data content: {data_dict}')

def main():
    node = InspireHandNode()
    
    try:
        rospy.spin()  # Start the ROS event loop
    except KeyboardInterrupt:
        elapsed_time = time.perf_counter() - node.start_time
        frequency = node.call_count / elapsed_time if elapsed_time > 0 else 0
        rospy.loginfo(
            f'Node shutting down. '
            f'Total calls: {node.call_count/100}, '
            f'Total elapsed time: {elapsed_time:.2f} s, '
            f'Final frequency: {frequency:.2f} Hz'
        )
    finally:
        rospy.loginfo("Shutting down ROS node.")

if __name__ == '__main__':
    main()
