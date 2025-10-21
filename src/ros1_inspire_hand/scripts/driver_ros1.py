#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from inspire_sdkpy import inspire_sdk
import time

class InspireHandNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('headless_driver_node')

        # Create publishers - matching the actual data structure
        self.angle_pub = rospy.Publisher('inspire_hand/angle', Float32MultiArray, queue_size=10)
        self.force_pub = rospy.Publisher('inspire_hand/force', Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher('inspire_hand/status', Float32MultiArray, queue_size=10)
        self.current_pub = rospy.Publisher('inspire_hand/current', Float32MultiArray, queue_size=10)
        self.pos_pub = rospy.Publisher('inspire_hand/position', Float32MultiArray, queue_size=10)
        self.temp_pub = rospy.Publisher('inspire_hand/temperature', Float32MultiArray, queue_size=10)
        self.error_pub = rospy.Publisher('inspire_hand/error', Float32MultiArray, queue_size=10)

        # Define the structure for the hand data
        self.states_structure = [
            ('angle_act', 1546, 6, 'short'),
            ('force_act', 1582, 6, 'short'),
            ('status', 1612, 3, 'byte'),
            ('current', 1594, 6, 'short')
        ]
    
        # Initialize the hand interface
        self.handler = inspire_sdk.ModbusDataHandler(
            LR='r',
            device_id=1,
            use_serial=True,
            serial_port='/dev/inspirehand_serial_usb',
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
                rospy.logwarn_throttle(5.0, "handler.read() returned None")
                return

            # Check if 'states' key exists
            if 'states' not in data_dict:
                rospy.logwarn_throttle(5.0, "No 'states' key in data_dict")
                return

            states = data_dict['states']
            published_count = 0
            
            # Publish angle data (ANGLE_ACT key, uppercase)
            if 'ANGLE_ACT' in states:
                angle_msg = Float32MultiArray()
                angle_msg.data = [float(x) for x in states['ANGLE_ACT']]
                self.angle_pub.publish(angle_msg)
                published_count += 1
            
            # Publish force data (FORCE_ACT key, uppercase)
            if 'FORCE_ACT' in states:
                force_msg = Float32MultiArray()
                force_msg.data = [float(x) for x in states['FORCE_ACT']]
                self.force_pub.publish(force_msg)
                published_count += 1
            
            # Publish status data (STATUS key, uppercase)
            if 'STATUS' in states:
                status_msg = Float32MultiArray()
                status_msg.data = [float(x) for x in states['STATUS']]
                self.status_pub.publish(status_msg)
                published_count += 1
            
            # Publish current data (CURRENT key, uppercase)
            if 'CURRENT' in states:
                current_msg = Float32MultiArray()
                current_msg.data = [float(x) for x in states['CURRENT']]
                self.current_pub.publish(current_msg)
                published_count += 1
            
            # Publish position data (POS_ACT key)
            if 'POS_ACT' in states:
                pos_msg = Float32MultiArray()
                pos_msg.data = [float(x) for x in states['POS_ACT']]
                self.pos_pub.publish(pos_msg)
                published_count += 1
            
            # Publish temperature data (TEMP key)
            if 'TEMP' in states:
                temp_msg = Float32MultiArray()
                temp_msg.data = [float(x) for x in states['TEMP']]
                self.temp_pub.publish(temp_msg)
                published_count += 1
            
            # Publish error data (ERROR key)
            if 'ERROR' in states:
                error_msg = Float32MultiArray()
                error_msg.data = [float(x) for x in states['ERROR']]
                self.error_pub.publish(error_msg)
                published_count += 1

            # Log statistics every second
            if self.call_count % 100 == 0:
                elapsed_time = time.perf_counter() - self.start_time
                frequency = self.call_count / elapsed_time
                rospy.loginfo(
                    f'Frequency: {frequency:.2f} Hz | '
                    f'Calls: {self.call_count} | '
                    f'Published: {published_count} topics | '
                )

        except Exception as e:
            rospy.logerr(f'Error in timer callback: {str(e)}')
            import traceback
            rospy.logerr(traceback.format_exc())

def main():
    node = InspireHandNode()
    
    try:
        rospy.spin()  # Start the ROS event loop
    except KeyboardInterrupt:
        elapsed_time = time.perf_counter() - node.start_time
        frequency = node.call_count / elapsed_time if elapsed_time > 0 else 0
        rospy.loginfo(
            f'Node shutting down. '
            f'Total calls: {node.call_count}, '
            f'Total time: {elapsed_time:.2f}s, '
            f'Final frequency: {frequency:.2f} Hz'
        )
    finally:
        rospy.loginfo("Shutting down ROS node.")

if __name__ == '__main__':
    main()