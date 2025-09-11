#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from inspire_sdkpy import inspire_sdk, inspire_hand_defaut
import time

class InspireHandNode(Node):
    def __init__(self):
        super().__init__('headless_driver_node')
        
        # Create publishers
        self.angle_pub = self.create_publisher(Float32MultiArray, 'inspire_hand/angle', 10)
        self.force_pub = self.create_publisher(Float32MultiArray, 'inspire_hand/force', 10)
        self.status_pub = self.create_publisher(Float32MultiArray, 'inspire_hand/status', 10)
        
        # Define the structure for the hand data
        self.states_structure = [
            ('angle_act', 1546, 6, 'short'),
            ('force_act', 1582, 6, 'short'),
            ('status', 1612, 3, 'byte'),
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
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('Inspire Hand Node started')
        
        # Statistics
        self.call_count = 0
        self.start_time = time.perf_counter()

    def timer_callback(self):
        try:
            data_dict = self.handler.read()
            self.call_count += 1

            if data_dict is None:
                raise Exception("Failed to read data from hand")

            # Publish angle data
            if 'angle_act' in data_dict:
                angle_msg = Float32MultiArray()
                # Convert values to float32
                angle_msg.data = [float(x) for x in data_dict['angle_act']]
                self.angle_pub.publish(angle_msg)

            # Publish force data
            if 'force_act' in data_dict:
                force_msg = Float32MultiArray()
                # Convert values to float32
                force_msg.data = [float(x) for x in data_dict['force_act']]
                self.force_pub.publish(force_msg)

            # Publish status data
            if 'status' in data_dict:
                status_msg = Float32MultiArray()
                # Convert values to float32
                status_msg.data = [float(x) for x in data_dict['status']]
                self.status_pub.publish(status_msg)

            # Log statistics every second
            if self.call_count % 100 == 0:  # Every 100 calls (approximately 1 second)
                elapsed_time = time.perf_counter() - self.start_time
                frequency = self.call_count / elapsed_time
                self.get_logger().info(
                    f'Current frequency: {frequency:.2f} Hz, '
                    f'Calls: {self.call_count}, '
                    f'Time: {elapsed_time:.2f} s'
                )

        except Exception as e:
            self.get_logger().error(f'Error reading hand data: {str(e)}')
            if 'data_dict' in locals():
                self.get_logger().error(f'Available data keys: {data_dict.keys() if data_dict else "None"}')
                self.get_logger().error(f'Data content: {data_dict}')

def main(args=None):
    rclpy.init(args=args)
    node = InspireHandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        elapsed_time = time.perf_counter() - node.start_time
        frequency = node.call_count / elapsed_time if elapsed_time > 0 else 0
        node.get_logger().info(
            f'Node shutting down. '
            f'Total calls: {node.call_count}, '
            f'Total time: {elapsed_time:.2f} s, '
            f'Final frequency: {frequency:.2f} Hz'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
