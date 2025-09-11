#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#The original code here: ~/workspace/inspire_hand/inspire_hand_ws/inspire_hand_sdk/example/dds_publish.py
#A copy of the original code is dds_publisher_485_r.py in this package.

import rclpy
from rclpy.node import Node
import time
import sys
import numpy as np
import threading

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds
from inspire_sdkpy import inspire_sdk

class DDSPublisherNode(Node):
    def __init__(self):
        super().__init__('dds_publisher_node')

        # Initialize Cyclone DDS with better error handling
        self.dds_available = False
        try:
            ChannelFactoryInitialize(1)
            self.get_logger().info("Cyclone DDS initialized.")
            self.dds_available = True
        except Exception as e:
            self.get_logger().warn(f"DDS initialization failed: {e}. Will continue without DDS.")

        # Publisher for right hand
        self.pub_r = None
        if self.dds_available:
            try:
                self.pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
                self.pub_r.Init()
                self.get_logger().info("DDS publisher initialized for right hand.")
            except Exception as e:
                self.get_logger().warn(f"Failed to create DDS publisher: {e}. Will continue without DDS.")

        # Default command
        self.cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
        self.short_value = 1000

        # Start the periodic publishing timer
        self.timer = self.create_timer(0.1, self.publish_dds_callback)

        # Counter
        self.cnd = 0

    def publish_dds_callback(self):
        """Publish commands periodically, mimicking original loop."""
        try:
            # Flip short_value every 10 iterations
            if (self.cnd + 1) % 10 == 0:
                self.short_value = 1000 - self.short_value

            # Generate values
            values_to_write = [self.short_value] * 6
            values_to_write[-1] = 1000 - values_to_write[-1]
            values_to_write[-2] = 1000 - values_to_write[-2]

            value_np = np.array(values_to_write)
            value_np = np.clip(value_np, 200, 800)

            # Fill DDS command
            self.cmd.angle_set = value_np.tolist()
            self.cmd.mode = 0b0001

            # Publish with detailed error handling
            if self.dds_available and self.pub_r:
                try:
                    res_r = self.pub_r.Write(self.cmd)
                    if self.cnd % 10 == 0:  # Log occasionally
                        self.get_logger().debug(f"DDS Write complete. Values={value_np.tolist()}")
                except Exception as e:
                    self.get_logger().error(f"DDS Write failed with: {str(e)}")
                    if "DDSException" in str(e):
                        self.get_logger().warn("DDS connection issue - check if subscriber is running")

            self.cnd += 1

        except Exception as e:
            self.get_logger().warn(f"Failed to publish DDS message: {e}")

def headless_driver_function():
    """Run the headless driver functionality in a separate thread."""
    states_structure = [
        ('angle_act', 1546, 6, 'short'),
        ('force_act', 1582, 6, 'short'),
        ('status', 1612, 3, 'byte'),
    ]
    
    handler = inspire_sdk.ModbusDataHandler(LR='r', device_id=1, use_serial=True, serial_port='/dev/ttyUSB0',states_structure=states_structure)

    call_count = 0
    start_time = time.perf_counter()

    try:
        while True:
            data_dict = handler.read()
            call_count += 1
            time.sleep(0.001)

            if call_count % 20 == 0:
                elapsed_time = time.perf_counter() - start_time
                frequency = call_count / elapsed_time
                print(f"当前频率: {frequency:.2f} Hz, 调用次数: {call_count}, 耗时: {elapsed_time:.6f} 秒")
    except KeyboardInterrupt:
        elapsed_time = time.perf_counter() - start_time
        frequency = call_count / elapsed_time if elapsed_time > 0 else 0
        print(f"程序结束. 总调用次数: {call_count}, 总耗时: {elapsed_time:.6f} 秒, 最终频率: {frequency:.2f} Hz")

def main(args=None):
    rclpy.init(args=args)
    
    # Start headless driver in separate thread
    headless_thread = threading.Thread(target=headless_driver_function, daemon=True)
    headless_thread.start()
    
    # Run DDS publisher node
    node = DDSPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()