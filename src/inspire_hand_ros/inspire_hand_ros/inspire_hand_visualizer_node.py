##this code is modified from dds_subscribe_485_r.py to build a gui.
##not directly related to dds_subscribe_485_r.py while calling this node.
##but still keep it for reference. So do not delete dds_subscribe_485_r.py

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds
import numpy as np
import time
import threading
import sys
from inspire_sdkpy import qt_tabs
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

class DDSHandler(Node):
   
    def __init__(self, network=None, sub_touch=True, LR='r'):
        super().__init__('inspire_hand_visualizer_node')
        
        if network is None:
            ChannelFactoryInitialize(0)
        else:
            ChannelFactoryInitialize(0, network)
        
        self.data = inspire_hand_defaut.data_sheet
        self.touch = {}
        self.states = {}
        self.data_touch_lock = threading.Lock()
        self.data_state_lock = threading.Lock()
        self.has_received_data = False  # Track if we've received any data
        
        if sub_touch:
            self.sub_touch = ChannelSubscriber("rt/inspire_hand/touch/"+LR, inspire_dds.inspire_hand_touch)
            self.sub_touch.Init(self.update_data_touch, 10)
        
        self.sub_states = ChannelSubscriber("rt/inspire_hand/state/"+LR, inspire_dds.inspire_hand_state)
        self.sub_states.Init(self.update_data_state, 10)
        
        self.get_logger().info(f"DDSHandler initialized for {LR} hand")

    def update_data_touch(self, msg: inspire_dds.inspire_hand_touch):
        with self.data_touch_lock:
            start_time = time.time()
            for i, (name, addr, length, size, var) in enumerate(self.data):
                value = getattr(msg, var)
                if value is not None:
                    matrix = np.array(value).reshape(size)
                    self.touch[var] = matrix
            self.has_received_data = True
            end_time = time.time()
            elapsed_time = end_time - start_time
            self.get_logger().debug(f"Touch data received")

    def update_data_state(self, states_msg: inspire_dds.inspire_hand_state):
        with self.data_state_lock:
            self.states = {
                #'POS_ACT': states_msg.pos_act,
                'ANGLE_ACT': states_msg.angle_act,
                'FORCE_ACT': states_msg.force_act,
                'CURRENT': states_msg.current,
                'ERROR': states_msg.err,
                'STATUS': states_msg.status,
            #    'TEMP': states_msg.temperature
            }
            self.has_received_data = True
            self.get_logger().debug(f"State data received")

    def read(self):
        # Return default values if no data has been received yet
        if not self.has_received_data:
            return {
                'states': {
                    #'POS_ACT': [0, 0, 0, 0, 0, 0],
                    'ANGLE_ACT': [0, 0, 0, 0, 0, 0],
                    'FORCE_ACT': [0, 0, 0, 0, 0, 0],
                    'CURRENT': [0, 0, 0, 0, 0, 0],
                    'ERROR': [0, 0, 0, 0, 0, 0],
                    'STATUS': [0, 0, 0, 0, 0, 0],
                #    'TEMP': [0, 0, 0, 0, 0, 0]
                },
                'touch': {}
            }
        
        with self.data_state_lock:
            states_copy = self.states.copy()
        
        with self.data_touch_lock:
            touch_copy = self.touch.copy()
        
        return {'states': states_copy, 'touch': touch_copy}

def main(args=None):
    rclpy.init(args=args)
    
    # Create the DDSHandler node
    dds_handler = DDSHandler(sub_touch=False, LR='r')
    
    dds_handler.get_logger().info("Starting Qt GUI...")
        
    # Create executor for ROS2
    executor = SingleThreadedExecutor()
    executor.add_node(dds_handler)
        
    # Create Qt application
    app = qt_tabs.QApplication(sys.argv)
    window = qt_tabs.MainWindow(data_handler=dds_handler, dt=80, name="Right DDS Subscribe", Plot_touch=False)
    window.reflash()
    dds_handler.get_logger().info("Qt window created and reflash called")
    window.show()
        
    # Timer to process ROS2 events within Qt event loop
    timer = QTimer()
    timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0))
    timer.start(10)  # Process ROS2 events every 10ms
        
    # Start Qt event loop
    app_exec_result = app.exec_()
        
    # Cleanup
    timer.stop()
    executor.shutdown()
    dds_handler.destroy_node()
    rclpy.shutdown()
        
    sys.exit(app_exec_result)

if __name__ == "__main__":
    main()