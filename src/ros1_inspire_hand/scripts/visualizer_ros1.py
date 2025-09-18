#!/usr/bin/env python3

import rospy
import time
import threading
import sys
from inspire_sdkpy import inspire_hand_defaut, inspire_dds, qt_tabs
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
import numpy as np
from PyQt5.QtCore import QTimer


class DDSHandler:
   
    def __init__(self, network=None, sub_touch=True, LR='r'):
        rospy.init_node('inspire_hand_visualizer_node', anonymous=True)
        
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
        
        rospy.loginfo(f"DDSHandler initialized for {LR} hand")

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
            rospy.logdebug(f"Touch data received")

    def update_data_state(self, states_msg: inspire_dds.inspire_hand_state):
        with self.data_state_lock:
            self.states = {
                'ANGLE_ACT': states_msg.angle_act,
                'FORCE_ACT': states_msg.force_act,
                'CURRENT': states_msg.current,
                'ERROR': states_msg.err,
                'STATUS': states_msg.status,
            }
            self.has_received_data = True
            rospy.logdebug(f"State data received")

    def read(self):
        # Return default values if no data has been received yet
        if not self.has_received_data:
            return {
                'states': {
                    'ANGLE_ACT': [0, 0, 0, 0, 0, 0],
                    'FORCE_ACT': [0, 0, 0, 0, 0, 0],
                    'CURRENT': [0, 0, 0, 0, 0, 0],
                    'ERROR': [0, 0, 0, 0, 0, 0],
                    'STATUS': [0, 0, 0, 0, 0, 0],
                },
                'touch': {}
            }
        
        with self.data_state_lock:
            states_copy = self.states.copy()
        
        with self.data_touch_lock:
            touch_copy = self.touch.copy()
        
        return {'states': states_copy, 'touch': touch_copy}

def main(args=None):
    rospy.init_node('inspire_hand_visualizer_node', anonymous=True)
    
    # Create the DDSHandler node
    dds_handler = DDSHandler(sub_touch=False, LR='r')
    
    rospy.loginfo("Starting Qt GUI...")
        
    # Create Qt application
    app = qt_tabs.QApplication(sys.argv)
    window = qt_tabs.MainWindow(data_handler=dds_handler, dt=80, name="Right DDS Subscribe", Plot_touch=False)
    window.reflash()
    rospy.loginfo("Qt window created and reflash called")
    window.show()
        
    # Timer to process ROS events within Qt event loop
    timer = QTimer()
    timer.timeout.connect(lambda: rospy.spin_once())  # ROS1 spin once in the Qt loop
    timer.start(10)  # Process ROS events every 10ms
        
    # Start Qt event loop
    app_exec_result = app.exec_()
        
    # Cleanup
    timer.stop()
    rospy.loginfo("Shutting down...")
    rospy.signal_shutdown("Shutting down DDSHandler")
    sys.exit(app_exec_result)

if __name__ == "__main__":
    main()
