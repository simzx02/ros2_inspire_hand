from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds
import numpy as np
import time
import threading
import sys
from inspire_sdkpy import qt_tabs
from PyQt5.QtCore import QTimer

class DDSHandler():
   
    def __init__(self, network=None, sub_touch=True, LR='r'):
        super().__init__()
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
            print(f"Touch data received: {self.touch}")

    def update_data_state(self, states_msg: inspire_dds.inspire_hand_state):
        with self.data_state_lock:
            self.states = {
                'POS_ACT': states_msg.pos_act,
                'ANGLE_ACT': states_msg.angle_act,
                'FORCE_ACT': states_msg.force_act,
                'CURRENT': states_msg.current,
                'ERROR': states_msg.err,
                'STATUS': states_msg.status,
                'TEMP': states_msg.temperature
            }
            self.has_received_data = True
            print(f"State data received: {self.states}")

    def read(self):
        # Return default values if no data has been received yet
        if not self.has_received_data:
            return {
                'states': {
                    'POS_ACT': [0, 0, 0, 0, 0, 0],
                    'ANGLE_ACT': [0, 0, 0, 0, 0, 0],
                    'FORCE_ACT': [0, 0, 0, 0, 0, 0],
                    'CURRENT': [0, 0, 0, 0, 0, 0],
                    'ERROR': [0, 0, 0, 0, 0, 0],
                    'STATUS': [0, 0, 0, 0, 0, 0],
                    'TEMP': [0, 0, 0, 0, 0, 0]
                },
                'touch': {}
            }
        
        with self.data_state_lock:
            states_copy = self.states.copy()
        
        with self.data_touch_lock:
            touch_copy = self.touch.copy()
        
        return {'states': states_copy, 'touch': touch_copy}

if __name__ == "__main__":
    ddsHandler = DDSHandler(sub_touch=True, LR='r')
    app = qt_tabs.QApplication(sys.argv)
    window = qt_tabs.MainWindow(data_handler=ddsHandler, dt=80, name="Right DDS Subscribe", Plot_touch=True)
    window.reflash()  # Call reflash directly like before
    print("Window created and reflash called")
    window.show()
    sys.exit(app.exec_())