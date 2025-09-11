'''Same as inspire_hand_visualizer, but this one shorter script.'''
import sys
from inspire_sdkpy import qt_tabs,inspire_sdk,inspire_hand_defaut
# import inspire_sdkpy
if __name__ == "__main__":
    app = qt_tabs.QApplication(sys.argv)
     ## publish All Data
    # states_structure = [
    #         ('pos_act', 1534, 6, 'short'),
    #         ('angle_act', 1546, 6, 'short'),
    #         ('force_act', 1582, 6, 'short'),
    #         ('current', 1594, 6, 'short'),
    #         ('err', 1606, 3, 'byte'),
    #         ('status', 1612, 3, 'byte'),
    #         ('temperature', 1618, 3, 'byte')
    #     ]
    
    ## Only publish this data to increase publishing frequency
    states_structure = [
             ('pos_act', 1534, 6, 'short'),
             ('angle_act', 1546, 6, 'short'),
             ('force_act', 1582, 6, 'short'),
             ('current', 1594, 6, 'short'),
             ('err', 1606, 3, 'byte'),
             ('status', 1612, 3, 'byte'),
             ('temperature', 1618, 3, 'byte')
        ]
    
    handler = inspire_sdk.ModbusDataHandler(LR='r', device_id=1, use_serial=True, serial_port='/dev/ttyUSB0',states_structure=states_structure)
    window = qt_tabs.MainWindow(data_handler=handler,dt=100,name="Right Hand Vision Driver",Plot_touch=False,run_time=False)
    window.reflash()
    window.show()
    sys.exit(app.exec_())