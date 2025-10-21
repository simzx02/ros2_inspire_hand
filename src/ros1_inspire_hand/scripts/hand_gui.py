#!/usr/bin/env python3

import sys
import subprocess
import signal
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QTextEdit, 
                             QGroupBox, QGridLayout, QProgressBar, QTabWidget,
                             QTableWidget, QTableWidgetItem, QHeaderView)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPalette, QColor
import rospy
from std_msgs.msg import String, Float32MultiArray
import os

class ROSNodeManager(QThread):
    """Manages ROS node processes in background thread"""
    log_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.processes = {}
        self.running = False
        
    def start_nodes(self):
        """Start all ROS nodes"""
        try:
            # Source ROS environment
            ros_setup = "source /opt/ros/noetic/setup.bash && source ~/Desktop/inspire_hand/ros1_inspire_hand/devel/setup.bash"
            
            # Start roscore
            self.log_signal.emit("Starting roscore...")
            self.processes['roscore'] = subprocess.Popen(
                f"{ros_setup} && roscore",
                shell=True,
                executable='/bin/bash',
                preexec_fn=os.setsid
            )
            time.sleep(3)
            
            # Start driver
            self.log_signal.emit("Starting driver node...")
            self.processes['driver'] = subprocess.Popen(
                f"{ros_setup} && rosrun ros1_inspire_hand driver_ros1.py",
                shell=True,
                executable='/bin/bash',
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            
            # Start grip controller
            self.log_signal.emit("Starting grip controller...")
            self.processes['grip'] = subprocess.Popen(
                f"{ros_setup} && rosrun ros1_inspire_hand grip.py",
                shell=True,
                executable='/bin/bash',
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            
            self.log_signal.emit("✓ All nodes started successfully!")
            self.running = True
            
        except Exception as e:
            self.log_signal.emit(f"✗ Error starting nodes: {str(e)}")
            
    def stop_nodes(self):
        """Stop all ROS nodes"""
        self.log_signal.emit("Stopping all nodes...")
        for name, proc in self.processes.items():
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                self.log_signal.emit(f"✓ Stopped {name}")
            except:
                pass
        self.processes.clear()
        self.running = False
        self.log_signal.emit("All nodes stopped")

class InspireHandGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inspire Hand Control Center")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize ROS (will be done after nodes start)
        self.ros_initialized = False
        self.node_manager = ROSNodeManager()
        self.node_manager.log_signal.connect(self.append_log)
        
        # Data storage
        self.current_angles = [0] * 6
        self.current_forces = [0] * 6
        self.slip_active = False
        
        # Setup UI
        self.setup_ui()
        
        # Setup timers
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.update_display)
        
    def setup_ui(self):
        """Setup the main UI"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Title
        title = QLabel("🤖 Inspire Hand Control Center")
        title_font = QFont("Arial", 20, QFont.Bold)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #2c3e50; padding: 10px;")
        main_layout.addWidget(title)
        
        # Control Panel
        control_group = self.create_control_panel()
        main_layout.addWidget(control_group)
        
        # Tabs for data display
        tabs = QTabWidget()
        tabs.addTab(self.create_sensor_tab(), "📊 Sensor Data")
        tabs.addTab(self.create_status_tab(), "📈 Status")
        tabs.addTab(self.create_log_tab(), "📝 System Log")
        main_layout.addWidget(tabs)
        
        # Apply stylesheet
        self.setStyleSheet("""
            QMainWindow {
                background-color: #ecf0f1;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #3498db;
                border-radius: 5px;
                margin-top: 10px;
                padding: 15px;
                background-color: white;
            }
            QGroupBox::title {
                color: #2c3e50;
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
                min-height: 40px;
            }
            QPushButton:hover {
                opacity: 0.8;
            }
        """)
        
    def create_control_panel(self):
        """Create main control buttons"""
        group = QGroupBox("Control Panel")
        layout = QHBoxLayout()
        
        # System Control
        system_layout = QVBoxLayout()
        
        self.start_btn = QPushButton("🚀 Start System")
        self.start_btn.setStyleSheet("background-color: #27ae60; color: white;")
        self.start_btn.clicked.connect(self.start_system)
        
        self.stop_btn = QPushButton("🛑 Stop System")
        self.stop_btn.setStyleSheet("background-color: #e74c3c; color: white;")
        self.stop_btn.clicked.connect(self.stop_system)
        self.stop_btn.setEnabled(False)
        
        system_layout.addWidget(self.start_btn)
        system_layout.addWidget(self.stop_btn)
        
        # Hand Control
        hand_layout = QVBoxLayout()
        
        self.open_btn = QPushButton("🖐️ OPEN")
        self.open_btn.setStyleSheet("background-color: #3498db; color: white; font-size: 16px;")
        self.open_btn.clicked.connect(self.open_hand)
        self.open_btn.setEnabled(False)
        
        self.close_btn = QPushButton("✊ CLOSE")
        self.close_btn.setStyleSheet("background-color: #9b59b6; color: white; font-size: 16px;")
        self.close_btn.clicked.connect(self.close_hand)
        self.close_btn.setEnabled(False)
        
        hand_layout.addWidget(self.open_btn)
        hand_layout.addWidget(self.close_btn)
        
        # Status indicators
        status_layout = QVBoxLayout()
        
        self.system_status = QLabel("⚪ System: Offline")
        self.system_status.setStyleSheet("font-size: 14px; padding: 5px;")
        
        self.slip_status = QLabel("⚪ Slip Detection: Inactive")
        self.slip_status.setStyleSheet("font-size: 14px; padding: 5px;")
        
        status_layout.addWidget(self.system_status)
        status_layout.addWidget(self.slip_status)
        status_layout.addStretch()
        
        layout.addLayout(system_layout, 1)
        layout.addLayout(hand_layout, 2)
        layout.addLayout(status_layout, 1)
        
        group.setLayout(layout)
        return group
        
    def create_sensor_tab(self):
        """Create sensor data display tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Angle display
        angle_group = QGroupBox("Finger Angles (degrees)")
        angle_layout = QGridLayout()
        
        self.angle_labels = []
        self.angle_bars = []
        for i in range(6):
            label = QLabel(f"F{i+1}:" if i < 5 else "Thumb:")
            value_label = QLabel("0°")
            value_label.setStyleSheet("font-weight: bold; font-size: 16px;")
            
            bar = QProgressBar()
            bar.setRange(0, 850)
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setFormat("%v°")  # Show actual value
            
            angle_layout.addWidget(label, i, 0)
            angle_layout.addWidget(bar, i, 1)
            angle_layout.addWidget(value_label, i, 2)
            
            self.angle_labels.append(value_label)
            self.angle_bars.append(bar)
            
        angle_group.setLayout(angle_layout)
        layout.addWidget(angle_group)
        
        # Force display
        force_group = QGroupBox("Finger Forces (grams)")
        force_layout = QGridLayout()
        
        self.force_labels = []
        self.force_bars = []
        for i in range(6):
            label = QLabel(f"F{i+1}:" if i < 5 else "Thumb:")
            value_label = QLabel("0g")
            value_label.setStyleSheet("font-weight: bold; font-size: 16px;")
            
            bar = QProgressBar()
            bar.setRange(0, 1000)
            bar.setValue(0)
            bar.setTextVisible(False)
            
            force_layout.addWidget(label, i, 0)
            force_layout.addWidget(bar, i, 1)
            force_layout.addWidget(value_label, i, 2)
            
            self.force_labels.append(value_label)
            self.force_bars.append(bar)
            
        force_group.setLayout(force_layout)
        layout.addWidget(force_group)
        
        return widget
        
    def create_status_tab(self):
        """Create status monitoring tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        self.status_table = QTableWidget()
        self.status_table.setColumnCount(2)
        self.status_table.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.status_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        status_items = [
            "ROS Nodes",
            "Driver Status",
            "Grip Controller",
            "Update Rate",
            "Slip Detection",
            "Last Command"
        ]
        
        self.status_table.setRowCount(len(status_items))
        for i, item in enumerate(status_items):
            self.status_table.setItem(i, 0, QTableWidgetItem(item))
            self.status_table.setItem(i, 1, QTableWidgetItem("N/A"))
            
        layout.addWidget(self.status_table)
        return widget
        
    def create_log_tab(self):
        """Create system log tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: #2c3e50; color: #ecf0f1; font-family: 'Courier New';")
        
        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self.log_text.clear)
        
        layout.addWidget(self.log_text)
        layout.addWidget(clear_btn)
        
        return widget
        
    def start_system(self):
        """Start all ROS nodes"""
        self.append_log("=== Starting System ===")
        self.start_btn.setEnabled(False)
        
        # Start nodes in background
        self.node_manager.start_nodes()
        
        # Wait a bit then initialize ROS subscriber
        QTimer.singleShot(5000, self.initialize_ros)
        
    def initialize_ros(self):
        """Initialize ROS node and subscribers"""
        try:
            if not self.ros_initialized:
                rospy.init_node('inspire_hand_gui', anonymous=True)
                self.ros_initialized = True
                
            # Subscribe to topics
            rospy.Subscriber("/inspire_hand/angle", Float32MultiArray, self.angle_callback)
            rospy.Subscriber("/inspire_hand/force", Float32MultiArray, self.force_callback)
            
            # Start data update timer
            self.data_timer.start(100)  # Update every 100ms
            
            # Enable controls
            self.stop_btn.setEnabled(True)
            self.open_btn.setEnabled(True)
            self.close_btn.setEnabled(True)
            
            self.system_status.setText("🟢 System: Online")
            self.system_status.setStyleSheet("font-size: 14px; padding: 5px; color: green;")
            
            self.append_log("✓ ROS subscribers initialized")
            self.update_status_table(0, 1, "Running")
            
        except Exception as e:
            self.append_log(f"✗ Error initializing ROS: {str(e)}")
            
    def stop_system(self):
        """Stop all ROS nodes"""
        self.data_timer.stop()
        self.node_manager.stop_nodes()
        
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.open_btn.setEnabled(False)
        self.close_btn.setEnabled(False)
        
        self.system_status.setText("⚪ System: Offline")
        self.system_status.setStyleSheet("font-size: 14px; padding: 5px;")
        
    def open_hand(self):
        """Send open command"""
        try:
            pub = rospy.Publisher('/finger_command', String, queue_size=1)
            time.sleep(0.1)
            pub.publish("open")
            self.append_log("→ Sent: OPEN command")
            self.update_status_table(5, 1, "OPEN")
        except Exception as e:
            self.append_log(f"✗ Error sending open: {str(e)}")
            
    def close_hand(self):
        """Send close command"""
        try:
            pub = rospy.Publisher('/finger_command', String, queue_size=1)
            time.sleep(0.1)
            pub.publish("close")
            self.append_log("→ Sent: CLOSE command")
            self.update_status_table(5, 1, "CLOSE")
            
            # Activate slip detection indicator
            QTimer.singleShot(3000, self.activate_slip_detection)
        except Exception as e:
            self.append_log(f"✗ Error sending close: {str(e)}")
            
    def activate_slip_detection(self):
        """Indicate slip detection is active"""
        self.slip_status.setText("🟢 Slip Detection: Active")
        self.slip_status.setStyleSheet("font-size: 14px; padding: 5px; color: green;")
        self.update_status_table(4, 1, "Active")
        
    def angle_callback(self, msg):
        """Handle angle data"""
        self.current_angles = list(msg.data)
        
    def force_callback(self, msg):
        """Handle force data"""
        self.current_forces = list(msg.data)
        
    def update_display(self):
        """Update all display elements"""
        # Update angles
        for i in range(6):
            angle = int(self.current_angles[i]) if i < len(self.current_angles) else 0
            # Clamp to valid range
            angle = max(0, min(850, angle))
            
            self.angle_labels[i].setText(f"{angle}°")
            self.angle_bars[i].setValue(angle)
            
            # Color code based on position
            if angle < 200:
                self.angle_bars[i].setStyleSheet("QProgressBar::chunk { background-color: #e74c3c; }")
            elif angle < 600:
                self.angle_bars[i].setStyleSheet("QProgressBar::chunk { background-color: #f39c12; }")
            else:
                self.angle_bars[i].setStyleSheet("QProgressBar::chunk { background-color: #27ae60; }")
                
        # Update forces with calibration already applied in grip.py
        for i in range(6):
            force = int(self.current_forces[i]) if i < len(self.current_forces) else 0
            # Force is already calibrated in grip.py, just ensure non-negative
            force = max(0, force)
            
            self.force_labels[i].setText(f"{force}g")
            self.force_bars[i].setValue(min(force, 1000))
            
            # Color code based on force
            if force > 300:
                self.force_bars[i].setStyleSheet("QProgressBar::chunk { background-color: #e74c3c; }")
            elif force > 100:
                self.force_bars[i].setStyleSheet("QProgressBar::chunk { background-color: #f39c12; }")
            else:
                self.force_bars[i].setStyleSheet("QProgressBar::chunk { background-color: #3498db; }")
                
    def update_status_table(self, row, col, value):
        """Update status table"""
        self.status_table.setItem(row, col, QTableWidgetItem(str(value)))
        
    def append_log(self, message):
        """Append message to log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        
    def closeEvent(self, event):
        """Handle window close"""
        self.stop_system()
        event.accept()

def main():
    app = QApplication(sys.argv)
    gui = InspireHandGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()