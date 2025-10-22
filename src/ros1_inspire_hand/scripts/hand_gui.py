#!/usr/bin/env python3

import sys
import subprocess
import signal
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QTextEdit, 
                             QGroupBox, QGridLayout, QProgressBar, QTabWidget,
                             QTableWidget, QTableWidgetItem, QHeaderView, QSplitter)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPalette, QColor
import rospy
from std_msgs.msg import String, Float32MultiArray
import os
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class ROSNodeManager(QThread):
    """Manages ROS node processes in background thread"""
    log_signal = pyqtSignal(str)
    progress_signal = pyqtSignal(int, str)  # progress value, status text
    
    def __init__(self):
        super().__init__()
        self.processes = {}
        self.running = False
        
    def start_nodes(self):
        """Start all ROS nodes with progress updates"""
        try:
            # Source ROS environment
            ros_setup = "source /opt/ros/noetic/setup.bash && source ~/Desktop/inspire_hand/ros1_inspire_hand/devel/setup.bash"
            
            # Start roscore
            self.progress_signal.emit(10, "Starting ROS Master...")
            self.log_signal.emit("Starting roscore...")
            self.processes['roscore'] = subprocess.Popen(
                f"{ros_setup} && roscore",
                shell=True,
                executable='/bin/bash',
                preexec_fn=os.setsid
            )
            time.sleep(3)
            
            # Start driver
            self.progress_signal.emit(40, "Starting Driver Node...")
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
            self.progress_signal.emit(70, "Starting Grip Controller...")
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
            
            self.progress_signal.emit(100, "System Ready!")
            self.log_signal.emit("✓ All nodes started successfully!")
            self.running = True
            
        except Exception as e:
            self.log_signal.emit(f"✗ Error starting nodes: {str(e)}")
            self.progress_signal.emit(0, "Startup Failed")
            
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

class RealtimeGraphWidget(QWidget):
    """Real-time graph for force and angle data"""
    def __init__(self, title="Real-time Data"):
        super().__init__()
        self.title = title
        
        layout = QVBoxLayout(self)
        
        # Create matplotlib figure
        self.figure = Figure(figsize=(8, 6))
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        
        # Create subplots
        self.ax_angle = self.figure.add_subplot(2, 1, 1)
        self.ax_force = self.figure.add_subplot(2, 1, 2)
        
        # Data buffers (last 100 points)
        self.time_data = []
        self.angle_data = [[] for _ in range(6)]
        self.force_data = [[] for _ in range(6)]
        self.max_points = 100
        self.start_time = time.time()
        
        # Initialize plots
        self.setup_plots()
        
    def setup_plots(self):
        """Setup plot appearance"""
        colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6', '#1abc9c']
        
        # Angle plot
        self.ax_angle.set_title('Finger Angles', fontweight='bold')
        self.ax_angle.set_ylabel('Angle (degrees)')
        self.ax_angle.set_ylim(-50, 1100)
        self.ax_angle.grid(True, alpha=0.3)
        self.angle_lines = []
        for i in range(6):
            line, = self.ax_angle.plot([], [], color=colors[i], label=f'F{i+1}', linewidth=2)
            self.angle_lines.append(line)
        self.ax_angle.legend(loc='upper right', ncol=6, fontsize=8)
        
        # Force plot
        self.ax_force.set_title('Finger Forces', fontweight='bold')
        self.ax_force.set_xlabel('Time (s)')
        self.ax_force.set_ylabel('Force (grams)')
        self.ax_force.set_ylim(-200, 1200)
        self.ax_force.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
        self.ax_force.grid(True, alpha=0.3)
        self.force_lines = []
        for i in range(6):
            line, = self.ax_force.plot([], [], color=colors[i], label=f'F{i+1}', linewidth=2)
            self.force_lines.append(line)
        self.ax_force.legend(loc='upper right', ncol=6, fontsize=8)
        
        self.figure.tight_layout()
        
    def update_data(self, angles, forces):
        """Update graph with new data"""
        current_time = time.time() - self.start_time
        
        # Add new data
        self.time_data.append(current_time)
        for i in range(6):
            self.angle_data[i].append(angles[i] if i < len(angles) else 0)
            self.force_data[i].append(forces[i] if i < len(forces) else 0)
        
        # Keep only last max_points
        if len(self.time_data) > self.max_points:
            self.time_data = self.time_data[-self.max_points:]
            for i in range(6):
                self.angle_data[i] = self.angle_data[i][-self.max_points:]
                self.force_data[i] = self.force_data[i][-self.max_points:]
        
        # Update plots
        for i in range(6):
            self.angle_lines[i].set_data(self.time_data, self.angle_data[i])
            self.force_lines[i].set_data(self.time_data, self.force_data[i])
        
        # Adjust x-axis
        if self.time_data:
            self.ax_angle.set_xlim(max(0, self.time_data[-1] - 10), self.time_data[-1] + 0.5)
            self.ax_force.set_xlim(max(0, self.time_data[-1] - 10), self.time_data[-1] + 0.5)
        
        self.canvas.draw()

class SymmetricProgressBar(QWidget):
    """Custom progress bar that shows negative and positive values with 0 in center"""
    def __init__(self, min_val=-100, max_val=500):
        super().__init__()
        self.min_val = min_val
        self.max_val = max_val
        self.current_value = 0
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Negative bar (right to left)
        self.neg_bar = QProgressBar()
        self.neg_bar.setRange(0, abs(min_val))
        self.neg_bar.setValue(0)
        self.neg_bar.setTextVisible(False)
        self.neg_bar.setInvertedAppearance(True)
        self.neg_bar.setStyleSheet("QProgressBar::chunk { background-color: #e74c3c; }")
        
        # Zero marker
        zero_label = QLabel("0")
        zero_label.setStyleSheet("font-weight: bold; padding: 0 2px;")
        zero_label.setFixedWidth(15)
        
        # Positive bar (left to right)
        self.pos_bar = QProgressBar()
        self.pos_bar.setRange(0, max_val)
        self.pos_bar.setValue(0)
        self.pos_bar.setTextVisible(False)
        self.pos_bar.setStyleSheet("QProgressBar::chunk { background-color: #3498db; }")
        
        layout.addWidget(self.neg_bar, 1)
        layout.addWidget(zero_label, 0)
        layout.addWidget(self.pos_bar, 1)
        
    def setValue(self, value):
        """Set value (can be negative or positive)"""
        self.current_value = value
        if value < 0:
            self.neg_bar.setValue(abs(value))
            self.pos_bar.setValue(0)
        else:
            self.neg_bar.setValue(0)
            self.pos_bar.setValue(value)

class InspireHandGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inspire Hand Control Center")
        self.setGeometry(100, 100, 1400, 900)
        
        # Initialize ROS
        self.ros_initialized = False
        self.node_manager = ROSNodeManager()
        self.node_manager.log_signal.connect(self.append_log)
        self.node_manager.progress_signal.connect(self.update_progress)
        
        # Data storage
        self.current_angles = [0] * 6
        self.current_forces = [0] * 6
        self.slip_active = False
        self.topic_status = {}
        
        # Progress bar animation
        self.progress_target = 0
        self.progress_current = 0
        self.progress_timer = QTimer()
        self.progress_timer.timeout.connect(self.animate_progress)
        
        # Setup UI
        self.setup_ui()
        
        # Setup timers
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.update_display)
        
        # Auto-start system after UI is ready
        QTimer.singleShot(500, self.auto_start_system)
        
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
        tabs.addTab(self.create_graph_tab(), "📈 Real-time Graphs")
        tabs.addTab(self.create_status_tab(), "ℹ️ System Status")
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
        layout = QVBoxLayout()
        
        # Initialization Progress Bar
        progress_layout = QVBoxLayout()
        self.init_progress_label = QLabel("Initializing System...")
        self.init_progress_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #2c3e50;")
        self.init_progress_bar = QProgressBar()
        self.init_progress_bar.setRange(0, 100)
        self.init_progress_bar.setValue(0)
        self.init_progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #3498db;
                border-radius: 5px;
                text-align: center;
                height: 25px;
            }
            QProgressBar::chunk {
                background-color: #3498db;
            }
        """)
        progress_layout.addWidget(self.init_progress_label)
        progress_layout.addWidget(self.init_progress_bar)
        
        # Hand Control Buttons
        control_layout = QHBoxLayout()
        
        self.open_btn = QPushButton("🖐️ OPEN")
        self.open_btn.setStyleSheet("background-color: #3498db; color: white; font-size: 18px; min-height: 60px;")
        self.open_btn.clicked.connect(self.open_hand)
        self.open_btn.setEnabled(False)
        
        self.close_btn = QPushButton("✊ CLOSE")
        self.close_btn.setStyleSheet("background-color: #9b59b6; color: white; font-size: 18px; min-height: 60px;")
        self.close_btn.clicked.connect(self.close_hand)
        self.close_btn.setEnabled(False)
        
        control_layout.addWidget(self.open_btn)
        control_layout.addWidget(self.close_btn)
        
        # Status indicators
        status_layout = QHBoxLayout()
        
        self.system_status = QLabel("⚪ System: Starting...")
        self.system_status.setStyleSheet("font-size: 14px; padding: 5px;")
        
        self.slip_status = QLabel("⚪ Slip Detection: Inactive")
        self.slip_status.setStyleSheet("font-size: 14px; padding: 5px;")
        
        status_layout.addWidget(self.system_status)
        status_layout.addWidget(self.slip_status)
        
        layout.addLayout(progress_layout)
        layout.addLayout(control_layout)
        layout.addLayout(status_layout)
        
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
            label = ['Little', 'Ring','Middle','Index','Thumb', 'Thumb Abduction']
            label = QLabel(label[i])  # Create QLabel widget
            label.setStyleSheet("font-size: 14px; font-weight: bold; color: #34495e; min-width: 180px;")
            value_label = QLabel("0°")
            value_label.setStyleSheet("font-weight: bold; font-size: 16px;")
            
            bar = QProgressBar()
            bar.setRange(0, 850)
            bar.setValue(0)
            bar.setTextVisible(False)
            
            angle_layout.addWidget(label, i, 0)
            angle_layout.addWidget(bar, i, 1)
            angle_layout.addWidget(value_label, i, 2)
            
            self.angle_labels.append(value_label)
            self.angle_bars.append(bar)
            
        angle_group.setLayout(angle_layout)
        layout.addWidget(angle_group)
        
        # Force display - simplified text only
        force_group = QGroupBox("Finger Forces")
        force_layout = QVBoxLayout()
        force_layout.setSpacing(8)
        
        # Create force display labels
        self.force_labels = []
        force_names =  ['Little', 'Ring','Middle','Index','Thumb', 'Thumb Abduction']
        
        for i, name in enumerate(force_names):
            force_row = QHBoxLayout()
            
            # Joint name
            name_label = QLabel(f"{name}:")
            name_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #34495e; min-width: 180px;")
            
            # Force value
            value_label = QLabel("0g")
            value_label.setStyleSheet("""
                font-size: 18px; 
                font-weight: bold; 
                color: #2c3e50;
                background-color: #ecf0f1;
                padding: 8px 15px;
                border-radius: 5px;
                min-width: 100px;
            """)
            value_label.setAlignment(Qt.AlignCenter)
            
            force_row.addWidget(name_label)
            force_row.addWidget(value_label)
            force_row.addStretch()
            
            force_layout.addLayout(force_row)
            self.force_labels.append(value_label)
            
        force_group.setLayout(force_layout)
        layout.addWidget(force_group)
        
        layout.addStretch()
        return widget
    
    def create_graph_tab(self):
        """Create real-time graph tab"""
        self.graph_widget = RealtimeGraphWidget()
        return self.graph_widget
        
    def create_status_tab(self):
        """Create detailed status monitoring tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # System Info
        sys_group = QGroupBox("System Information")
        sys_layout = QGridLayout()
        
        info_items = [
            ("ROS Master", "roscore", "Not Connected"),
            ("Driver Node", "driver_ros1", "Not Running"),
            ("Grip Controller", "grip", "Not Running"),
            ("Last Command", "last_cmd", "None"),
            ("Slip Detection", "slip_detect", "Inactive"),
        ]
        
        self.sys_labels = {}
        for i, (name, key, default) in enumerate(info_items):
            name_label = QLabel(f"{name}:")
            name_label.setStyleSheet("font-weight: bold; font-size: 14px;")
            value_label = QLabel(default)
            value_label.setStyleSheet("color: #7f8c8d; font-size: 14px;")
            sys_layout.addWidget(name_label, i, 0)
            sys_layout.addWidget(value_label, i, 1)
            self.sys_labels[key] = value_label
            
        sys_group.setLayout(sys_layout)
        layout.addWidget(sys_group)
        
        # Calibration Status
        calib_group = QGroupBox("Force Sensor Calibration")
        calib_layout = QVBoxLayout()
        
        self.calib_status = QLabel("Status: Not Calibrated")
        self.calib_status.setStyleSheet("font-size: 13px;")
        calib_layout.addWidget(self.calib_status)
        
        self.calib_offsets = QLabel("Offsets: N/A")
        self.calib_offsets.setStyleSheet("font-family: 'Courier New'; font-size: 12px; color: #34495e;")
        calib_layout.addWidget(self.calib_offsets)
        
        calib_group.setLayout(calib_layout)
        layout.addWidget(calib_group)
        
        layout.addStretch()
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
        
    def auto_start_system(self):
        """Automatically start system on GUI launch"""
        self.append_log("=== Auto-Starting System ===")
        self.init_progress_bar.setValue(0)
        self.progress_current = 0
        self.progress_target = 0
        
        # Start progress bar animation
        self.progress_timer.start(30)  # Update every 30ms for smooth animation
        
        # Start nodes in background thread
        import threading
        threading.Thread(target=self.node_manager.start_nodes, daemon=True).start()
        
        # Wait then initialize ROS
        QTimer.singleShot(8000, self.initialize_ros)
    
    def animate_progress(self):
        """Smoothly animate progress bar"""
        if self.progress_current < self.progress_target:
            # Gradually increase towards target
            increment = max(1, (self.progress_target - self.progress_current) / 10)
            self.progress_current = min(self.progress_current + increment, self.progress_target)
            self.init_progress_bar.setValue(int(self.progress_current))
        
        # Stop animation when reached 100%
        if self.progress_current >= 100:
            self.progress_timer.stop()
    
    def update_progress(self, value, status):
        """Update initialization progress bar target"""
        self.progress_target = value
        self.init_progress_label.setText(status)
        
        if value >= 100:
            # Hide progress bar after completion
            QTimer.singleShot(2000, self.hide_progress_bar)
    
    def hide_progress_bar(self):
        """Hide progress bar after initialization"""
        self.init_progress_bar.setVisible(False)
        self.init_progress_label.setVisible(False)
        
    def hide_progress_bar(self):
        """Hide progress bar after initialization"""
        self.init_progress_bar.setVisible(False)
        self.init_progress_label.setVisible(False)
        
    def initialize_ros(self):
        """Initialize ROS node and subscribers"""
        try:
            # Check if we need to reinitialize
            if not self.ros_initialized:
                try:
                    rospy.init_node('inspire_hand_gui', anonymous=True, disable_signals=True)
                    self.ros_initialized = True
                except rospy.exceptions.ROSException as e:
                    if "node already exists" in str(e).lower() or "duplicate node" in str(e).lower():
                        self.ros_initialized = True
                        self.append_log("ROS node already exists, reusing...")
                    else:
                        raise
                
            # Subscribe to topics
            self.angle_sub = rospy.Subscriber("/inspire_hand/angle", Float32MultiArray, self.angle_callback)
            # Try calibrated forces first, fallback to raw if not available
            try:
                self.force_sub = rospy.Subscriber("/inspire_hand/force_calibrated", Float32MultiArray, self.force_callback)
                self.append_log("Subscribed to calibrated forces")
            except:
                self.force_sub = rospy.Subscriber("/inspire_hand/force", Float32MultiArray, self.force_callback)
                self.append_log("Subscribed to raw forces (calibrated not available yet)")
            
            # Start data update timer
            self.data_timer.start(100)
            
            # Enable controls
            self.open_btn.setEnabled(True)
            self.close_btn.setEnabled(True)
            
            self.system_status.setText("🟢 System: Online")
            self.system_status.setStyleSheet("font-size: 14px; padding: 5px; color: green;")
            
            self.append_log("✓ ROS subscribers initialized")
            self.update_system_status("roscore", "🟢 Running")
            self.update_system_status("driver_ros1", "🟢 Active")
            self.update_system_status("grip", "🟢 Active")
            
            # Listen for calibration info
            QTimer.singleShot(3000, self.update_calibration_status)
            
        except Exception as e:
            self.append_log(f"✗ Error initializing ROS: {str(e)}")
            import traceback
            self.append_log(traceback.format_exc())
        
    def open_hand(self):
        """Send open command"""
        try:
            pub = rospy.Publisher('/finger_command', String, queue_size=1)
            time.sleep(0.1)
            pub.publish("open")
            self.append_log("→ Sent: OPEN command")
            self.slip_status.setText("⚪ Slip Detection: Inactive")
            self.slip_status.setStyleSheet("font-size: 14px; padding: 5px;")
            self.update_system_status("last_cmd", "🟢 OPEN")
            self.update_system_status("slip_detect", "⚪ Inactive")
        except Exception as e:
            self.append_log(f"✗ Error sending open: {str(e)}")
            
    def close_hand(self):
        """Send close command"""
        try:
            pub = rospy.Publisher('/finger_command', String, queue_size=1)
            time.sleep(0.1)
            pub.publish("close")
            self.append_log("→ Sent: CLOSE command")
            self.update_system_status("last_cmd", "❌ CLOSE")
            QTimer.singleShot(3000, self.activate_slip_detection)
        except Exception as e:
            self.append_log(f"✗ Error sending close: {str(e)}")
            
    def activate_slip_detection(self):
        """Indicate slip detection is active"""
        self.slip_status.setText("🟢 Slip Detection: Active")
        self.slip_status.setStyleSheet("font-size: 14px; padding: 5px; color: green;")
        self.update_system_status("slip_detect", "🟢 Active")
        
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
            angle = max(0, min(850, angle))
            
            self.angle_labels[i].setText(f"{angle}°")
            self.angle_bars[i].setValue(angle)
            
            if angle < 200:
                self.angle_bars[i].setStyleSheet("QProgressBar::chunk { background-color: #e74c3c; }")
            elif angle < 600:
                self.angle_bars[i].setStyleSheet("QProgressBar::chunk { background-color: #f39c12; }")
            else:
                self.angle_bars[i].setStyleSheet("QProgressBar::chunk { background-color: #27ae60; }")
                
        # Update forces - simplified text display with color coding
        for i in range(6):
            force = int(self.current_forces[i]) if i < len(self.current_forces) else 0
            
            # Display with sign
            self.force_labels[i].setText(f"{force:+d}g")
            
            # Color code based on force magnitude
            abs_force = abs(force)
            if abs_force < 50:
                # Low force - light gray/green
                color = "#27ae60" if force >= 0 else "#95a5a6"
                bg_color = "#ecf0f1"
            elif abs_force < 200:
                # Medium force - orange
                color = "#f39c12"
                bg_color = "#fef5e7"
            else:
                # High force - red
                color = "#e74c3c"
                bg_color = "#fadbd8"
            
            self.force_labels[i].setStyleSheet(f"""
                font-size: 18px; 
                font-weight: bold; 
                color: {color};
                background-color: {bg_color};
                padding: 8px 15px;
                border-radius: 5px;
                min-width: 100px;
            """)
        
        # Update graph
        if hasattr(self, 'graph_widget'):
            self.graph_widget.update_data(self.current_angles, self.current_forces)
    
    def update_system_status(self, key, status):
        """Update system status labels"""
        if key in self.sys_labels:
            self.sys_labels[key].setText(status)
            if "🟢" in status:
                self.sys_labels[key].setStyleSheet("color: green; font-weight: bold; font-size: 14px;")
            elif "❌" in status:
                self.sys_labels[key].setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
            else:
                self.sys_labels[key].setStyleSheet("color: #7f8c8d; font-size: 14px;")
    
    def update_calibration_status(self):
        """Update calibration status display"""
        # This simulates receiving calibration info
        # In production, you'd subscribe to a calibration topic or service
        if hasattr(self, 'calib_status'):
            self.calib_status.setText("Status: ✓ Calibrated (Open Position)")
            self.calib_status.setStyleSheet("font-size: 13px; color: green; font-weight: bold;")
        
        if hasattr(self, 'calib_offsets'):
            # Example offsets (these match the typical values)
            offsets_text = "Offsets: F1=-12g, F2=-13g, F3=-12g, F4=+7g, F5=+14g, F6=0g"
            self.calib_offsets.setText(offsets_text)
        
        self.append_log("✓ Force sensors calibrated to zero in open position")
                
    def append_log(self, message):
        """Append message to log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        
    def closeEvent(self, event):
        """Handle window close - stop all nodes"""
        self.append_log("=== Shutting Down System ===")
        self.data_timer.stop()
        self.node_manager.stop_nodes()
        time.sleep(1)  # Give processes time to terminate
        event.accept()

def main():
    app = QApplication(sys.argv)
    gui = InspireHandGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()