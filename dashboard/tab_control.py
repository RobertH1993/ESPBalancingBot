from PySide6 import QtWidgets, QtCore, QtGui
import robot_pb2

class ControlTab(QtWidgets.QWidget):
    def __init__(self, send_signal):
        super().__init__()
        self.send_signal = send_signal
        self.transaction_id = 0
        
        # Current control values
        self.speed = 0.0
        self.turn_rate = 0.0
        
        self.setup_ui()
        
    def setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        
        # Title
        title = QtWidgets.QLabel("Robot Besturing")
        title.setStyleSheet("font-size: 18pt; font-weight: bold;")
        layout.addWidget(title)
        
        # Create control panel
        control_layout = QtWidgets.QHBoxLayout()
        
        # Left side: Button controls
        button_layout = QtWidgets.QVBoxLayout()
        
        # Direction buttons in grid
        button_grid = QtWidgets.QGridLayout()
        
        self.btn_forward = QtWidgets.QPushButton("▲ Vooruit")
        self.btn_forward.setMinimumSize(120, 60)
        self.btn_forward.setStyleSheet("font-size: 14pt;")
        button_grid.addWidget(self.btn_forward, 0, 1)
        
        self.btn_left = QtWidgets.QPushButton("◄ Links")
        self.btn_left.setMinimumSize(120, 60)
        self.btn_left.setStyleSheet("font-size: 14pt;")
        button_grid.addWidget(self.btn_left, 1, 0)
        
        self.btn_stop = QtWidgets.QPushButton("■ Stop")
        self.btn_stop.setMinimumSize(120, 60)
        self.btn_stop.setStyleSheet("font-size: 14pt; background-color: #ff4444;")
        button_grid.addWidget(self.btn_stop, 1, 1)
        
        self.btn_right = QtWidgets.QPushButton("► Rechts")
        self.btn_right.setMinimumSize(120, 60)
        self.btn_right.setStyleSheet("font-size: 14pt;")
        button_grid.addWidget(self.btn_right, 1, 2)
        
        self.btn_backward = QtWidgets.QPushButton("▼ Achteruit")
        self.btn_backward.setMinimumSize(120, 60)
        self.btn_backward.setStyleSheet("font-size: 14pt;")
        button_grid.addWidget(self.btn_backward, 2, 1)
        
        button_layout.addLayout(button_grid)
        button_layout.addStretch()
        
        control_layout.addLayout(button_layout)
        
        # Right side: Slider controls
        slider_layout = QtWidgets.QFormLayout()
        
        # Speed slider
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setMinimum(-100)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(0)
        self.speed_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.speed_slider.setTickInterval(10)
        
        self.speed_label = QtWidgets.QLabel("0 cm/s")
        self.speed_label.setMinimumWidth(80)
        
        speed_layout = QtWidgets.QHBoxLayout()
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        slider_layout.addRow("Snelheid:", speed_layout)
        
        # Turn rate slider
        self.turn_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.turn_slider.setMinimum(-500)
        self.turn_slider.setMaximum(500)
        self.turn_slider.setValue(0)
        self.turn_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.turn_slider.setTickInterval(30)
        
        self.turn_label = QtWidgets.QLabel("0 deg/s")
        self.turn_label.setMinimumWidth(80)
        
        turn_layout = QtWidgets.QHBoxLayout()
        turn_layout.addWidget(self.turn_slider)
        turn_layout.addWidget(self.turn_label)
        slider_layout.addRow("Draaisnelheid:", turn_layout)
        
        # Send button
        self.btn_send = QtWidgets.QPushButton("Verstuur Commando")
        self.btn_send.setMinimumHeight(40)
        slider_layout.addRow("", self.btn_send)
        
        control_layout.addLayout(slider_layout)
        
        layout.addLayout(control_layout)
        
        # Status display
        status_group = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QFormLayout()
        
        self.status_speed = QtWidgets.QLabel("0.0 cm/s")
        self.status_turn = QtWidgets.QLabel("0.0 deg/s")
        
        status_layout.addRow("Huidige snelheid:", self.status_speed)
        status_layout.addRow("Huidige draaisnelheid:", self.status_turn)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
        
        # Connect signals
        self.btn_forward.pressed.connect(self.on_forward_pressed)
        self.btn_forward.released.connect(self.on_button_released)
        self.btn_backward.pressed.connect(self.on_backward_pressed)
        self.btn_backward.released.connect(self.on_button_released)
        self.btn_left.pressed.connect(self.on_left_pressed)
        self.btn_left.released.connect(self.on_button_released)
        self.btn_right.pressed.connect(self.on_right_pressed)
        self.btn_right.released.connect(self.on_button_released)
        self.btn_stop.clicked.connect(self.on_stop_clicked)
        
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        self.turn_slider.valueChanged.connect(self.on_turn_changed)
        self.btn_send.clicked.connect(self.send_control)
        
    def on_forward_pressed(self):
        self.speed = 10.0
        self.speed_slider.setValue(int(self.speed))
        self.send_control()
        
    def on_backward_pressed(self):
        self.speed = -10.0
        self.speed_slider.setValue(int(self.speed))
        self.send_control()
        
    def on_left_pressed(self):
        self.turn_rate = -90.0
        self.turn_slider.setValue(int(self.turn_rate))
        self.send_control()
        
    def on_right_pressed(self):
        self.turn_rate = 90.0
        self.turn_slider.setValue(int(self.turn_rate))
        self.send_control()
        
    def on_button_released(self):
        self.speed = 0.0
        self.turn_rate = 0.0
        self.speed_slider.setValue(0)
        self.turn_slider.setValue(0)
        self.send_control()
        
    def on_stop_clicked(self):
        self.speed = 0.0
        self.turn_rate = 0.0
        self.speed_slider.setValue(0)
        self.turn_slider.setValue(0)
        self.send_control()
        
    def on_speed_changed(self, value):
        self.speed = float(value)
        self.speed_label.setText(f"{value} cm/s")
        
    def on_turn_changed(self, value):
        self.turn_rate = float(value)
        self.turn_label.setText(f"{value} deg/s")
        
    def send_control(self):
        # Update status display
        self.status_speed.setText(f"{self.speed:.1f} cm/s")
        self.status_turn.setText(f"{self.turn_rate:.1f} deg/s")
        
        # Create and send SetRobotControl message
        msg = robot_pb2.ConfigMessage()
        msg.transaction_id = self.transaction_id
        self.transaction_id += 1
        
        msg.set_robot_controlp.speed = self.speed
        msg.set_robot_controlp.turn_rate = self.turn_rate
        
        self.send_signal.emit(msg)
