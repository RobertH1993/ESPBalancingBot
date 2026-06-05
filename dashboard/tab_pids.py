from PySide6 import QtWidgets
import pyqtgraph as pg
import robot_pb2

class PidTab(QtWidgets.QWidget):
    def __init__(self, send_signal, parent=None):
        super().__init__(parent)
        self.send_signal = send_signal
        self.layout = QtWidgets.QGridLayout()

        # Graphs
        self.balance_widget = pg.PlotWidget(title="Setpoint and Error")
        self.balance_widget.addLegend()
        self.balance_widget.showGrid(x=True, y=True)
        self.balance_angle_curve = self.balance_widget.plot(pen='y', name='Setpoint')
        self.balance_setpoint_curve = self.balance_widget.plot(pen='r', name="Error")
        self.layout.addWidget(self.balance_widget, 0, 0)

        self.balance_pid_widget = pg.PlotWidget(title="PID Values and output")
        self.balance_pid_widget.addLegend()
        self.balance_pid_widget.showGrid(x=True, y=True)
        self.balance_pid_p_curve = self.balance_pid_widget.plot(pen='r', name='P')
        self.balance_pid_i_curve = self.balance_pid_widget.plot(pen='y', name='I')
        self.balance_pid_d_curve = self.balance_pid_widget.plot(pen='b', name='D')
        self.balance_pid_output_curve = self.balance_pid_widget.plot(pen='g', name='Output')
        self.layout.addWidget(self.balance_pid_widget, 1, 0)

        # Controls
        self.controls_layout = QtWidgets.QVBoxLayout()
        self.controls_group = QtWidgets.QGroupBox("PID Tuning")
        self.controls_group_layout = QtWidgets.QFormLayout()

        # PID select
        self.pid_selector = QtWidgets.QComboBox()
        self.pid_selector.addItems(["BALANCE", "SPEED", "POSITION", "WHEEL_TRIM"])
        self.controls_group_layout.addRow("Target PID:", self.pid_selector)

        # PID fields
        self.kp_input = QtWidgets.QDoubleSpinBox()
        self.kp_input.setRange(0, 1000)
        self.kp_input.setDecimals(4)
        self.kp_input.setSingleStep(0.1)
        self.controls_group_layout.addRow("Kp:", self.kp_input)

        self.ki_input = QtWidgets.QDoubleSpinBox()
        self.ki_input.setRange(0, 1000)
        self.ki_input.setDecimals(4)
        self.ki_input.setSingleStep(0.01)
        self.controls_group_layout.addRow("Ki:", self.ki_input)

        self.kd_input = QtWidgets.QDoubleSpinBox()
        self.kd_input.setRange(0, 10)
        self.kd_input.setDecimals(4)
        self.kd_input.setSingleStep(0.01)
        self.controls_group_layout.addRow("Kd:", self.kd_input)

        # Send button
        self.send_button = QtWidgets.QPushButton("Apply PID Settings")
        self.send_button.setStyleSheet("background-color: #FFC107; color: white; font-weight: bold; padding: 10px;")
        self.send_button.clicked.connect(self.on_send_clicked)
        self.controls_group_layout.addRow(self.send_button)

        # Save button
        self.save_button = QtWidgets.QPushButton("Save PID Settings")
        self.save_button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.save_button.clicked.connect(self.on_save_clicked)
        self.controls_group_layout.addRow(self.save_button)

        self.controls_group.setLayout(self.controls_group_layout)
        self.controls_layout.addWidget(self.controls_group)
        self.controls_layout.addStretch()

        self.layout.addLayout(self.controls_layout, 0, 1, 2, 1)

        self.layout.setColumnStretch(0, 4)
        self.layout.setColumnStretch(1, 1)

        self.setLayout(self.layout)

    def on_send_clicked(self):
        target = self.pid_selector.currentText()
        kp = self.kp_input.value()
        ki = self.ki_input.value()
        kd = self.kd_input.value()
        print(f"Sending {target}: Kp={kp}, Ki={ki}, Kd={kd}")

        target_enum = getattr(robot_pb2, target)
        msg = robot_pb2.ConfigMessage()
        msg.set_pid_paramsp.target = target_enum
        msg.set_pid_paramsp.kp = kp
        msg.set_pid_paramsp.ki = ki
        msg.set_pid_paramsp.kd = kd
        self.send_signal.emit(msg)

    def on_save_clicked(self):
        print(f"TODO Sending save signal")