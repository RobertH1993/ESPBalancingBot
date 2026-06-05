from PySide6 import QtWidgets, QtCore
import pyqtgraph as pg


class TelemetryTab(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.layout = QtWidgets.QGridLayout()

        # Angle widget
        self.balance_widget = pg.PlotWidget(title="Balance")
        self.balance_widget.addLegend()
        self.balance_widget.showGrid(x=True, y=True)
        self.balance_angle_curve = self.balance_widget.plot(pen='y', name='Current Angle')
        self.balance_setpoint_curve = self.balance_widget.plot(pen='r', name="Setpoint")
        self.layout.addWidget(self.balance_widget, 0, 0)

        # Balance PID widget
        self.balance_pid_widget = pg.PlotWidget(title="Balance PID Values")
        self.balance_pid_widget.addLegend()
        self.balance_pid_widget.showGrid(x=True, y=True)
        self.balance_pid_p_curve = self.balance_pid_widget.plot(pen='r', name='P')
        self.balance_pid_i_curve = self.balance_pid_widget.plot(pen='y', name='I')
        self.balance_pid_d_curve = self.balance_pid_widget.plot(pen='b', name='D')
        self.balance_pid_output_curve = self.balance_pid_widget.plot(pen='g', name='Output')
        self.layout.addWidget(self.balance_pid_widget, 0, 1)

        # Speed widget
        self.speed_widget = pg.PlotWidget(title="Motor Speed (RPM)")
        self.speed_widget.addLegend()
        self.speed_widget.showGrid(x=True, y=True)
        self.speed_left_motor_curve = self.speed_widget.plot(pen='r', name="Left")
        self.speed_right_motor_curve = self.speed_widget.plot(pen='b', name="Right")
        self.layout.addWidget(self.speed_widget, 1, 0)

        # Power widget
        self.power_widget = pg.PlotWidget(title="Power")
        self.power_widget.addLegend()
        self.power_widget.showGrid(x=True, y=True)
        self.power_voltage_curve = self.power_widget.plot(pen="r", name="Voltage")
        self.power_amp_curve = self.power_widget.plot(pen="y", name="Amps")
        self.layout.addWidget(self.power_widget, 1, 1)

        self.setLayout(self.layout)



