import sys
from PySide6 import QtWidgets, QtCore
import pyqtgraph as pg
from udpconnection import UDPRobotLink
from tab_telemetry import TelemetryTab
from tab_pids import PidTab

class TelemetryDash(QtWidgets.QMainWindow):
    send_signal = QtCore.Signal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("BalanceBot Telemetry Dashboard")
        self.resize(1000, 700)

        # UDP connection object
        self.connection = UDPRobotLink("192.168.2.145", 3334)
        self.connection.start()
        self.send_signal.connect(self.send_data)

        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)
        
        self.tabs = QtWidgets.QTabWidget()
        self.layout.addWidget(self.tabs)

        # Create statusbar
        self.status = self.statusBar()

        # Create tabs
        self.tab_telemetry = TelemetryTab()
        self.tab_pid = PidTab(self.send_signal)
        #self.tab_sensors = QtWidgets.QWidget()
        #self.tab_control = QtWidgets.QWidget()
        #self.tab_settings = QtWidgets.QWidget()
        
        self.tabs.addTab(self.tab_telemetry, "Telemetry")
        self.tabs.addTab(self.tab_pid, "PIDs")
        #self.tabs.addTab(self.tab_pids, "PIDs")
        #self.tabs.addTab(self.tab_sensors, "Sensors")
        #self.tabs.addTab(self.tab_control, "Control")
        #self.tabs.addTab(self.tab_settings, "Settings")

        # --- SETUP TAB 1: Grafieken ---
        #self.setup_graph_tab()

    def setup_settings_tab(self):
        # Simpele layout voor instellingen
        layout = QtWidgets.QFormLayout(self.tab_settings)
        
        self.ip_input = QtWidgets.QLineEdit("127.0.0.1")
        self.port_input = QtWidgets.QLineEdit("5005")
        self.save_btn = QtWidgets.QPushButton("Verbinding Bijwerken")
        
        layout.addRow("Bot IP:", self.ip_input)
        layout.addRow("Poort:", self.port_input)
        layout.addRow(self.save_btn)

    @QtCore.Slot(object)
    def send_data(self, msg):
        self.connection.send(msg)






if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion") 
    dash = TelemetryDash()
    dash.show()
    sys.exit(app.exec_())