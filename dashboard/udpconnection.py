import threading
import socket
import queue
from PySide6 import QtCore
import logging

class UDPRobotLink(QtCore.QObject):
    # Signals sending the new UDP data
    data_received = QtCore.Signal(dict)
    
    def __init__(self, remote_ip : str | None, remote_port : int | None, local_ip : str ="0.0.0.0", local_port : int = 5005):
        super().__init__()
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.local_ip = local_ip
        self.local_port = local_port

        # Create and bind the socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.local_ip, self.local_port))

        # Create output queue
        self.send_queue = queue.Queue()

    def start(self):
        self.running = True
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.output_thread = threading.Thread(target=self._output_loop, daemon=True)

        self.input_thread.start()
        self.output_thread.start()

        # TODO actively subscribe to the bot when a remote address has been set

    def send(self, msg):
        self.send_queue.put(msg)

    def _subscribe(self, remote_ip : str, remote_port : int):
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        pass

    def _input_loop(self):
        print("UDP input started")

        while self.running:
            try:
                data, _ = self.udp_socket.recvfrom(1024)
                if(data):
                    # TODO unpack data
                    self.data_received.emit(data)

            except socket.timeout:
                continue
            except Exception as e:
                print(f"error: {e}")
                return
            
    def _output_loop(self):
        print("UDP output started")

        while self.running:
            try:
                msg = self.send_queue.get()

                if self.remote_ip and self.remote_port and msg:
                    len = self.udp_socket.sendto(msg.SerializeToString(), (self.remote_ip, self.remote_port))
                else:
                    logging.log(logging.ERROR, "Failed to send data because remote ip is not set or message was empty")

            except Exception as e:
                print(f"UDP output error: {e}")

    def stop(self):
        self.running = False