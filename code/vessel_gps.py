# Forwards serial vessel GPS data to the 
# ROV companion computer (192.168.2.2) at UDP port 27099.
#
# Usage:
#   python vessel_gps.py
#

import socket
import serial


class VesselGPS:
    """Handles Vessel GPS functions."""

    def __init__(self):

        # Vessel GPS settings
        self.serial_port = 'COM1'  # DB9 port on the laptop
        self.serial_baud = 57600  # vessel GPS baud rate
        # self.broadcast_address = '10.0.1.255'  # mixz network
        # self.broadcast_address = '192.168.2.2'  # ROV network
        self.udp_port = 14401  # default QGroundControl NMEA device port

    def open_serial_port(self):
        """Opens the serial port."""

        self.port_open = False
        try:
            self.ser_port = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.serial_baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1
                )
            if self.ser_port.is_open:
                self.port_open = True
        except Exception:
            print('cannot open serial port')
            pass

    def serial_to_udp(self):
        """Listen for all incoming serial messages and forwards them to UDP.
        """

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s1:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s2:
                # s1.bind(('',0))
                # s2.bind(('',0))
                # s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                # Reads from serial port
                while self.ser_port.is_open:

                    msg_str = self.ser_port.readline().decode('utf-8', 'ignore').strip()
                    if msg_str:
                        print(msg_str)
                        try:
                            msg_bytes = bytes(str(msg_str) + '\r\n', "ascii")
                            s1.sendto(msg_bytes, ('192.168.2.2', self.udp_port))
                            s2.sendto(msg_bytes, ('192.168.2.1', self.udp_port))
                        except Exception as e:
                            print('could not send message')
                            pass

    def run(self):
        self.open_serial_port()
        if self.port_open:
            self.serial_to_udp()


v = VesselGPS()
v.run()
