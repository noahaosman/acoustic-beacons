#!/usr/bin/env python3

# Periodically forward current position to PixHawk
#
# NOTE: The only GGA fields parsed by the code that consumes
# these messages on the ROV (nmea-receiver.py) are:
# lat, lon, altitude, hdop, and satellites_visible.
# Satellites_visible has to be greater than 6 for the Pixhawk to
# use the lat/lon data, so we hardcode it here as 7.

import socket
import pynmea2
from datetime import datetime
import time

# GPS data are expected on port 27000
# set up a client socket (the ROV companion has the server socket)
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(('127.0.0.1', 27000))
    print('client connected')

    fix_quality = '7'  # 7: manual input mode
    sat_vis = '07'  # has to be greater than 6 for Pixhawk to use it
    hdop = '1.0'  # ideal (<1), excellent (1-2), good (2-5)
    interval = 1.0  # time between messages in seconds

    while True:
        nowtime = datetime.utcnow().strftime('%H%M%S.00')
        lat_nmea = '4433.6178'
        lat_dir = 'N'
        lon_nmea = '12345.6117'
        lon_dir = 'W'
        alt = '0'
        # create a GGA NMEA sentence
        msg_obj = pynmea2.GGA(
          'GP', 'GGA',
          (nowtime,
           lat_nmea, lat_dir,
           lon_nmea, lon_dir,
           fix_quality, sat_vis, hdop,
           alt, 'M', '', 'M', '', ''))
        msg = str(msg_obj)
        # msg = '$GPGGA,211740.00,4433.765000,N,12316.724554,W,7,06,1.0,-10.07960972883314,M,,M,,*4B'
        print(msg)
        # send gps position to ROV
        # sockit.sendto(msg.encode(), ('0.0.0.0', 27000))
        # sockit.sendto(msg.encode(), ('192.168.2.2', 27000))
        s.send(msg.encode())
        print('sent message')
        time.sleep(interval)
