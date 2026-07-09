#!/usr/bin/env python3

# Listen for incoming GPS data on port 27000

import socket

# GPS data are expected on port 27000
# set up a client socket to listen for the gps data
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(('127.0.0.1', 27000))
    print('client connected')
    while True:
        # s.send(b'hi')
        data = s.recv(1024)
        print(data)
        if not data:
            break
