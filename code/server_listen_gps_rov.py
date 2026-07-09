#!/usr/bin/env python3

# Listen for incoming GPS data on port 27000

import socket

# GPS data are expected on port 27000
# set up a server socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind(('127.0.0.1', 27000))
    s.listen(5)  # become a server socket and allow max 5 connections
    print('waiting for connection')
    conn, addr = s.accept()
    with conn:
        print(addr)
        while True:
            data = conn.recv(1024)
            print(data)
            conn.sendall(data)
            if not data:
                break
