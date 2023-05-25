#!/usr/bin/python3
""" Script to manage a message/command received via beacon.

Usage:
receive_command.py
where PORT is the port socket to listen on.
"""

import socket

PORT = 3000
HOST = '127.0.0.1'
MAX_LENGTH = 4096

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.bind((HOST, PORT))
serversocket.listen(5)

while True:
    # accept connections from outside
    (clientsocket, address) = serversocket.accept()

    msg = clientsocket.recv(MAX_LENGTH)
    if msg == '':  # client terminated connection
        clientsocket.close()
    strmsg = msg.decode()
    print(strmsg)
