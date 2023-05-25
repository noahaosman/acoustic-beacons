#!/usr/bin/python3
""" Script to send a message/command to another beacon.

Usage:
send_command.py <BEACON_ID> <COMMAND>
where:
BEACON_ID is the numeric beacon ID to receive the message (e.g. 202)
COMMAND is the short text message to send.
"""

import socket
import sys

if len(sys.argv) < 3:
    print("Usage: send_command.py <BEACON_ID> <COMMAND>")
    exit()

HOST = '127.0.0.1'
PORT = 4000
s = socket.socket()
s.connect((HOST, PORT))

target = sys.argv[1]
msg = sys.argv[2]
fullmsg = target + ' ' + msg
s.send(fullmsg.encode())
s.close()
