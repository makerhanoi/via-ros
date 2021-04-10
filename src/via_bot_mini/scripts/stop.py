#!/usr/bin/env python3
from __future__ import print_function

import socket

CONTROL_IP = "192.168.4.1"
CONTROL_PORT = 1234
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)

control_msg = "{} {}".format(
    0, 0).encode('ascii')
sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))
