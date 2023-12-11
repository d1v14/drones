#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 15 01:04:47 2021

@author: argus
"""

import socket
import sys
import numpy as np
import struct
import time
host = '127.0.0.1'
port = 12346
addr = (host, port)

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# объявляем переменные вектора состояния
X = 0
Y = 0
Z = 2
Vx = 0
Vy = 0
Vz = 0
roll = 0
pitch = 0  # np.pi/2
yaw = 0
OmegaXbody = 0
OmegaYbody = 0
OmegaZbody = 0
timeStamp = 0

# упаковываем вектор состояния в структуру
data = bytearray(struct.pack("ddddddddddddd", X, Y, Z, Vx, Vy,
                 Vz, roll, pitch, yaw, OmegaXbody, OmegaYbody, OmegaZbody, timeStamp))
# отправляем структуру данных по UDP на указанный выше порт
udp_socket.sendto(data, addr)

# Раскомментируйте цикл для демонстрации движения ЛА в визуализаторе
while True:

    data = bytearray(struct.pack("ddddddddddddd", X, Y, Z, roll, pitch, yaw, Vx, Vy,
                     Vz, OmegaXbody, OmegaYbody, OmegaZbody, timeStamp))

    udp_socket.sendto(data, addr)
    pitch += 0.00
    roll += 0.003
    X += 0.001
    yaw += 0
    timeStamp+=0.001
    time.sleep(0.01)
    if roll > np.pi * 4:
        break


udp_socket.close()
