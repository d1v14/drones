#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: RegisLab
"""

import sys
from vispy import app, scene
import socket
import struct
import numpy as np
import threading


'''
Вектор состояния БЛА
Положение ЛА в стартовой СК
0 - X
1 - Y
2 - Z
Скорость ЛА в стартовой СК
3 - VelX
4 - VelY
5 - VelZ
Угловое положение ЛА
6 - Pitch
7 - Roll
8 - Yaw
Угловая скорость ЛА
9  - PitchRate
10 - RollRate
11 - YawRate
Метка времени симуляции
12 - timeStamp
'''

# вносим интересующую переменную вектора состояния сюда
########################################################
index =7        #
########################################################


class plotter():

    def __init__(self):
        self.data = []
        self.time = []
        self.canvas = scene.SceneCanvas(keys='interactive', show=True)
        self.grid = self.canvas.central_widget.add_grid(spacing=0)
        self.viewbox = self.grid.add_view(row=0, col=1, camera='panzoom')

        # add axes
        x_axis = scene.AxisWidget(orientation='bottom')
        x_axis.stretch = (1, 0.1)
        self.grid.add_widget(x_axis, row=1, col=1)
        x_axis.link_view(self.viewbox)
        y_axis = scene.AxisWidget(orientation='left')
        y_axis.stretch = (0.1, 1)
        self.grid.add_widget(y_axis, row=0, col=0)
        y_axis.link_view(self.viewbox)
        self.viewbox.camera.set_range((0, 100), (-20, 20))

        self.host = '127.0.0.1'
        self.port = 12346
        self.msgLen = 104
        self.addr = (self.host, self.port)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)

        self.udp_socket.bind(self.addr)
        self.currentDataIndex = index
        pos = np.array([[0, 0], [1, 1], [2, 2]])
        self.LastTime = 0
        self.line = scene.Line(pos, color=(0, 1, 1, 1),
                               parent=self.viewbox.scene)
        
        self.grid = scene.visuals.GridLines(scale=(1, 1), color='w',
                                            parent=self.viewbox.scene)

        self.stateVector = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rcv_thread = threading.Thread(target=self._reciveData,
                                      name="rcv_thread")
    

        
        rcv_thread.start()

    def _reciveData(self):
        while True:
            # print('wait data...')
            # recvfrom - получает UDP сообщения
            conn, addr = self.udp_socket.recvfrom(self.msgLen)
            self.stateVector = self._convertMessage(conn)
            if self.stateVector[-1] < self.LastTime:
                self.data = []
                self.time = []
                self.data.append(self.stateVector)
                self.time.append(self.stateVector[-1])

            self.data.append(self.stateVector)
            self.time.append(self.stateVector[-1])
            self.LastTime = self.stateVector[-1]

    def _convertMessage(self, data):
        revData = bytearray(reversed(data))
        decodeData = list(
            reversed(struct.unpack("!ddddddddddddd", revData)))
        return decodeData

    def startPlotter(self):
        timer = app.Timer()
        timer.connect(self._updatePlot)
        timer.start()
        app.run()

    def _updatePlot(self, ev):
        print(self.stateVector)
        self.currData = [i[self.currentDataIndex] for i in self.data]
       
        dataPlot = np.array([self.time, self.currData]).T
        
        self.line.set_data(pos=dataPlot, color=(0, 1, 1, 1))




if __name__ == '__main__' and sys.flags.interactive == 0:
    plot = plotter()
    plot.startPlotter()
