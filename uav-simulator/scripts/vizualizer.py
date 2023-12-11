#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 23:25:07 2021

@author: argus
"""
from vispy import app, scene
from vispy.io import read_png
# from vispy.util.filter import gaussian_filter
from vispy.visuals.transforms import STTransform, MatrixTransform
from vispy import geometry
import numpy as np
import socket
import threading
import struct
from quat import TQuat

radToDeg = 57.29577951308232
drawTrajLine = True

class uavDynamicVisualizer():
    def __init__(self):
        self.canvas = scene.SceneCanvas(keys='interactive', bgcolor='grey')
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = "turntable"
        # 3D axis
        self.axis = scene.visuals.XYZAxis(parent=self.view.scene)

        xax = scene.Axis(pos=[[0, 0], [5, 0]], tick_direction=(0, -1),
                         axis_color='r', tick_color='r', text_color='r',
                         font_size=16, parent=self.view.scene, domain=(0., 5.))

        yax = scene.Axis(pos=[[0, 0], [0, 5]], tick_direction=(-1, 0),
                         axis_color='g', tick_color='g', text_color='g',
                         font_size=16, parent=self.view.scene, domain=(0., 5.))

        zax = scene.Axis(pos=[[0, 0], [-5, 0]], tick_direction=(0, -1),
                         axis_color='b', tick_color='b', text_color='b',
                         font_size=16, parent=self.view.scene, domain=(0., 5.))

        # # its acutally an inverted xaxis
        zax.transform = scene.transforms.MatrixTransform()
        zax.transform.rotate(90, (0, 1, 0))  # rotate cw around yaxis
        zax.transform.rotate(-45, (0, 0, 1))  # tick direction towards (-1,-1)

        img_data = read_png('textures/grass.png')
        interpolation = "nearest"
        self.image = scene.visuals.Image(img_data, interpolation=interpolation,
                                         parent=self.view.scene, method='auto')
        imageScale = 0.01
        self.image.transform = STTransform(translate=[-self.image.size[0] *
                                                      imageScale/2,
                                                      -self.image.size[1] *
                                                      imageScale/2, -0.1],
                                           scale=(imageScale, imageScale))
        self._initUavGeometry()

      
        self.server_address = ('127.0.0.1', 12346)
        self.msgLen = 104
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        self.udp_socket.bind(self.server_address)
        

        self.stateVector = [0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rcv_thread = threading.Thread(target=self._reciveData,
                                      name="rcv_thread")
        rcv_thread.start()
        # средняя скорость вращения роторов (рад/сек)
        self.markers = scene.visuals.GridLines()
        self.dataX = [0]
        self.dataY = [0]
        self.dataZ = [0]
        self.dataPos = []

        pos = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]])
        self.line = scene.Line(pos, color=(0, 1, 1, 1), width=3)
        self.view.add(self.line)
        self.scatter = scene.visuals.Markers(edge_color=(1, 1, 1, 1),
                                             face_color=(0, 0, 1, 1),
                                             size=15)

    def _addArrow(self, lenth, radius):
        meshData = geometry.generation.create_arrow(rows=100, cols=100,
                                                    radius=0.1,
                                                    length=1,
                                                    cone_radius=None,
                                                    cone_length=None)
        uavTrustArrow = scene.visuals.Mesh(mode="triangles",
                                           meshdata=meshData)

        return uavTrustArrow

    def _addPropVisualization(self, pose, poseOffset):
        """
        :param pose: DESCRIPTION
        :type pose: TYPE
        :param poseOffset: DESCRIPTION
        :type poseOffset: TYPE
        :return: DESCRIPTION
        :rtype: TYPE

        """
        rows = 100
        cols = 100
        meshData = geometry.generation.create_cylinder(rows, cols,
                                                       radius=[0.05, 0.5],
                                                       length=0.1,
                                                       offset=False)

        uavProp = scene.visuals.Mesh(mode="triangles", meshdata=meshData)
        self.propPoseBodyList.append(pose+poseOffset)
        self.propVisList.append(uavProp)
        self.view.add(uavProp)

    def _initUavGeometry(self):
        self.bodyBoxHeight = 1
        self.bodyBoxWidth = 1
        self.bodyBoxDepth = 1
        self.armLenth = 1
        self.armWidth = 0.1
        self.armCount = 4
        self.armBodyHeight = 0
        self.armPoseBodyList = []
        self.armVisList = []
        self.propPoseBodyList = []
        self.propVisList = []
        self.arrowVisList = []
        self.armAngleBody = 0
        self.armOrientationBodyList = []
        self.uavBody = scene.visuals.Box(width=self.bodyBoxWidth,
                                         height=self.bodyBoxHeight,
                                         depth=self.bodyBoxDepth,
                                         color=(0, 0, 1, 1),
                                         edge_color='green')
        self.view.add(self.uavBody)
        # добавляем связную СК
        posAxes = np.array([[0, 0, 0],
                        [3, 0, 0],
                        [0, 0, 0],
                        [0, 3, 0],
                        [0, 0, 0],
                        [0, 0, 3]])
        self.axisBody = scene.visuals.XYZAxis(pos=posAxes, width = 5)
        self.view.add(self.axisBody)

        armAngleStep = 2 * np.pi / self.armCount
        currentAngle = self.armAngleBody
        # добавляем луч и пропеллер
        armPoseX = self.bodyBoxWidth / 2 + self.armLenth / 2
        # propPoseX = self.bodyBoxWidth / 2 + self.armLenth - 0.3
        armPoseY = 0
        armAngle = currentAngle
        armBodyX = armPoseX * np.cos(armAngle) - armPoseY * np.sin(armAngle)
        armBodyY = armPoseX * np.sin(armAngle) + armPoseY * np.cos(armAngle)
        armBodyZ = self.armBodyHeight
        pose = np.array([armBodyX, armBodyY, armBodyZ])
        self.armPoseBodyList.append(pose)
        self.armOrientationBodyList.append(np.array([0, 0, armAngle]))
        self.uavArm = scene.visuals.Box(width=self.armLenth,
                                        height=self.armWidth,
                                        depth=self.armWidth,
                                        color=(0, 1, 1, 1),
                                        edge_color='green')

        self.armVisList.append(self.uavArm)
        self.view.add(self.uavArm)
        currentAngle += armAngleStep
        poseOffset = np.array([self.armLenth + self.bodyBoxWidth / 2 - 1,
                               0, 0.05])
        self._addPropVisualization(pose, poseOffset)
        # добавляем луч и пропеллер
        armAngle = currentAngle
        armBodyX = armPoseX * np.cos(armAngle) - armPoseY * np.sin(armAngle)
        armBodyY = armPoseX * np.sin(armAngle) + armPoseY * np.cos(armAngle)
        armBodyZ = self.armBodyHeight
        pose = np.array([armBodyX, armBodyY, armBodyZ])
        self.armPoseBodyList.append(pose)
        self.armOrientationBodyList.append(np.array([0, 0, armAngle]))
        self.uavArm = scene.visuals.Box(width=self.armWidth,
                                        height=self.armWidth,
                                        depth=self.armLenth,
                                        color=(1, 0, 1, 1),
                                        edge_color='green')
        self.armVisList.append(self.uavArm)
        self.view.add(self.uavArm)
        currentAngle += armAngleStep
        poseOffset = np.array([0, self.armLenth + self.bodyBoxWidth / 2 - 1,
                               0.05])
        self._addPropVisualization(pose, poseOffset)
        # добавляем луч и пропеллер
        armAngle = currentAngle
        armBodyX = armPoseX * np.cos(armAngle) - armPoseY * np.sin(armAngle)
        armBodyY = armPoseX * np.sin(armAngle) + armPoseY * np.cos(armAngle)
        armBodyZ = self.armBodyHeight
        pose = np.array([armBodyX, armBodyY, armBodyZ])
        self.armPoseBodyList.append(pose)
        self.armOrientationBodyList.append(np.array([0, 0, armAngle]))
        self.uavArm = scene.visuals.Box(width=self.armLenth,
                                        height=self.armWidth,
                                        depth=self.armWidth,
                                        color=(0, 1, 1, 1),
                                        edge_color='green')
        self.armVisList.append(self.uavArm)
        self.view.add(self.uavArm)
        currentAngle += armAngleStep
        poseOffset = np.array([-(self.armLenth + self.bodyBoxWidth / 2 - 1),
                               0, 0.05])
        self._addPropVisualization(pose, poseOffset)
        # добавляем луч и пропеллер
        armAngle = currentAngle
        armBodyX = armPoseX * np.cos(armAngle) - armPoseY * np.sin(armAngle)
        armBodyY = armPoseX * np.sin(armAngle) + armPoseY * np.cos(armAngle)
        armBodyZ = self.armBodyHeight
        pose = np.array([armBodyX, armBodyY, armBodyZ])
        self.armPoseBodyList.append(pose)
        self.armOrientationBodyList.append(np.array([0, 0, armAngle]))
        self.uavArm = scene.visuals.Box(width=self.armWidth,
                                        height=self.armWidth,
                                        depth=self.armLenth,
                                        color=(0, 1, 1, 1),
                                        edge_color='green')
        self.armVisList.append(self.uavArm)
        self.view.add(self.uavArm)
        poseOffset = np.array([0, -(self.armLenth + self.bodyBoxWidth / 2 - 1),
                               0.05])
        self._addPropVisualization(pose, poseOffset)

    def startVisualization(self):
        """

        :return: DESCRIPTION
        :rtype: TYPE

        """
        # добавляем таймер и запускаем приложение
        timer = app.Timer()
        timer.connect(self._updatePlot)
        # timer.start(step, totalTime)
        timer.start()
        self.canvas.show()
        app.run()

    def _convertMessage(self, data):
        revData = bytearray(reversed(data))
        decodeData = list(
            reversed(struct.unpack("!ddddddddddddd", revData)))
        return decodeData

    def _reciveData(self):
        while True:
            # print('wait data...')
            # recvfrom - получает UDP сообщения
            conn, addr = self.udp_socket.recvfrom(self.msgLen)
            # print('client addr: ', addr)
            # print(conn)
            self.stateVector = self._convertMessage(conn)
            self.dataPos.append([self.stateVector[0],
                                 self.stateVector[1],
                                 self.stateVector[2]])
            # self.scatter.set_data(self.dataPos,
            #              edge_color=(0, 1, 1, 1),
            #              face_color=(0, 1, 1, 1),
            #              size=15)
            # sendto - передача сообщения UDP
            # self.udp_socket.sendto(b'message received by the server', addr)
            if drawTrajLine:
                self.dataX.append(self.stateVector[0])
                self.dataY.append(self.stateVector[1])
                self.dataZ.append(self.stateVector[2])

    def _transformObjectPosition(self, bodyOrientation, poseBody, visualObj):
        tr = MatrixTransform()
        tr.rotate(radToDeg * (bodyOrientation[0]), (1, 0, 0))
        tr.rotate(radToDeg * (bodyOrientation[1]), (0, 1, 0))
        tr.rotate(radToDeg * (bodyOrientation[2]), (0, 0, 1))

        worldPose = TQuat.rotate_vector_angles(bodyOrientation[0],
                                               bodyOrientation[1],
                                               bodyOrientation[2],
                                               poseBody)
        tr.translate((worldPose[0] + self.stateVector[0],
                      worldPose[1] + self.stateVector[1],
                      worldPose[2] + self.stateVector[2]))
        visualObj.transform = tr


    def _updatePlot(self, ev):

        # print(self.stateVector)
        tr = MatrixTransform()
        tr.rotate(radToDeg * self.stateVector[3], (1, 0, 0))
        tr.rotate(radToDeg * self.stateVector[4], (0, 1, 0))
        tr.rotate(radToDeg * self.stateVector[5], (0, 0, 1))
        tr.translate((self.stateVector[0],
                      self.stateVector[1],
                      self.stateVector[2]))
        self.uavBody.transform = tr
        self.axisBody.transform = tr

        bodyOrientation = np.array([self.stateVector[3],
                                    self.stateVector[4],
                                    self.stateVector[5]])

        if drawTrajLine:
            dataPlot = np.array([self.dataX, self.dataY, self.dataZ]).T
            # print(dataPlot)
            self.line.set_data(pos=dataPlot, color=(0, 1, 1, 1))

        for i in range(len(self.armVisList)):

            self._transformObjectPosition(bodyOrientation,
                                          self.armPoseBodyList[i],
                                          self.armVisList[i])

        for i in range(len(self.propVisList)):

            self._transformObjectPosition(bodyOrientation,
                                          self.propPoseBodyList[i],
                                          self.propVisList[i])



vizualizer = uavDynamicVisualizer()
vizualizer.startVisualization()
