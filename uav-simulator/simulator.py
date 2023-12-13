import socket
import struct
import time as tm
from matplotlib import pyplot as plt
#Класс симулятора БЛА, который осуществляет подключение к UDP порту, для передачи вектора состояния БЛА, 
#осуществляет вычисление управляющего воздействия, по этому управляющему воздействию осуществляет пересчет значений математической модели
class Simulator:
    def __init__(self,controller,model,dt):
        self.controller = controller
        self.model = model
        self.dt = dt 
        self.time = 0 
        #инициализация порта
        self.host = "127.0.0.1"        
        self.port = 12346
        self.addr = (self.host,self.port)
        #инициализация сокета
        self.udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 
        #объявление массивов, необходимых для сохранения значений XYZ,VX,VY,VZ,Roll,Pitch,Yaw,RollRate,PitchRate,YawRate,Time
        self.XPositionList = []
        self.YPositionList = []
        self.ZPositionList = []
        self.VelocityXList = []                      
        self.VelocityYList = []
        self.VelocityZList = []
        self.RollAngleList = []
        self.PitchAngleList =[]
        self.YawAngleList = []
        self.YawAngleVepocityList = []
        self.RollAngleVelocityList = []
        self.PitchAngleVelocityList = []
        self.timeList = []
    #метод, осуществляющий запуск симуляции(вычисление управляющего воздействия через объект класса UAVController на основе 12 ПИД-регуляторов, 
    # пересчет значений БЛА на основе управляющего воздействия) и вывод графиков полета по миссии
    def run(self):
        kS = 0
        end = 0
        while end!=1:
            state = self.model.getState()
            self.XPositionList.append(state[0][0])
            self.YPositionList.append(state[1][0])
            self.ZPositionList.append(state[2][0])
            self.RollAngleList.append(state[3][0])
            self.PitchAngleList.append(state[4][0])
            self.YawAngleList.append(state[5][0])
            self.VelocityXList.append(state[6][0])
            self.VelocityYList.append(state[7][0])
            self.VelocityZList.append(state[8][0])
            self.RollAngleVelocityList.append(state[9][0])
            self.PitchAngleVelocityList.append(state[10][0])
            self.YawAngleVepocityList.append(state[11][0])
            u = self.controller.update(state,self.dt)
            self.model.updateState(u,self.dt)
            self.sendData(state)
            self.time+=self.dt
            self.timeList.append(self.time)
            kS+=1
            end = self.controller.missionCompelete
            tm.sleep(self.dt)
        self.showPlots()
#Вывод графиков, отображающих состояни БЛА во время полета по миссии
    def showPlots(self):
        fig = plt.figure(constrained_layout=True,figsize=(500,500))
        gs = fig.add_gridspec(4, 3)
        positionXPlot = fig.add_subplot(gs[0, 0])
        positionXPlot.plot(self.timeList,self.XPositionList)
        positionXPlot.grid()
        positionXPlot.set_title('X')

        positionYPlot = fig.add_subplot(gs[0, 1])
        positionYPlot.plot(self.timeList,self.YPositionList)
        positionYPlot.grid()
        positionYPlot.set_title('Y')

        positionZPlot = fig.add_subplot(gs[0, 2])
        positionZPlot.plot(self.timeList,self.ZPositionList)
        positionZPlot.grid()
        positionZPlot.set_title('Z')

        rollPlot = fig.add_subplot(gs[1, 1])
        rollPlot.plot(self.timeList,self.RollAngleList)
        rollPlot.grid()
        rollPlot.set_title('Roll')

        pitchPlot = fig.add_subplot(gs[1, 0])
        pitchPlot.plot(self.timeList,self.PitchAngleList)
        pitchPlot.grid()
        pitchPlot.set_title('Pitch')

        yawPlot = fig.add_subplot(gs[1, 2])
        yawPlot.plot(self.timeList,self.YawAngleList)
        yawPlot.grid()
        yawPlot.set_title('YAW')

        velocityXPlot = fig.add_subplot(gs[2, 0])
        velocityXPlot.plot(self.timeList,self.VelocityXList, "g")
        velocityXPlot.grid()
        velocityXPlot.set_title('Velocity X')
        
        velocityYPlot = fig.add_subplot(gs[2, 1])
        velocityYPlot.plot(self.timeList,self.VelocityYList, "g")
        velocityYPlot.grid()
        velocityYPlot.set_title('Velocity Y')
        
        velocityZPlot = fig.add_subplot(gs[2, 2])
        velocityZPlot.plot(self.timeList,self.VelocityZList, "g")
        velocityZPlot.grid()
        velocityZPlot.set_title('Velocity Z')

        AngleVelocityRollPlot = fig.add_subplot(gs[3, 1])
        AngleVelocityRollPlot.plot(self.timeList,self.RollAngleVelocityList, "r")
        AngleVelocityRollPlot.grid()
        AngleVelocityRollPlot.set_title('Angle Velocity Roll')
    
        AngleVelocityPitchPlot = fig.add_subplot(gs[3, 0])
        AngleVelocityPitchPlot.plot(self.timeList,self.PitchAngleVelocityList, "r")
        AngleVelocityPitchPlot.grid()
        AngleVelocityPitchPlot.set_title('Angle Velocity Pitch')

        AngleVelocityYaw = fig.add_subplot(gs[3, 2])
        AngleVelocityYaw.plot(self.timeList,self.YawAngleVepocityList, "r")
        AngleVelocityYaw.grid()
        AngleVelocityYaw.set_title('Angle Velocity Yaw')
        plt.show()

#Упаковка и отправка данных по UDP сокету     
    def sendData(self,state):
        data = bytearray(struct.pack("ddddddddddddd",state[0],state[1],state[2],state[3],state[4],state[5],
                                     state[6],state[7],state[8],state[9],state[10],state[11],self.time))
        self.udp_socket.sendto(data,self.addr)


