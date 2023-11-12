from matplotlib import pyplot as plt
import numpy as np

class DynamicModel():
    def __init__(self,AnglePos,AngleVel,AngleAcc,l,I,Kthrust,Thrust):
        self.anglePosition = AnglePos
        self.angleVelocity= AngleVel
        self.angleAcceleration= AngleAcc
        self.l = l
        self.I = I
        self.KThrust = Kthrust
        self.thrust = Thrust
        self.velocity1 = 0
        self.velocity2 = 0
        self.M = 0
        
    def handleControlMove(self,u):
        self.velocity1 = u+self.thrust
        self.velocity2 = -u+self.thrust
        print(self.velocity1,self.velocity2)
        self.M = self.KThrust * self.l/2 * (self.velocity1**2 - self.velocity2**2)
        self.angleAcceleration = self.M/self.I
    def integrate(self,dt):
        self.angleVelocity+= self.angleAcceleration*dt
        self.anglePosition+= self.angleVelocity *dt
    def calculateAnglePosition(self,u,dt):
        self.handleControlMove(u)
        self.integrate(dt)
    def getAngleVelocity(self):
        return self.angleVelocity
    def getAnglePosition(self):
        return self.anglePosition
    def getAngleAcceleration(self):
        return self.angleAcceleration


class Controller():
    def __init__(self,kP,kI,kD,ControlLimitMin,ControlLimitMax):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.controlLimitMax = ControlLimitMax
        self.controlLimitMin = ControlLimitMin
        self.error = 0
        self.errorPast = 0
        self.integral = 0
        self.destValue = 0

    def setDestValue(self,destVal):
        self.destValue = destVal

    def PID(self,currentValue,dt):
        self.error = self.destValue - currentValue
        
        self.integral += self.error *dt
        u = self.kP*self.error + self.kI*self.integral+self.kD *((self.error - self.errorPast)/dt)
        
        self.errorPast = self.error
        u = self.saturation(u)
        return u
    
    def saturation(self,value):
        if(value > self.controlLimitMax):
            value = self.controlLimitMax
        elif(value < self.controlLimitMin):
            value = self.controlLimitMin
        return value

class Simulator():
    def __init__(self,dt,T,model,controllerPosition,controllerVelocity):
        self.Tend = T
        self.dt = dt
        self.AnglePositionList = []
        self.AngleVelocityList = []
        self.AngleAccelerationList = []
        self.model = model
        self.controllerPosition = controllerPosition
        self.controllerVelocity = controllerVelocity
    def runSimulate(self):
        tstart = 0 
        while(tstart <= self.Tend):
            AngleVel = self.model.getAngleVelocity()
            AnglePos = self.model.getAnglePosition()
            AngleAcc = self.model.getAngleAcceleration()
            self.AngleVelocityList.append(AngleVel)
            self.AnglePositionList.append(AnglePos)
            self.AngleAccelerationList.append(AngleAcc)
            uVelocity = self.controllerPosition.PID(AnglePos,self.dt)
            print(uVelocity)
            controllerVelocity.setDestValue(uVelocity)
            uAcceleration = self.controllerVelocity.PID(AngleVel,self.dt)

            self.model.calculateAnglePosition(uAcceleration,self.dt)
            tstart+=self.dt
    def showPlots(self):
        fig = plt.figure(constrained_layout=True)
        gs = fig.add_gridspec(3, 5)
        positionPlot = fig.add_subplot(gs[0, :-1])
        positionPlot.plot(self.AnglePositionList)
        positionPlot.grid()
        positionPlot.set_title('position')

        velocityPlot = fig.add_subplot(gs[1, :-1])
        velocityPlot.plot(self.AngleVelocityList, "g")
        velocityPlot.grid()
        velocityPlot.set_title('velocity')

        accelerationPlot = fig.add_subplot(gs[2, :-1])
        accelerationPlot.plot(self.AngleAccelerationList, "r")
        accelerationPlot.grid()
        accelerationPlot.set_title('acceleration')
        plt.show()
            
            
            


#Начальные условия
AnglePositionCMD = 0.52
AnglePositionInit = 0
AngleVelocityInit= 0

#Параметры времени моделирования
dt = 0.01
T = 10
thrust = 10
kPVelocity = 250
kIVelocity = 20
kDVelocity = 1
kPPosition = 1200
kIPosition = 5
kDPosition = 210
l= 0.17
#Параметры модели
l = 0.17
I = 7.16914e-05
kThrust = 3.9865e-08
controllerVelocity = Controller(kPVelocity,kIVelocity,kDVelocity,-3000,3000)
controllerPosition = Controller(kPPosition,kIPosition,kDPosition,-1000,1000)
controllerPosition.setDestValue(AnglePositionCMD)
model = DynamicModel(AnglePositionInit,AngleVelocityInit,0,l,I,kThrust,thrust)
simulator = Simulator(dt,T,model,controllerPosition,controllerVelocity)
simulator.runSimulate()
simulator.showPlots()