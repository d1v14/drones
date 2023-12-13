import numpy as np
import constants as cs
from constants import States
class PID():
    def __init__(self):
        self.kp = 0
        self.ki=0
        self.kd = 0
        self.integral = 0
        self.lastError = 0
        self.error = 0
        self.integralLimit = 0
        self.minVal = 0
        self.maxVal = 0
    def setPidCoeff(self,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
    def setCommandLimit(self,minVal,maxVal):
        self.minVal = minVal
        self.maxVal = maxVal
    def setIntgralLimit(self,limit):
        self.integralLimit = limit
    def update(self,desValue,currValue,dt):
        self.error = desValue - currValue
        self.integral += self.error *dt

        u = self.kp*self.error +self.ki*self.integral + self.kd*((self.error - self.lastError)/dt)
        self.lastError = self.error
        u = self.saturation(u,self.minVal,self.maxVal)
        return u
    def getError(self):
        return self.error
    def saturation(self,value,minVal,maxVal):
        if value < minVal:
            return np.array([minVal])
        else:
            if value>maxVal:
                return np.array([maxVal])
        return value
    def saturationI(self,value,minVal,maxVal):
        if value <0.0:
            return 0
        return value

class UAVcontroller():
    def __init__(self):
        self.XDest = 0
        self.YDest =5
        self.ZDest = 10
        self.yawDest = np.pi/3
        self.missionIndex=0
        self.missionCompelete=0
        self.mission = list()
        self.u = np.array([[0.0],[0.0],[0.0],[0.0]])
        
        self.xLinearPosPID = PID()
        self.yLinearPosPID = PID()
        self.zLinearPosPID = PID()
        self.xLinearVelocityPID = PID()
        self.yLinearVelocityPID = PID()
        self.ThrustPID = PID()
        self.rollAnglePosPID = PID()
        self.pitchAnglePosPID = PID()
        self.yawAnglePosPID = PID()
        self.rollRatePID = PID()
        self.pitchRatePID = PID()
        self.yawRatePID = PID()
    def setDestPosition(self,x,y,z,yaw):
        self.XDest = x
        self.YDest = y
        self.ZDest = z
        self.yawDest = yaw
    def setMission(self,mission):
        self.mission = mission
        self.missionIndex = 0
        self.updateMission()

    def updateMission(self):
        self.XDest = self.mission[self.missionIndex][0]
        self.YDest = self.mission[self.missionIndex][1]
        self.ZDest = self.mission[self.missionIndex][2]
        self.yawDest = self.mission[self.missionIndex][3]

    
    def checkMission(self,state):
        print(self.missionIndex)
        if(abs(state[0][0] - self.XDest) <0.3 and
           abs(state[1][0] - self.YDest) <0.3 and
           abs(state[2][0] - self.ZDest) <0.3 and 
           self.missionIndex < len(self.mission)-1):
            self.missionIndex +=1
            self.updateMission()
        elif (abs(state[0][0] - self.XDest) <0.3 and
           abs(state[1][0] - self.YDest) <0.3 and
           abs(state[2][0] - self.ZDest) <0.3 and 
           self.missionIndex == len(self.mission)-1):
            self.missionCompelete = 1

    def update(self,state,dt):
        LinearVelocityXCommand = self.xLinearPosPID.update(self.XDest,state[States.X],dt)
        LinearVelocityYCommand = self.yLinearPosPID.update(self.YDest,state[States.Y],dt)
        LinearVelocityZCOmmand = self.zLinearPosPID.update(self.ZDest,state[States.Z],dt)
        PitchAngleCommand = self.xLinearVelocityPID.update(LinearVelocityXCommand,state[States.VX],dt)
        RollAngleCommand = self.yLinearVelocityPID.update(LinearVelocityYCommand,state[States.VY],dt)
        ThrustCommand = self.ThrustPID.update(LinearVelocityZCOmmand,state[States.VZ],dt)
        ThrustCommand *= cs.kT
        ThrustCommand = self.ThrustPID.saturation(ThrustCommand,cs.rotorsRpmMin,cs.rotorsRpmMax)
        targetPitchRoll = self.rotationMatrix2d(state[States.YAW][0]).transpose()@ np.array([[PitchAngleCommand][0],[RollAngleCommand][0]])
        RollCommand = targetPitchRoll[1]
        PitchCommand = targetPitchRoll[0]
        RollRateCommand = self.rollAnglePosPID.update(-RollCommand,state[States.ROLL],dt)
        PitchRateCommand = self.pitchAnglePosPID.update(PitchCommand,state[States.PITCH ],dt)
        YawRateCommand = self.yawAnglePosPID.update(self.yawDest,state[States.YAW],dt)
        rollAccCommand = self.rollRatePID.update(RollRateCommand,state[States.ROLL_RATE],dt)
        pitchAccCommand = self.pitchRatePID.update(PitchRateCommand,state[States.PITCH_RATE],dt)
        yawAccCommand = self.yawRatePID.update(YawRateCommand,state[States.YAW_RATE],dt)
        self.mixer(rollAccCommand,pitchAccCommand,yawAccCommand,ThrustCommand)
        self.u [0]= self.ThrustPID.saturation(self.u[0],cs.rotorsRpmMin,cs.rotorsRpmMax)
        self.u[1] = self.ThrustPID.saturation(self.u[1],cs.rotorsRpmMin,cs.rotorsRpmMax)
        self.u[2] = self.ThrustPID.saturation(self.u[2],cs.rotorsRpmMin,cs.rotorsRpmMax)
        self.u[3]= self.ThrustPID.saturation(self.u[3],cs.rotorsRpmMin,cs.rotorsRpmMax)
        self.checkMission(state)
        return self.u
    
    def mixer(self,roll,pitch,yaw,thrust):
        self.u[0] = thrust + roll - yaw
        self.u[1] = thrust - pitch +yaw
        self.u[2] = thrust - roll  - yaw
        self.u[3] = thrust +pitch +yaw 


    def rotationMatrix2d(self,theta):
        return np.array([[np.cos(theta),-np.sin(theta)],
                         [np.sin(theta),np.cos(theta)]])
    
    