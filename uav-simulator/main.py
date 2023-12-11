import constants as cs
from controller import UAVcontroller
from mathModel import QuadrocopterModel
from simulator import Simulator
import numpy as np
def calculateBestKPID(k,model,controller,limit):
    kPDes = 0
    kIDes = 0
    kDDes = 0
    lastSumError = 100000
    controller.ThrustPID.setCommandLimit(-limit,limit)
    for kp in np.arange(0.0,6.0,1):
        for ki in np.arange(0.0,6.0,1):
            for kd in np.arange(0.0,6.0,1):
                print(kp,ki,kd)
                controller.ThrustPID.setPidCoeff(kp,ki,kd)
                sumError = 0
                for i in range(k):
                    state = model.getState()
                    u = controller.update(state,cs.dt)
                    sumError+= abs(controller.ThrustPID.getError())
                    model.updateState(u,cs.dt)
                print(sumError)
                if(sumError<lastSumError):
                    lastSumError = sumError
                    kPDes = kp
                    kIDes = ki
                    kDDes = kd
    return(kPDes,kIDes,kDDes)



controller = UAVcontroller()


controller.xLinearPosPID.setPidCoeff(1,0,1)
controller.xLinearPosPID.setIntgralLimit(0)
controller.xLinearPosPID.setCommandLimit(-15,15)

controller.yLinearPosPID.setPidCoeff(1,0,1)
controller.yLinearPosPID.setIntgralLimit(0)
controller.yLinearPosPID.setCommandLimit(-15,15)

controller.zLinearPosPID.setPidCoeff(50,0.9,40)
controller.zLinearPosPID.setIntgralLimit(0)
controller.zLinearPosPID.setCommandLimit(-15,15)


controller.xLinearVelocityPID.setPidCoeff(0.1,0,0)
controller.xLinearVelocityPID.setIntgralLimit(1)
controller.xLinearVelocityPID.setCommandLimit(-np.pi/3,np.pi/3)

controller.yLinearVelocityPID.setPidCoeff(0.1,0,0)
controller.yLinearVelocityPID.setIntgralLimit(1)
controller.yLinearVelocityPID.setCommandLimit(-np.pi/3,np.pi/3)


controller.ThrustPID.setPidCoeff(0.4,0,0)
controller.ThrustPID.setCommandLimit(-5000,5000)


controller.pitchAnglePosPID.setPidCoeff(16,0,0)
controller.pitchAnglePosPID.setIntgralLimit(0)
controller.pitchAnglePosPID.setCommandLimit(-60,60)


controller.rollAnglePosPID.setPidCoeff(16,0,0)
controller.rollAnglePosPID.setIntgralLimit(0)
controller.rollAnglePosPID.setCommandLimit(-60,60)

controller.yawAnglePosPID.setPidCoeff(60,0,2)
controller.yawAnglePosPID.setIntgralLimit(0)
controller.yawAnglePosPID.setCommandLimit(-60,60)



controller.pitchRatePID.setPidCoeff(300,0,0)
controller.pitchRatePID.setIntgralLimit(1)
controller.pitchRatePID.setCommandLimit(-5000,5000)

controller.rollRatePID.setPidCoeff(300,0,0)
controller.rollRatePID.setIntgralLimit(1)
controller.rollRatePID.setCommandLimit(-5000,5000)

controller.yawRatePID.setPidCoeff(30,0,0)
controller.yawRatePID.setIntgralLimit(0)
controller.yawRatePID.setCommandLimit(-5000,5000)

model = QuadrocopterModel(cs.uavI,cs.uavMass,cs.kThrust,cs.kDrag,cs.l)

simulator = Simulator(controller,model,cs.dt)
# kp,ki,kd = calculateBestKPID(1000,model,controller,5)
# print(kp,ki,kd)
controller.setMission([[0,0,10,0],[5,2,7,0],[2,8,10,0],[3,5,12,0],[6,6,6,0],[6,5,10,0],[3,0,10,0],[1,1,1,0],[10,10,1,0]])
simulator.run()

