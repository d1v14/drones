import constants as cs
from controller import UAVcontroller
from mathModel import QuadrocopterModel
from simulator import Simulator
import numpy as np  
#Метод для высчитывания оптимальных коэффициентов ПИД-регулятора (не использовался из-за высокой сложности, остается на доработку)
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


#Объект класса UAVcontroller, отвечающий за регулирование параметров БЛА
controller = UAVcontroller()

#Устанавливаем коэффициенты ПИД-регуляторов для поступательного движения
#Устанавливаем коэффициенты для ПИД-регуляторов, отвечающих за управление положение БЛА. Выходной сигнал - сигнал по управлению линейной скоростью БЛА, ограничиваем скорость в 
# пределе [-15,15] метров в секунду
controller.xLinearPosPID.setPidCoeff(1,0,1)
controller.xLinearPosPID.setCommandLimit(-15,15)

controller.yLinearPosPID.setPidCoeff(1,0,1)
controller.yLinearPosPID.setCommandLimit(-15,15)

controller.zLinearPosPID.setPidCoeff(50,0.9,40)
controller.zLinearPosPID.setCommandLimit(-15,15)


#Устанавливаем коэффициенты для ПИД-регуляторов, отвечающих за управление линейной скоростью БЛА в каналах XY. Выходной сигнал - сигнал по управлению угловым положением БЛА,
#ограничиваем угол в пределе [-pi/3,pi/3] радиан
controller.xLinearVelocityPID.setPidCoeff(0.1,0,0)
controller.xLinearVelocityPID.setCommandLimit(-np.pi/3,np.pi/3)

controller.yLinearVelocityPID.setPidCoeff(0.1,0,0)
controller.yLinearVelocityPID.setCommandLimit(-np.pi/3,np.pi/3)

#Устанавливаем коэффициенты ПИД-регуляторов для вращательного движения БЛА
#Устанавливаем коэффициенты для ПИД-регуляторов, отвечающих за управление линейной скоростью БЛА в канале Z. Выходной сигнал - сигнал по управлению тягой БЛА,
#ограничиваем тягу в пределе [-5000,5000] радиан
controller.ThrustPID.setPidCoeff(0.4,0,0)
controller.ThrustPID.setCommandLimit(-5000,5000)


#Устанавливаем коэффициенты для ПИД-регуляторов, отвечающих за управление угловым положение БЛА в каналах Roll Pitch Yaw. Выходной сигнал - сигнал по управлению угловой скоростью БЛА,
#ограничиваем угловую скорость в пределе [-60,60] радиан/c для Roll и Pitch, [-120,120] рад/c для Yaw
controller.pitchAnglePosPID.setPidCoeff(16,0,0)
controller.pitchAnglePosPID.setCommandLimit(-60,60)


controller.rollAnglePosPID.setPidCoeff(16,0,0)
controller.rollAnglePosPID.setCommandLimit(-60,60)

controller.yawAnglePosPID.setPidCoeff(20,0,0)
controller.yawAnglePosPID.setCommandLimit(-120,120)


#Устанавливаем коэффициенты для ПИД-регуляторов, отвечающих за управление угловой скоростью БЛА в каналах Roll Pitch YAw. Выходной сигнал - угловое ускорение,
#ограничиваем угловое ускорение в пределе [-5000,5000] радиан/c^2 для Roll и Pitch, [-15000,15000] рад/c^2 для Yaw, чтобы данная команда могла пересилить Roll и Yaw в миксере 
controller.pitchRatePID.setPidCoeff(300,0,0)
controller.pitchRatePID.setCommandLimit(-5000,5000)

controller.rollRatePID.setPidCoeff(300,0,0)
controller.rollRatePID.setCommandLimit(-5000,5000)

controller.yawRatePID.setPidCoeff(800,0,0)
controller.yawRatePID.setCommandLimit(-15000,15000)

#Создаем математическую модель дрона
model = QuadrocopterModel(cs.uavI,cs.uavMass,cs.kThrust,cs.kDrag,cs.l)
#Создаем объект для запуска симуляции
simulator = Simulator(controller,model,cs.dt)
#В контроллер полета загружаем миссию из массива точек
controller.setMission([[0,0,10,3],[0,0,10,1],[5,1,7,1],[6,6,6,0.5],[6,5,10,2],[3,0,10,-1],[1,1,1,0],[10,10,1,0],[5,2,5,1]])
#Запускаем симуляцию
simulator.run()

