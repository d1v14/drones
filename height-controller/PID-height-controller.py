from matplotlib import pyplot as plt

class DroneModel():
    #Инициализация конструктора класса, который принимает на вход данные о БЛА.
    #Mass - масса БЛА, velocityInit - начальная скорость, accelerationInit - начальное ускорение
    #Kthrust - коэффициент тяги роторов, rotorCount - количество роторов, positionInit - начаальная позиция
    def __init__(self, mass,velocityInit, accelerationInit, Kthrust,rotorCount,positionInit):
        self.mass = mass
        self.velocity = velocityInit
        self.acceleration = accelerationInit
        self.Kthrust = Kthrust
        self.rotorCount = rotorCount
        self.position = positionInit
        self.g = 9.81
    #Метод, реализующий вычисление значения ускорения по управляющему действию u.
    def calculateControlMove(self,u):
        #Вычисляем суммарное значение квадратов тяги роторов
        thrustMove = self.rotorCount * (u**2)
        #Вычисляем ускорение по первому закону динамики. ma = sum(Fi)
        self.acceleration = (self.Kthrust * thrustMove)/self.mass - self.g
    #Метод, реализующий интегрирование полученного ускорения БЛА для определения положения
    def integrateValues(self,dt):
        self.velocity += self.acceleration * dt
        self.position +=  self.velocity*dt
    #Метод, вычисляющий текущее положение БЛА, принимающий управляющее воздействие и вызывающий методы вычисления 
    def calculatePosition(self,u,dt):
        self.calculateControlMove(u)
        self.integrateValues(dt)
    
    #Методы возвращающие значения 
    def getPosition(self):
        return self.position
    def getVelocity(self):
        return self.velocity
    def getAcceleration(self):
        return self.acceleration
#Контроллер
class ControlSystem():
    def __init__(self,kP,kI,kD,controlLimit):
        self.kI = kI
        self.kP = kP
        self.kD = kD
        self.integral = 0
        self.desPosition = 0
        self.error = 0
        self.errorPast = 0
        self.controlLimit = controlLimit
    #Метод позволяющий установить значение точки, которую нужно удерживать
    def setDesiredPosition(self,position):
        self.desPosition = position
    #Метод реализующий функционал ПИД-контроллера. Высчитывает необходимое управление исходя из ошибки 
    def PID(self,currPosition,dt):
        self.error = self.desPosition - currPosition
        self.integral +=self.error*dt
        u = self.kP*self.error + self.kI * self.integral + self.kD * ((self.error - self.errorPast)/dt)
        u = self.saturation(u)
        self.errorPast = self.error
        return u
    #Метод, реализующий функцию насыщения, которая ограничивает выходное значение скорости
    def saturation(self, velocity):
        if velocity > self.controlLimit:
            velocity = self.controlLimit
        else:
            if velocity < -self.controlLimit:
                velocity = -self.controlLimit
        return velocity
#Класс, описывающий симуляцию 
class Simulator():
    def __init__(self,Tend,dt,controller,model):
        self.Tend = Tend
        self.dt =dt
        self.controlSystem = controller
        self.droneModel = model
        self.accelerationList = []
        self.velocityList = []
        self.positionList = []
        self.timeList = []
    #Метод, запускающий симуляцию
    def runSimulation(self):
        startTime = 0.00
        while(startTime <=self.Tend):
            #Получаем значения скорости ускорения и позиции с нашего ЛА
            velocity = self.droneModel.getVelocity()
            acceleration = self.droneModel.getAcceleration()
            position = self.droneModel.getPosition()
            #Заполняем массивы для построения графиков\
            self.accelerationList.append(acceleration)
            self.velocityList.append(velocity)
            self.positionList.append(position)
            #Высчитываем управление 
            u = self.controlSystem.PID(position,self.dt)
            #Персчитываем значение позиции ЛА
            self.droneModel.calculatePosition(u,self.dt)
            startTime+=self.dt
    def simulationPlots(self):

        fig = plt.figure(constrained_layout=True)
        gs = fig.add_gridspec(3, 5)
        positionPlot = fig.add_subplot(gs[0, :-1])
        positionPlot.plot(self.positionList)
        positionPlot.grid()
        positionPlot.set_title('position')

        velocityPlot = fig.add_subplot(gs[1, :-1])
        velocityPlot.plot(self.velocityList, "g")
        velocityPlot.grid()
        velocityPlot.set_title('velocity')

        accelerationPlot = fig.add_subplot(gs[2, :-1])
        accelerationPlot.plot(self.accelerationList, "r")
        accelerationPlot.grid()
        accelerationPlot.set_title('acceleration')
        plt.show()
#Задаеи коэффициенты регулятора
kP = 300
kD = 180
kI = 35
#Время выполнения
tEnd = 20
dt = 0.01
#Данные о дроне
droneMass = 0.006
velocityInit = 0
accelerationInit = 0
positionInit = 0
rotorCount = 4
kThrust = 3.9865e-08
speedLimit = 1000
#Создаем объект класса контроллер, который будет реализовывать ПИД
controller = ControlSystem(kP,kI,kD,speedLimit)
controller.setDesiredPosition(10)
#Создаем объект, в котором будут храниться данные о дроне
droneModel = DroneModel(droneMass,velocityInit,accelerationInit,kThrust,rotorCount,positionInit)
#Создаем и запускаем симуляцию
simulator = Simulator(tEnd,dt,controller,droneModel)
simulator.runSimulation()
simulator.simulationPlots()