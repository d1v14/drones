import numpy as np
class QuadrocopterModel:
    def __init__(self,inertion,mass,kT,kD,l): 
        #Инициализация тензора инерции
        self.inertion = inertion 
        #Вычисление обратной матрицы тензора инерции
        self.inertionInv = np.linalg.inv(inertion)
        #Инициализация массы, коэффициента тяги, сопротивления, длины луча
        self.mass = mass
        self.kT = kT
        self.kD = kD
        self.l = l
        #Вектор, описывающий воздействие силы тяги
        self.g = np.array([[0.0],[0.0],[-9.81]])
        #Вектор тяги двигателей
        self.rotorThrust = np.array([[0.0],[0.0],[0.0]])
        #Вектор моментов по осям
        self.rotorMoments = np.array([[0.0],[0.0],[0.0]])
        #Вектор состония БЛА
        self.state = np.array([[0.0],#X
                               [0.0],#Y
                               [0.0],#Z
                               [0.0],#roll
                               [0.0],#pitch
                               [0.0],#yaw
                               [0.0],#velocity X
                               [0.0],#velocity Y
                               [0.0],#velocity Z
                               [0.0],#roll rate
                               [0.0],#pitch rate
                               [0.0],#yaw rate
                               ])
    #Вектор обработки входящего управляющего воздействия для обновления состояния БЛА
    def calculateCommand(self,cmd):
        #Высчитываем суммарную тягу роторов
        self.rotorThrust[2] = (cmd**2 *self.kT).sum()
        #Высчитываем матрицу поворота по всем углам БЛА
        R = self.RotationMatrix3d(self.state[3][0],self.state[4][0],self.state[5][0])
        R.resize((3,3))
        #Высчитываем поступательное движение
        self.linearAcсeleration = R.transpose()@self.rotorThrust / self.mass +self.g  
        #Высчитываем моменты двигателей по всем осям
        self.rotorMoments[0] = self.l*self.kT * (cmd[0]**2 - cmd[2]**2)
        self.rotorMoments[1] = self.l*self.kT * (cmd[3]**2 - cmd[1]**2)
        self.rotorMoments[2] = self.kD * (cmd[3]**2 +cmd[1]**2 - cmd[0]**2 - cmd[2]**2)
        #Высчитываем вращательное движение
        self.angularAcceleration = self.inertionInv @ (self.rotorMoments-np.cross((self.state[9:12]),(self.inertion@self.state[9:12]),axis=0))
    #Метод, формирующий матрицу поворота по углам БЛА
    def RotationMatrix3d(self,roll,pitch,yaw):
            R = np.array([[np.cos(yaw)*np.cos(roll),np.sin(yaw)*np.cos(roll),-np.sin(roll)],
                          [np.cos(yaw)*np.sin(pitch)*np.sin(roll)-np.sin(yaw)*np.cos(pitch),
                          np.sin(yaw)*np.sin(pitch)*np.sin(roll)+np.cos(yaw)*np.cos(pitch),
                          np.sin(pitch)*np.cos(roll)],
                          [np.cos(yaw)*np.sin(pitch)*np.cos(roll)+np.sin(yaw)*np.sin(roll),
                          np.sin(yaw)*np.cos(roll)*np.sin(pitch)-np.cos(yaw)*np.sin(roll),
                          np.cos(pitch)*np.cos(roll)]])
            return R
    #Метод, дважды итегрирующий угловое ускорение и линейное ускорение для получения линейных скоростей, линейного положения, угловой скорости, углового положения
    def integrate(self,dt):
        self.state[9:12] += self.angularAcceleration *dt
        self.state[3:6] += self.state[9:12] * dt
        self.state[6:9] += self.linearAcсeleration *dt
        self.state[0:3] += self.state[6:9] * dt
    #Метод, обновляющий вектор состояния БЛА
    def updateState(self,cmd,dt):
        self.calculateCommand(cmd)
        self.integrate(dt)
    #Метод, возвращающий вектор состония БЛА
    def getState(self):
        return self.state