from enum import IntEnum, unique
import numpy as np

@unique
class States(IntEnum):
    X = 0
    Y = 1
    Z = 2
    ROLL =3  
    PITCH = 4
    YAW = 5
    VX = 6
    VY = 7
    VZ =8
    ROLL_RATE = 9
    PITCH_RATE = 10
    YAW_RATE = 11

l = 0.17    # длина луча
kThrust = 3.9865e-8 #коэффициент тяги
kDrag = 7.5e-10   #коэффициент сопротивления

rotorsRpmMax = 5000  #ограничение по максимальной тяге роторов
rotorsRpmMin = 200   #ограничение по минимальной тяге роторов

kT = 1000     #коэффициент умножения управления по тяге
uavMass = 0.0630  #масса аппарата
uavI = np.array([[5.82857e-4,0.,0.],    #тензор инерции, осевые состовляющие
                [0.,7.16914e-4,0.],          
                [0.,0.,0.0001]])
dt= 0.01  