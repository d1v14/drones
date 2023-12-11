from enum import IntEnum, unique
import numpy as np

@unique
class FlightMode(IntEnum):
    PositionHold = 1
    VelocityHold = 2
    AngleGold = 3
    Acro = 4

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

l = 0.17
kThrust = 3.9865e-8
kDrag = 7.5e-7

rotorsRpmMax = 5000
rotorsRpmMin = 200

kT = 1000

uavMass = 0.0630
uavI = np.array([[5.82857e-4,0.,0.],
                [0.,7.16914e-4,0.],
                [0.,0.,0.01]])
dt= 0.01