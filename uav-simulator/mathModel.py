import numpy as np
class QuadrocopterModel:
    def __init__(self,inertion,mass,kT,kD,l):
        self.inertion = inertion
        self.inertionInv = np.linalg.inv(inertion)
        self.mass = mass
        self.kT = kT
        self.kD = kD
        self.l = l
        self.g = np.array([[0.0],[0.0],[-9.81]])
        self.rotorThrust = np.array([[0.0],[0.0],[0.0]])
        self.rotorMoments = np.array([[0.0],[0.0],[0.0]])
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
    def calculateCommand(self,cmd):
        self.rotorThrust[2] = (cmd**2 *self.kT).sum()
        R = self.RotationMatrix3d(self.state[3][0],self.state[4][0],self.state[5][0])
        R.resize((3,3))
        self.linearAcсeleration = R.transpose()@self.rotorThrust / self.mass +self.g  
        #self.linearAcсeleration[0] = 0
        #self.linearAcсeleration[1] = 0
        #self.linearAcсeleration[2] = 0 
        self.rotorMoments[0] = self.l*self.kT * (cmd[0]**2 - cmd[2]**2)
        self.rotorMoments[1] = self.l*self.kT * (cmd[3]**2 - cmd[1]**2)
        self.rotorMoments[2] = self.kD * (cmd[3]**2 +cmd[1]**2 - cmd[0]**2 - cmd[2]**2)
        self.angularAcceleration = self.inertionInv @ (self.rotorMoments-np.cross((self.state[9:12]),(self.inertion@self.state[9:12]),axis=0))
        #self.angularAcceleration[0] = 0
        #self.angularAcceleration[1] = 0
        self.angularAcceleration[2] = 0
    def RotationMatrix3d(self,roll,pitch,yaw):
            # RRoll = np.array([[1,0,0],[0,np.cos(roll),np.sin(roll)],[0,-np.sin(roll),np.cos(roll)]])
            # RPitch = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
            # RYaw = np.array([[np.cos(yaw),np.sin(yaw),0],[-np.sin(yaw),np.cos(yaw),0],[0,0,1]])
            # R = RRoll @ RPitch @ RYaw
            R = np.array([[np.cos(yaw)*np.cos(roll),np.sin(yaw)*np.cos(roll),-np.sin(roll)],
                          [np.cos(yaw)*np.sin(pitch)*np.sin(roll)-np.sin(yaw)*np.cos(pitch),
                          np.sin(yaw)*np.sin(pitch)*np.sin(roll)+np.cos(yaw)*np.cos(pitch),
                          np.sin(pitch)*np.cos(roll)],
                          [np.cos(yaw)*np.sin(pitch)*np.cos(roll)+np.sin(yaw)*np.sin(roll),
                          np.sin(yaw)*np.cos(roll)*np.sin(pitch)+np.cos(yaw)*np.sin(roll),
                          np.cos(pitch)*np.cos(roll)]])
            return R
    
    def integrate(self,dt):
        self.state[9:12] += self.angularAcceleration *dt
        self.state[3:6] += self.state[9:12] * dt
        self.state[6:9] += self.linearAcсeleration *dt
        self.state[0:3] += self.state[6:9] * dt
    def updateState(self,cmd,dt):
        self.calculateCommand(cmd)
        self.integrate(dt)
    def getState(self):
        return self.state