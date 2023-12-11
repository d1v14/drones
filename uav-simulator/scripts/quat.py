#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple quaternion operation library
@author: RegisLab
"""
import numpy as np



class TQuat():
    
    def __init__(self,rotate_angle=None,vector=np.array([None,None,None])):
        '''
        

        Parameters
        ----------
        rotate_angle : float
            DESCRIPTION.
        vector : numpy.ndarray
            DESCRIPTION.

        Returns
        -------
        вектор должен быть нормализован его длинна должна быть равна 1
        
        w – компонента, описывающая поворот (косинус половины угла).

        '''
        vector = self.normalize_vector(vector)
        self.w = np.cos(rotate_angle/2)
        self.x = vector[0] * np.sin(rotate_angle/2)
        self.y = vector[1] * np.sin(rotate_angle/2)
        self.z = vector[2] * np.sin(rotate_angle/2)
        
    def show(self):
        print('w = ',self.w,'\n','x = ',self.x,'\n','y = ',self.y,'\n','z = ',self.z,'\n')
    
    @staticmethod
    def normalize_vector(vector):
        '''
        

        Parameters
        ----------
        vector : numpy.ndarray
            DESCRIPTION.

        Returns
        -------
        vector : numpy.ndarray
            DESCRIPTION.

        '''
        lenth = np.sqrt(vector[0]**2+vector[1]**2+vector[2]**2)
        if lenth >0:      
            vector = vector / lenth
        
        return vector
    
    @staticmethod
    def get_quat_lenth(quat):
        '''
        

        Parameters
        ----------
        qwat : TQuat
            обьект экземпляр класса Tquat

        Returns
        -------
        qwat_lenth : float
            длинна кватерниона

        '''
        
        quat_lenth = np.sqrt(quat.w**2+quat.x**2+quat.y**2+quat.z**2)
        
        return quat_lenth
    
    @staticmethod
    def quat_mul_scale(quat,scale):
        '''
        

        Parameters
        ----------
        quat : TQuat
            DESCRIPTION.
        scale : float
            DESCRIPTION.

        Returns
        -------
        quat : TQuat
            DESCRIPTION.
        умножение кватерниона на скаляр
        '''
        result = TQuat(0,np.array([0,0,0]))
        
        result.w = quat.w * scale
        result.x = quat.x * scale
        result.y = quat.y * scale
        result.z = quat.z * scale
        
        return result
    @staticmethod
    def quat_normalize(quat):
        result = TQuat(0,np.array([0,0,0]))
        lenth = TQuat.get_quat_lenth(quat)
        
        result = TQuat.quat_mul_scale(quat,lenth)
        
        return result
    
    @staticmethod
    def quat_invert(quat):
        
        res = TQuat(0,np.array([0,0,0]))
        res.w = quat.w
        res.x = -quat.x
        res.y = -quat.y
        res.z = -quat.z
        
        #invert_quat = TQuat.quat_normalize(res)
        
        return res
    
    @staticmethod
    def quat_multiplication(a,b):
        '''
        

        Parameters
        ----------
        a : TQuat
            DESCRIPTION.
        b : TQuat
            DESCRIPTION.

        Returns результат умножения кватернионов а именно кватернион
        -------
        result : TQuat
            умножение кватернионов

        '''
        result = TQuat(0,np.array([0,0,0]))
        
        result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
        result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
        result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
        
        return result
    
    @staticmethod
    def quat_mul_vector(quat,vector):
        '''
        

        Parameters
        ----------
        quat : TYPE
            DESCRIPTION.
        vector : TYPE
            DESCRIPTION.

        Returns
        -------
        result : TYPE
            DESCRIPTION.
            умножение кватерниона на вектор
            
            для умножения вектора на кватернион необходимо привести вектор
            к виду кватерниона приняв параметр w = 0, таким образом получаются
            следующие выражения
        '''
        result = TQuat(0,np.array([0,0,0]))
        
        result.w = -quat.x * vector[0] - quat.y * vector[1] - quat.z * vector[2]
        result.x = quat.w * vector[0] + quat.y * vector[2] - quat.z * vector[1]
        result.y = quat.w * vector[1] - quat.x * vector[2] + quat.z * vector[0]
        result.z = quat.w * vector[2] + quat.x * vector[1] - quat.y * vector[0]
        
        return result
    
    @staticmethod 
    def quat_transform_vector(quat,vector):
        
        t = TQuat(0,np.array([0,0,0]))
        
        t = TQuat.quat_mul_vector(quat,vector) 
        t = TQuat.quat_multiplication(t, TQuat.quat_invert(quat))
        
        result = np.array([t.x,t.y,t.z])
        
        return result
    
    @staticmethod
    def rotate_vector_angles(roll = 0,pitch=0,yaw=0,vector= np.array([0,1,0])):
        
        q_2 = TQuat.euler_to_quat(roll,pitch,yaw)
        
        result = TQuat.quat_transform_vector(q_2, vector)
        
        return result
    
    @staticmethod
    def euler_to_quat(roll = 0,pitch=0,yaw=0):
        rot_yaw = TQuat(yaw,np.array([0,0,1]))
       
        rot_pitch = TQuat(pitch,np.array([0,1,0]))
     
        rot_roll = TQuat(roll,np.array([1,0,0]))
        
        q_1 = TQuat.quat_multiplication(rot_yaw, rot_pitch)
        q_rot = TQuat.quat_multiplication(q_1, rot_roll)
        
        return q_rot
    
    @staticmethod
    def quat_to_euler(quat):
        roll = np.arctan2(2*(quat.w*quat.x+quat.y*quat.z),
                          1-2*(quat.x**2+quat.y**2))
        
        sinp = 2 * (quat.w * quat.y - quat.z * quat.x)
        if(abs(sinp)>=1):
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        yaw = np.arctan2(2 * (quat.w * quat.z + quat.x * quat.y),
                         1 - 2 * (quat.y * quat.y + quat.z * quat.z))
        
        return roll , pitch ,yaw
