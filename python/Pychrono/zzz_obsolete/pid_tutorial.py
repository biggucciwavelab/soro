# -*- coding: utf-8 -*-
"""
Created on Thu Nov 21 13:10:14 2019

@author: Roshan
"""
from pid_controller.pid import PID
import numpy as np

class PidControl:
    def __init__(self, nb, mag, tar=[], XdirForce=[], YdirForce=[],ZdirForce=[],XposBody=[],YposBody=[],ZposBody=[],Kp =10, Ki=.1, Kd=0):
        self.nb = nb
        self.mag = mag
        self.tar = tar
        self.XdirForce = XdirForce
        self.YdirForce = YdirForce
        self.ZdirForce = ZdirForce
        self.XposBody = XposBody
        self.YposBody = YposBody
        self.ZposBody = ZposBody
        self.angle_threshold = 2 
        self.pos_threshold  = 0.1
        # Initialize the controllers
        self.pid = PID(p=Kp, i=Ki, d=Kd)
     
        self.Tlocation = [self.tar[0],self.tar[1],self.tar[2]]
        self.dirF = [self.XdirForce[1],self.YdirForce[1],self.YdirForce[1]]
    
        self.XcenPos = sum(self.XposBody)/self.nb
        self.YcenPos = sum(self.YposBody)/self.nb
        self.ZcenPos = sum(self.ZposBody)/self.nb
        
        self.cur_angle = self.start_control_loop()
        self.intarget = self.reaching_target()
        
    def set_curangle(self):
        self.cur_angle = np.arctan(self.dirF[2]/self.dirF[0])
        self.cur_angle = np.degrees(self.cur_angle)
        return self.cur_angle
        
            
    def set_target(self):
        Zaxis = self.Tlocation[2] - self.ZcenPos
        Xaxis = self.Tlocation[0] - self.XcenPos
        tar_angle = np.degrees(np.arctan((Zaxis/Xaxis)))
        
        if Zaxis > 0.01:    
            if -0.01<= Xaxis <= 0.01:
               return 90
            if Xaxis < -0.01:
               return 180 + tar_angle
            if Xaxis > 0.01:
               return tar_angle
           
        if Zaxis < -0.01:
            if -0.01 <= Xaxis <= 0.01:
               return -90
            if Xaxis > 0.01:
               return tar_angle
            if Xaxis < -0.01:
               return tar_angle-180
           
        if -0.01 <= Zaxis <= 0.01:
            if -0.01 <= Xaxis <= 0.01:
               return 0
            if Xaxis < -0.01:
               return 180
            if Xaxis > 0.01:
               return 0  
                 
  
    def start_control_loop(self):
        while True:
            tar_angle = self.set_target()
            self.pid.target = tar_angle
            self.cur_angle = self.set_curangle()
            while True:
                self.pid(feedback=self.cur_angle)
                error = self.pid.error
                angles_are_set = (abs(error) < self.angle_threshold)  # Verify if every angles are within the tolerances
                if angles_are_set:
                    return self.cur_angle
                    break
                else:
                    if tar_angle >= 0:
                        self.cur_angle = self.cur_angle + 2   
                    
                    else:
                        self.cur_angle = self.cur_angle - 2
                        
                        
    def reaching_target(self):
        dis = np.zeros(self.nb)
        pos = np.zeros(self.nb)
        for i in range(self.nb):
            dis[i] = np.sqrt((self.XposBody[i]-self.tar[0])**2 + (self.ZposBody[i]-self.tar[2])**2)
            pos[i] = dis[i] < self.pos_threshold
            
        if True in pos:
            return 0
        else: 
            return self.mag
        
        
        
              

        
        
        
    
        
        