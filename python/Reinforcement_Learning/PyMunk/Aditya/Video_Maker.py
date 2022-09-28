# -*- coding: utf-8 -*-
"""
Created on Wed Mar  3 13:06:22 2021

@author: adity
"""

import cv2
import numpy as np
import glob


    
img_array = []
for filename in glob.glob('C:\Experiment_158_v1_VideoImages/*.jpg'):
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)
 
 
out = cv2.VideoWriter('Trial_157.avi',cv2.VideoWriter_fourcc(*'DIVX'), 20, size)
 
for i in range(len(img_array)):
    out.write(img_array[i])
out.release()
    
