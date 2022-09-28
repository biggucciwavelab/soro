# -*- coding: utf-8 -*-
"""
Created on Tue Feb  4 15:20:19 2020

@author: qiyua
"""

import numpy as np
import os


data=np.load('3.npz',allow_pickle=True)
print(data['time'])