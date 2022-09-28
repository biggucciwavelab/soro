# -*- coding: utf-8 -*-
"""
Created on Fri Feb  7 12:47:22 2020

@author: dmulr
"""


runs=20


for i in range(runs):
    u=fin(l,ub,ul,P,Ac,k,h,i)

    np.savez("U"+str(i)+".npz",allow_pickle=True,u=u)
    
    
i=1
data=np.load("U"+str(i)+".npz",allow_pickle=True)

U1=data["u"]