# In[Header]
"""
author: declan mulroy
project: JAMoEBA
email: dmulroy@hawk.iit.edu
date: 12/4/19
Max values
"""

import os
import numpy as np

def MaxValues(R1,diameter,nb):
    Rin=R1
    ngrans1=int(Rin/(diameter))

    ri=np.zeros((1,ngrans1))
    ni=np.zeros((1,ngrans1))
    
    radii=Rin-(diameter/2)
    for i in range(ngrans1):
        remainder=((diameter))*i
        ri[:,i]=radii-remainder
        ni[:,i]=np.floor((ri[:,i]*np.pi*2)/diameter)

    ni=np.asarray(ni,dtype=int)

    return(ni)

nb=np.array([5,10,15,20,25,30,35,40,45,50,55,60,65,70,75])
diameter=.07
diameter2=.07
ni=[]
for i in (nb): 
    R1=(diameter*i/(np.pi*2))+.1
    ni.append(MaxValues(R1,diameter,nb))


for i in (ni[3]):

    count=i+1
    print(count)