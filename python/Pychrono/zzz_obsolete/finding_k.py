

import numpy as np
# In[ making the K1 & K2]
#A = a*b #where a is the thickness of the unit cell and b is the width 


def K1(E,G,A,L):
    k1 = (A*(E-G))/(2*L)
    return k1

def K2(G,A,L):
    k2 = (G*A)/(np.sqrt(2)*L)
    return k2

L = 0.001
A = 0.001*0.001

# In[for aluminium]
#Lets test for the Aluminium material of
E1 = 68.3e9
G1 = 27e9

a1 = K1(E1,G1,A,L)
b1 = K2(G1,A,L)

#putting force
F1 = 30*10**3
#displacement along the spring components
d1 = F1/a1
print(d1)
print("K1 is:", a1,"N/m")
print("K2 is:", b1,"N/m")

# In[ for copper]
E2 = 128*10**9
G2 = 45*10**9

a2 = K1(E2,G2,A,L)
b2 = K2(G2,A,L)

#putting force
F2 = 40*10**3
#displacement along the spring components
d2 = F2/a2
print(d2)
print("K2 is:", a2,"N/m")
# In[ for steel]
E3 = 215*10**9
G3 = 78*10**9

a3 = K1(E3,G3,A,L)
b3 = K2(G3,A,L)

#putting force
F3 = 50*10**3
#displacement along the spring components
d3 = F3/a3
print(d3)