# -*- coding: utf-8 -*-
"""
Created on Fri May 15 16:42:39 2020

@author: dmulr
"""

from sympy.physics.mechanics import *
from sympy import sin, cos, symbols, Matrix, solve

#Inertial Reference Frame
N = ReferenceFrame('N')

#Define a world coordinate origin
O = Point('O')
O.set_vel(N, 0)

theta = dynamicsymbols('theta')                      #Rotation about N.z
x, y = dynamicsymbols('x, y')                        #Coordinates of robot in World Frame
phi1, phi2 = dynamicsymbols('phi_1, phi_2')          #Angular displacement of wheel

#Create q and dq vectors
q = Matrix([x, y, theta, phi1, phi2])
dq = q.diff()

#Constants for the wheels
r = symbols('r')                                     #Radius of wheel
m_w = symbols('m_w')                                 #Mass of the wheel
t_w = symbols('t_w')                                 #Thickness of the wheel

#Constants for the Robot Body
w = symbols('w')                                     #2*w is the width of the wheel base
d = symbols('d')                                     #Distance between axel and center of mass
m_b = symbols('m_b')                                 #Mass of the body
Ixx, Iyy, Izz = symbols('Ixx, Iyy, Izz')       

#Robot Reference Frame
R = N.orientnew('R', 'Axis', [theta, N.z])

#Center of wheel base
Cw = O.locatenew('Cw', x*N.x + y*N.y)

#Set the velocity of point Cw
Cw.set_vel(N, x.diff()*N.x + y.diff()*N.y)


#Points at wheel hubs
H1 = Cw.locatenew('H1', -w*R.y)
H2 = Cw.locatenew('H2', w*R.y)

#Set the velocity of points H1 and H2
H1.v2pt_theory(Cw, N, R)
H2.v2pt_theory(Cw, N, R);


#Create reference frames for wheels 1 and 2
W1 = R.orientnew('W1', 'Axis', [phi1, R.y])
W2 = R.orientnew('W2', 'Axis', [phi2, R.y])

#Calculate inertia of the wheel
Iw = inertia(R, m_w*(3*r**2 + t_w**2)/12, m_w*r**2/2, m_w*(3*r**2 + t_w**2)/12)

#Create rigid bodies for wheels
Wheel1 = RigidBody('Wheel1', H1, W1, m_w, (Iw, H1))
Wheel2 = RigidBody('Wheel2', H2, W2, m_w, (Iw, H2))

#Calculate inertia of body
Ib = inertia(R, Ixx, Iyy, Izz)

#Center of mass of body
Cm = Cw.locatenew('Cm', d*R.x)
Cm.v2pt_theory(Cw, N, R)

#Create a rigid body object for body
Body = RigidBody('Body', Cm, R, m_b, (Ib, Cm))

#Create two points, where the wheels contact the ground
C1 = H1.locatenew('C1', -r*R.z)
C2 = H2.locatenew('C2', -r*R.z)
#Calculate velocity of points
C1.v2pt_theory(H1, N, W1)
C2.v2pt_theory(H2, N, W2);

#Express the velocity of points in the inertial frame
con1 = C1.vel(N).express(N).args[0][0]
con2 = C2.vel(N).express(N).args[0][0]
#Create a matrix of constraints
constraints = con1.col_join(con2)
mprint(constraints)

#Solve for dx, dy, and dtheta in terms of dphi1 and dphi2
sol = solve(constraints, dq[:3])

sol_rhs = Matrix(sol.values())
sol_lhs = Matrix(sol.keys())