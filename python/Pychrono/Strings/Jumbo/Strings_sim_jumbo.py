# -*- coding: utf-8 -*-
"""
Created on Fri Oct 16 16:02:00 2020

@author: declan Mulroy
"""


import pychrono.core as chrono
import timeit
start=timeit.default_timer()
import Strings_objects_jumbo as sim_obj
import Strings_config_jumbo as cf
import math as ma
end=timeit.default_timer()
print('time importing',end-start)

startt=timeit.default_timer()
# Create system
chrono.SetChronoDataPath(cf.data_path)
my_system = chrono.ChSystemNSC() 
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

# Create floor material 
material=sim_obj.Material(cf.mu_f,cf.mu_b,cf.mu_r,cf.mu_s,cf.C,cf.Ct,cf.Cr,cf.Cs)

# create floor
ENV=sim_obj.enviroment(my_system,material,cf.length,cf.tall,cf.env_mode,cf.gapw)
(my_system)=ENV.return_env()
body_floor=ENV.body_floor
start=timeit.default_timer()
# create robots
boundary= sim_obj.robot(cf.nb,cf.radius,cf.height,cf.rowr,material,cf.k,cf.rl,body_floor,my_system,\
                       cf.fixed,cf.type_spring,cf.obj,cf.R,cf.geom,cf.xc,cf.zc,cf.skind,cf.rationM)
(my_system,Springs,bots,obj,force)=boundary.return_system()

end=timeit.default_timer()

print('time making bots',(end-start)/60)
# create interior
inter=sim_obj.Interiors(cf.nb,cf.radius2,cf.rowp,cf.height,my_system,obj,body_floor,material,cf.fixed,cf.mode,cf.R,cf.xc,cf.zc,cf.n)
(my_system,particles,obj)=inter.return_system()
end=timeit.default_timer()
print('time making interiors',(end-start)/60)
# create ball
if cf.control_type=="pot_field_grab" or cf.control_type=="grab_drag":
    balls=sim_obj.Ball(cf.control_type,my_system,body_floor,obj,material,height=cf.height,R=cf.Rb,rho=cf.rowb,zball=cf.zball,xball=cf.xball)
    (my_system)=balls.return_system()
else:
    balls=None

# contact collector 2 for local sensing 
my_rep2 = sim_obj.MyReportContactCallback2()

# controller
controller=sim_obj.Controls(force,bots,inter,Springs,my_system,cf.nb,cf.control_type,balls,cf.tstep,my_rep2,cf.w,cf.tn,cf.tpull,cf.args)

# collect contact points
my_rep = sim_obj.MyReportContactCallback()


# Create simulation
simulation=sim_obj.simulate(my_system,boundary,inter,balls,controller,Springs,obj,my_rep,cf.sim,cf.tstep,cf.tend,cf.visual,cf.data_path)
# run simulation
(boundary,time,controller,cx,cy,cz,Fxct,Fyct,Fzct,nc,bodiesA,bodiesB)=simulation.simulate()
endt=timeit.default_timer()
print('sim run time',(endt-startt)/60, 'minutes')
start=timeit.default_timer()
# export data
#sim_obj.export_data(boundary,inter,balls,cf.phi,controller,cf.tend,time,cf.sim,cf.nb,cf.mr,cf.mp,cf.mu_f,cf.mu_b,cf.mu_r,cf.mu_s,cf.C,cf.Ct,cf.Cr,cf.Cs,cx,cy,cz,Fxct,Fyct,Fzct,nc,bodiesA,bodiesB,cf.pwm,cf.w,cf.tn,cf.gapw,cf.env_mode,cf.save_data)
end=timeit.default_timer()
print('export data run time',(start-end)/60, 'minutes')