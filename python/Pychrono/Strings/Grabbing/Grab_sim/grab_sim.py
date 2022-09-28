f"""c
Created on Mon Apr 27 18:26:11 2020

@author: dmulr
"""
import pychrono.core as chrono
import timeit
import grab_sim_objects as sim_obj
import config as cf
import math as ma

start=timeit.default_timer()
# Create system
chrono.SetChronoDataPath(cf.data_path)
my_system = chrono.ChSystemNSC() 
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

# Create interiors floor material and robots
material=sim_obj.Material(cf.mu_f,cf.mu_b,cf.mu_r,cf.mu_s,cf.C,cf.Ct,cf.Cr,cf.Cs)

# create floor
ENV=sim_obj.enviroment(my_system,material,cf.length,cf.tall,cf.env_mode,cf.gapw)
(my_system)=ENV.return_env()
body_floor=ENV.body_floor
# create robots
boundary= sim_obj.robot(cf.nb,cf.diameter,cf.height,cf.rowr,material,cf.k,cf.rl,body_floor,my_system,\
                       cf.fixed,cf.type_spring,cf.obj,cf.R,cf.pathind,cf.geom,cf.xc,cf.zc,cf.skind,cf.rationM)
(my_system,Springs,bots,obj,force)=boundary.return_system()

# create interior
inter=sim_obj.Interiors(cf.nb,cf.diameter,cf.diameter2,cf.rowp,cf.height,my_system,obj,\
                        body_floor,material,cf.fixed,cf.mode,cf.granmode,cf.R,cf.xc,cf.zc  )
(my_system,particles,obj,fbound)=inter.return_system()

if cf.control_type=="path_following" or cf.control_type=="shape_form" or cf.control_type=="tunneling" or cf.control_type=="shape_formation" or cf.control_type=="shape_form" or cf.control_type=="move_right"or cf.control_type=="shape_form2":
    balls=None
if cf.control_type=="grab_drag" or cf.control_type=="grab_drag_A" or cf.control_type=="grab_drag_2" or cf.control_type=="pot_field_grab_A"or cf.control_type=="pot_field_grab" or cf.control_type=="pot_field_grab_2" or cf.control_type=="verify" or cf.control_type=="local_field" :
# create ball
    balls=sim_obj.Ball(cf.control_type,my_system,body_floor,obj,material,cf.args)
    (my_system)=balls.return_system()
    
# collect contact points
my_rep2 = sim_obj.MyReportContactCallback2()

# create controller
controller=sim_obj.Controls(force,bots,inter,fbound,Springs,my_system,cf.k,cf.rl,cf.rlmax,cf.type_spring,\
                            cf.nb,cf.control_type,balls,cf.tstep,cf.diameter,cf.height,my_rep2,cf.args)
 

# collect contact points
my_rep = sim_obj.MyReportContactCallback()
# Create simulation
simulation=sim_obj.simulate(my_system,boundary,inter,balls,controller,Springs,obj,my_rep,cf.sim,cf.tstep,cf.tend,cf.visual,cf.data_path)
  
## run simulation
(boundary,time,controller,cx,cy,cz,Fxct,Fyct,Fzct,nc,bodiesA,bodiesB)=simulation.simulate()

end=timeit.default_timer()
print('Time wasted away in simulation: ',(end-start)/60, 'mins')

### export data
start=timeit.default_timer()

data=sim_obj.export_data(boundary,cf.nb,cf.sim,time,cx,cy,cz,Fxct,Fyct,Fzct,nc,cf.save_data,\
                         cf.mr,cf.mp,cf.mu_f,cf.mu_b,cf.mu_r,cf.mu_s,cf.C,cf.Ct,cf.Cr,cf.Cs,\
                         cf.tend,inter,balls,controller,bodiesA,bodiesB,cf.shapes)
data.save_variables()

end=timeit.default_timer()
print('Time spent exporting data:      ',(end-start)/60, 'mins')
