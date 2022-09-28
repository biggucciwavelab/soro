 # -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 18:26:11 2020

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
from grab_sim_objects import *
from config import *
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import json

def run():
    
     
    with open('parameters.json') as json_data_file:
        parameters = json.load(json_data_file)
    sim=str(parameters["output_dir"])
    tstep=parameters["tstep"]
    tset=parameters["tset"]
    mu_f=parameters["mu_f"]
    mu_b=parameters["mu_b"]
    mu_s=parameters["mu_s"]
    mu_r=parameters["mu_r"]
    Ct=parameters["Ct"]
    C=parameters["C"]
    Cr=parameters["Cr"]
    Cs=parameters["Cs"]
    
    length=parameters["length"]
    tall=parameters["tall"]
    
    nb=parameters["nb"]
    diameter=parameters["diameter"]
    diameter2=parameters["diameter2"]
    height=parameters["height"]
    
    mr=parameters["mr"]
    mp=parameters["mp"]
    k=parameters["k"]
    rl=parameters["rl"]
    rlmax=parameters["rlmax"]
    tend=parameters["tend"]
    alpha=parameters["alpha"]
    beta=parameters["beta"]
    ##############################
    visual="irrlecht"
    type_spring="var"
    control_type="pot_field"
    fixed=False# is it fixed
    mode='nmax'  # will there be interiors
    colar_mode=False
    mag=2
    lc=0
    obj=[]
    data_path="C:/Users/dmulr/OneDrive/Documents/data/"
    shape="grab"
    volume=np.pi*.25*height*(diameter)**2   # calculate volume
    volume2=np.pi*.25*height*(diameter2)**2   # calculate volume
    rowr=mr/volume # calculate density of robot
    rowp=mp/volume2
    R=(diameter*nb/(np.pi*2))+.1 
    Rd=4*R

    bl=-1.5
    br=1.5
    p1=0
    p2=0
    
    if shape=="grab":
        nr=[]

    position=True
    forces=True
    control_force=True
    spring=True
    contact=True
    error=True

    save_data=[position,forces,control_force,spring,contact,error]

    # create folders
    path="F:/Grabbing3/"+sim
    # In[Create system]
    chrono.SetChronoDataPath(data_path)
    my_system = chrono.ChSystemNSC()
    my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
    my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 

    shapes=Points_for_shape(shape,p1,p2,nb,diameter,bl,br,R,nr,Rd)
    #shapes.create_RBF()
    (rbf,fnx,fny)=shapes.Create_shape_gradient()

    # In[Create interiors floor material and robots]
    material=Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs)

    # create floor
    body_floor=Floor(material,length,tall)
    my_system.Add(body_floor)

    # create robots
    boundary=robot(nb,diameter,height,rowr,material,k,rl,body_floor,my_system,fixed,type_spring,obj,mag,R)
    (my_system,Springs,bots,obj,force)=boundary.return_system()

    # create interior
    inter=Interiors(nb,diameter,diameter2,rowp,height,my_system,obj,body_floor,material,fixed,mode)
    (my_system,particles,obj)=inter.return_system()

    # create controller
    controller=Controls(force,bots,Springs,k,rl,rlmax,type_spring,control_type,mag,fnx,fny,rbf,alpha,beta,nb)

    my_rep = MyReportContactCallback()

    count=0
    #data_contact=Save_contact_data(my_rep,my_system,count)

    # Create simulation
    simulation=simulate(my_system,boundary,inter,Springs,obj,my_rep,sim,tstep,tend,visual,data_path,controller,lc)
    # run simulation
    (boundary,time,controller,cx,cy,cz,Fxct,Fyct,Fzct,nc)=simulation.simulate()

    data=export_data(boundary,inter,cx,cy,cz,Fxct,Fyct,Fzct,nc,controller,nb,sim,time,save_data,path)
    
    (Fcx,Fcy,Fcz)=data.return_contact_forces()
    (XL,ZL)=boundary.return_last_position()
    E=controller.E
    
    return(E[-1],Fcx,Fcy,Fcz)

