# -*- coding: utf-8 -*-
"""
Created on Sun Feb  9 11:18:33 2020

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
import json
from objects import BOTS_Jamming,Interior,Material,MyReportContactCallback,MaxValues,Floor,ExportData3,ExtractData3

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
    
    mb=parameters["mb"]
    mp=parameters["mp"]
    k=parameters["k"]
    b=parameters["b"]
    rl=parameters["rl"]
    record=parameters["run_mode"]
    tend=parameters["tend"]
    obj=[]              # empty matrix of bots and particles
    Springs=[]          # empty matrix of springs

    volume=np.pi*.25*height*(diameter)**2           # calculate volume
    volume2=np.pi*.25*height*(diameter2)**2         # calculate volume
    rowr=mb/volume # calculate density of robot
    rowp=mp/volume2 # calculate density of particles
    R1=(diameter*nb/(np.pi*2))+.15
    n=MaxValues(R1,diameter,nb)
    n=n[0]
    #n=np.array([6,1])
    ni=np.sum(n)
    nt=ni+nb
    # create folders
    path="F:/Robosoft2020_data/revisiions_jamming/trial11/results/"+sim
    if not os.path.exists(path):
        #os.mkdir("results/")
        os.mkdir(path)
    
    start = timeit.default_timer()

    # In[Set Path]
    chrono.SetChronoDataPath("C:/Users/dmulr/OneDrive/Documents/data/")
    # In[Create sysem and other misselanous things]
    my_system = chrono.ChSystemNSC()
    #my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    my_system.SetSolverType(chrono.ChSolver.Type_SOR_MULTITHREAD)
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    my_system.SetMaxItersSolverSpeed(300)
    

    material=Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs)
    
    (body_floor)=Floor(material,length,tall)
    my_system.Add(body_floor)
# In[Create bots]    
    for i in range(nb):
          # Initial postion of each bot
          theta=i*2*np.pi/nb
          x=R1*np.cos(theta)
          y=.5*height
          z=R1*np.sin(theta)  
          BOTS_Jamming(nb,i,x,y,z,theta,diameter,height,rowr,obj,material,body_floor,my_system,Springs,k,rl)
    
# In[Create Interior]
    for i in range(n.size):
        for j in range(n[i]):
            R2=(diameter*n[i])/(np.pi*2)     # Radius of interiro ring
            x=R2*np.cos(j*2*np.pi/n[i])     # x position
            y=.5*height                     # Y POSITION
            z=R2*np.sin(j*2*np.pi/n[i])     # z position
            Interior(x,y,z,i,diameter,height,rowp,R2,material,obj,my_system,body_floor)

# In[ Create empty matrices to be filled]
    Xpos=[]
    Ypos=[]
    Zpos=[]

    Xforce=[]
    Yforce=[]
    Zforce=[]

    # Contact forces 
    Xcontact=[]
    Ycontact=[]
    Zcontact=[]

    # empty temporary velocity matrices
    Xvel=[]
    Yvel=[]
    Zvel=[]

    templ=[]
    ttemp=[]
    # Rotation Positions
    rott0=[]
    rott1=[]
    rott2=[]
    rott3=[]
    # empty Spring force matrix
    Fm=[]
    sl=[]

    # VERY important
    count=0
    # Contact points
    cx=[]
    cy=[]
    cz=[]
    # number of contacts
    nc=[]

    # Contact forces
    Fxct=[]
    Fyct=[]
    Fzct=[]

    # used for collecting contact points
    my_rep = MyReportContactCallback()
    # normal and tan directions
    Vtx=[]
    Vtz=[]
    Vnx=[]
    Vnz=[]


# In[Pov RAY]
    if record==1:
    
        script_dir = os.path.dirname("povvideofiles"+str(sim)+"/")
        pov_exporter = postprocess.ChPovRay(my_system)

    
        # Sets some file names for in-out processes.
        pov_exporter.SetTemplateFile(chrono.GetChronoDataPath() + "_template_POV.pov")
        pov_exporter.SetOutputScriptFile("results/"+str(sim)+"/rendering"+str(sim)+".pov")
        pov_exporter.SetOutputDataFilebase("my_state")
        pov_exporter.SetPictureFilebase("picture")

        pov_exporter.SetAntialiasing(True,9,9)
        pov_exporter.SetPictureSize(1600,1200)
        pov_exporter.SetShowLinks(True,diameter/3)
        # create folders
        if not os.path.exists("results/"+str(sim)+"/output"):
            os.mkdir("results/"+str(sim)+"/output")
        if not os.path.exists("results/"+str(sim)+"/anim"):
                os.mkdir("results/"+str(sim)+"/anim")
        pov_exporter.SetOutputDataFilebase("results/"+str(sim)+"/output/my_state")
        pov_exporter.SetPictureFilebase("results/"+str(sim)+"/anim/picture")
        pov_exporter.AddAll()
        pov_exporter.ExportScript()
        #In[Run the simulation]
        count=0
        while (my_system.GetChTime() < tend) :
            my_rep.ResetList()
            print ('time=', my_system.GetChTime())
    
            t=tstep*count
            (Xforce,Yforce,Zforce,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Fm,sl)=ExtractData3(obj,nb,nt,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Xforce,Yforce,Zforce,Springs,Fm,sl)
            my_system.GetContactContainer().ReportAllContacts(my_rep)
            crt_list = my_rep.GetList()
            nc.append(my_system.GetContactContainer().GetNcontacts())
            cx.append(crt_list[0])
            cy.append(crt_list[1])
            cz.append(crt_list[2])
            Fxct.append(crt_list[3])
            Fyct.append(crt_list[4])
            Fzct.append(crt_list[5])
            count=count+1# count
            ttemp.append(t)
            my_system.DoStepDynamics(tstep)
            if count%15==0:
                pov_exporter.ExportData()  

# In[IRRLICHT code]
    else:
        myapplication = chronoirr.ChIrrApp(my_system, 'Large Scale', chronoirr.dimension2du(1200,800))
        myapplication.AddTypicalSky()
        myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
        myapplication.AddTypicalCamera(chronoirr.vector3df(0.75,0.75,1.5))
        myapplication.AddLightWithShadow(chronoirr.vector3df(2,5,2),chronoirr.vector3df(2,2,2),10,2,10,120)
        myapplication.DrawAll               
        myapplication.AssetBindAll();
        myapplication.AssetUpdateAll();
        myapplication.AddShadowAll();

        count=0
        # Time step
        myapplication.SetTimestep(tstep)
        myapplication.SetTryRealtime(False)

        while(myapplication.GetDevice().run()):
            myapplication.BeginScene()
            myapplication.DrawAll()
            my_rep.ResetList()
            print ('time=', my_system.GetChTime())
    
            t=tstep*count
            (Xforce,Yforce,Zforce,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Fm,sl)=ExtractData3(obj,nb,nt,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Xforce,Yforce,Zforce,Springs,Fm,sl)
            my_system.GetContactContainer().ReportAllContacts(my_rep)
            crt_list = my_rep.GetList()
            nc.append(my_system.GetContactContainer().GetNcontacts())
            cx.append(crt_list[0])
            cy.append(crt_list[1])
            cz.append(crt_list[2])
            Fxct.append(crt_list[3])
            Fyct.append(crt_list[4])
            Fzct.append(crt_list[5])
            count=count+1# count
            ttemp.append(t)
            myapplication.DoStep()
            myapplication.EndScene()
            if t > tend:
                myapplication.GetDevice().closeDevice() 
    # convert to array number of contacts
    nc=np.asarray(nc)

    # find max length of contacts
    lengthm=np.amax(nc)
    (xc,yc,zc,Fcx,Fcy,Fcz,Xv,Yv,Zv,Fmem,rot0,rot1,rot2,rot3,qx,qy,qz,Fxt,Fyt,Fzt,SL)=ExportData3(Xforce,Yforce,Zforce,Fxct,Fyct,Fzct,cx,cy,cz,nc,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Fm,sl,nb,ni,nt,count,lengthm)
    variables=([nb,ni,diameter,height,volume,mb,mp,k,b,mu_f,mu_b,mu_r,mu_s,rl])
    
    texts=["number of boundary(n/a)=","\r\n number of interior(n/a)=","\r\n diameter(m)=","\r\n height of each robot(m)=","\r\n volume(m^3)=","\r\n mass of each robot(kg)=","\r\n mass of each particle(kg)=","\r\n spring constant(N/m)=","\r\n damping coefficent(Ns/m)=","\r\nsliding friction=","\r\nmaterial dampning=","\r\nRolling friction=","\r\nSpinning friction=","\r\nresting length="]
    
    f= open("F:/Robosoft2020_data/revisiions_jamming/trial11/results/"+str(sim)+"/"+"variables.txt","w+")
    
    for i in range(np.size(variables)):
        
        f.write(texts[i]+str(variables[i]))
        
    with open("F:/Robosoft2020_data/revisiions_jamming/trial11/results/"+str(sim)+'/'+str(sim)+'.json', 'w') as outfile:
        json.dump(parameters, outfile,indent=2)
        
    np.savez("F:/Robosoft2020_data/revisiions_jamming/trial11/data_folder/"+str(sim)+".npz",allow_pickle=True,
             xc=xc,
             yc=yc,
             zc=zc,
             Fcx=Fcx,
             Fcy=Fcy,
             Fcz=Fcz,
             Xv=Xv,
             Yv=Yv,
             Zv=Zv,
             Fmem=Fmem,
             rot0=rot0,
             rot1=rot1,
             rot2=rot2,
             rot3=rot3,
             qx=qx,
             qy=qy,
             qz=qz,
             Fxt=Fxt,
             Fyt=Fyt,
             Fzt=Fzt,
             SL=SL,
             nb=nb,
             ni=ni,
             mb=mb,
             mp=mp,
             k=k,
             rowr=rowr,
             rowp=rowp,
             height=height,
             diameter=diameter,
             volume=volume,
             ttemp=ttemp,
             count=count,
             sim=sim,
             mu_f=mu_f,
             mu_r=mu_r,
             mu_s=mu_s)
    stop = timeit.default_timer()
    
    runtime=stop-start
    runtime=runtime
    print("Total runtime: "+str(runtime)+" seconds")
    
    return (qx,qz,runtime,ttemp,nb,ni,diameter,height,count)