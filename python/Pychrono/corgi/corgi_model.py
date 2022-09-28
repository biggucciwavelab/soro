'''

author: declan mulroy
project: JAMoEBA
email: dmulroy@hawk.iit.edu
date: 11/19/19
Corgi_model
'''

# In[import libraries]
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np

import timeit
from corgi_objects import Material,Floor,Interior,BOTS_Spinners,Wall,Controller3,MyReportContactCallback,ExtractData2,Exportdata2
from myconfig6 import *

start = timeit.default_timer()

# In[Set Path]
chrono.SetChronoDataPath("C:/Users/Amin/Documents/chrono-data/")

# In[Create sysem and other misselanous things]
my_system = chrono.ChSystemNSC()
# my_system.SetSolverType(chrono.ChSolver.Type_PSOR)
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
#my_system.SetTol(1e-6)
#my_system.SetSolverType(chrono.ChSolver.Type_APGD)
my_system.Set_G_acc(chrono.ChVectorD(-9.81,0, 0))
# my_system.SetMaxItersSolverSpeed(300)

# In[ Material 1]
material=Material(mu_f, mu_b, mu_r, mu_s, C, Ct, Cr, Cs)

# In[Material 2]
material2=Material(mu_f2,mu_b2,mu_r2,mu_s2,C,Ct,Cr,Cs)
# In[Create Floor]
body_floor=Floor(material2,length,tall)
my_system.Add(body_floor)




# In[Create Robots]
for i in range(nb):
    
    theta=i*2*np.pi/nb# define angle
    x=R1*np.cos(theta)# define x position
    y=.5*height         # define y position
    z=R1*np.sin(theta)  # defin z position
    # Create robots
    BOTS_Spinners(nb,i,x,y,z,diameter,height,theta,rowr,body_floor,obj,my_system,material,Springs,k,rl,botcall,force)
# In[Create Interior]
for i in range(n.size):
  
    # Creates interior with different diameters. Alternateds diameter 2 and diameter*sqrt(2)
    if i%2==0:
        diameter3=diameter2#*(2**.5)
    else:
        diameter3=diameter2
    for j in range(n[i]):
        print(j)
        R2=diameter3*n[i]/(np.pi*2)     # Radius of interiro ring
        x=R2*np.cos(j*2*np.pi/n[i])     # x position
        y=.5*height                     # Y POSITION
        z=R2*np.sin(j*2*np.pi/n[i])     # z position
        # create interior
        Interior(x,y,z,i,diameter3,height,rowp,R2,material,obj,my_system,body_floor)

# In[Create Wall]  
# x position of wall
z =  0      # Z position of wall
rotate = np.pi/2    # rotate wall
length = 10     # length of wall
height = .25    # height of wall
width = .1      # width of wall
y = height/1.9  # initial y position of wall
x=-R1-.03
Wall(x,y,z,rotate,length,height,width,material,my_system)



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

# empty ball matrix
ballp=[]
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
    
    script_dir = os.path.dirname("povvideofiles"+sim+"/")
    pov_exporter = postprocess.ChPovRay(my_system)

    # Sets some file names for in-out processes.
    pov_exporter.SetTemplateFile(chrono.GetChronoDataPath() + "_template_POV.pov")
    pov_exporter.SetOutputScriptFile("rendering"+str(sim)+".pov")
    pov_exporter.SetOutputDataFilebase("my_state")
    pov_exporter.SetPictureFilebase("picture")

    # create folders
    if not os.path.exists("output"+str(sim)):
        os.mkdir("output"+str(sim))
    if not os.path.exists("anim"+str(sim)):
            os.mkdir("anim"+str(sim))
    pov_exporter.SetOutputDataFilebase("output"+str(sim)+"/my_state")
    pov_exporter.SetPictureFilebase("anim"+str(sim)+"/picture")
    pov_exporter.SetCamera(chrono.ChVectorD(0,3,0), chrono.ChVectorD(0,0,0), 90)# specifiy camera location
    pov_exporter.AddAll()
    pov_exporter.ExportScript()

    #In[Run the simulation]

    count=0
    t=tstep*count 
    while (my_system.GetChTime() < tend) :
        # time 
        print ('time=', my_system.GetChTime() )
        t=tstep*count
        my_rep.ResetList()
        # controller function
        Controller(my_system,force,botcall,obj,Springs,jamcall,normcall,forceb,mag,mag2,mag3,magf,k,kj,t,tj,tp,tset,Fm,nb,templ,rl,rlmax,rlj,rljmax)
        # extract data function 
        ExtractData(obj,nb,nt,Xpos,Ypos,Zpos,Xforce,Yforce,Zforce,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,ballp,Balls,Springs,Fm,templ,Vn,NF,Xt,Zt,NT)
        # function for getting all contact points
        my_system.GetContactContainer().ReportAllContacts(my_rep)
        crt_list = my_rep.GetList()
        nc.append(my_system.GetContactContainer().GetNcontacts())   # number of contacts
        cx.append(crt_list[0])  # x positon
        cy.append(crt_list[1])  # y position
        cz.append(crt_list[2])  # z position
        Fxct.append(crt_list[3])    # x force
        Fyct.append(crt_list[4])    # y force
        Fzct.append(crt_list[5])    # z force
  
        # Track center of robot for RL 
        Xpostemp=[]
        Zpostemp=[]
        for i in range(nb):
            Xpostemp.append(obj[i].GetPos().x)
            Zpostemp.append(obj[i].GetPos().z)
     
        Xpostemp=np.asarray(Xpostemp)
        Zpostemp=np.asarray(Zpostemp)
        
        Xavg=np.mean(Xpostemp)
        Zavg=np.mean(Zpostemp)
        
        ttemp.append(t)
        count=count+1
        my_system.DoStepDynamics(tstep)
        print ('time=', my_system.GetChTime())
        pov_exporter.SetCamera(chrono.ChVectorD(Xavg,3,Zavg), chrono.ChVectorD(Xavg,0,Zavg), 90)# specifiy camera location
        pov_exporter.AddAll()
        pov_exporter.ExportScript()
        # Export every 15th frame 
        if count%15==0:
            pov_exporter.ExportData()  
# In[Irrlecht]
else:
    myapplication = chronoirr.ChIrrApp(my_system,sim, chronoirr.dimension2du(1600,1200))
    myapplication.AddTypicalSky()
    myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    myapplication.AddTypicalCamera(chronoirr.vector3df(0,1,0))
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
        my_rep.ResetList()
        myapplication.BeginScene()
        myapplication.DrawAll()
        print ('time=', my_system.GetChTime())
        t=tstep*count  
        # In[Controller Function]
        (force,Springs,templ,my_system,Fm)=Controller3(my_system,force,botcall,obj,Springs,jamcall,Fm,mag,k,kj,t,tj,tp,tset,nb,templ,rl,rlmax,rlj,rljmax)
        
        # In[Extract Data]
        (obj,Springs,Fm,templ,Xforce,Yforce,Zforce,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel)=ExtractData2(obj,nb,nt,Xpos,Ypos,Zpos,Xforce,Yforce,Zforce,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Springs,Fm,templ)
        # Contact container. Collect all contact points
        my_system.GetContactContainer().ReportAllContacts(my_rep)
        # Export and sort contacts
        crt_list = my_rep.GetList()
        nc.append(my_system.GetContactContainer().GetNcontacts())
        cx.append(crt_list[0])      # x position contacts
        cy.append(crt_list[1])      # y position contacts
        cz.append(crt_list[2])      # z position contacts
        Fxct.append(crt_list[3])    # Force x position contacts
        Fyct.append(crt_list[4])    #Force y position contacts
        Fzct.append(crt_list[5])    # Force Z position contacts
        ttemp.append(t)             # time append 
        count=count+1               # count
        
        # run step
        myapplication.DoStep()
        myapplication.EndScene()
# Close the simulation if time ends
        if t > tend:
            myapplication.GetDevice().closeDevice()


# convert to array number of contacts
nc=np.asarray(nc)

# find max length of contacts
lengthm=np.amax(nc)





# In[Export the data. Sort in arrays] 
(qx,qy,qz,rot0,rot1,rot2,rot3,Fxt,Fyt,Fzt,Xv,Yv,Zv,SL,Fmem,Fcx,Fcy,Fcz,xc,yc,zc)=Exportdata2(Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,templ,Xforce,Yforce,Zforce,Xcontact,Ycontact,Zcontact,Xvel,Yvel,Zvel,Fm,nb,nt,count,lengthm,nc,cx,cy,cz,Fxct,Fyct,Fzct)
   
# In[Save and export out to npz file]    
np.savez(sim+".npz",allow_pickle=True,
         Fxt=Fxt,
         Fyt=Fyt,
         Fzt=Fzt,
         qx=qx,
         qy=qy,
         qz=qz,
         nb=nb,
         ni=ni,
         mr=mr,
         mp=mp,
         k=k,
         rowr=rowr,
         rowp=rowp,
         height=height,
         diameter=diameter,
         volume=volume,
         ttemp=ttemp,
         count=count,
         rot0=rot0,
         rot1=rot1,
         rot2=rot2,
         rot3=rot3,
         botcall=botcall,
         SL=SL,
         Xv=Xv,
         Yv=Yv,
         Zv=Zv,
         sim=sim,
         Fmem=Fmem,
         xc=xc,
         yc=yc,
         zc=zc,
         nc=nc,
         Fcx=Fcx,
         Fcy=Fcy,
         Fcz=Fcz)
stop = timeit.default_timer()

# In[Print time out]
runtime=stop-start
runtime=runtime*(1/60)
print("Total runtime: "+str(runtime)+" minutes")