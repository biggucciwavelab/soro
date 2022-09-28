# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 10:06:48 2020

@author: Qiyuan

Tkinter GUI to easily tweak all sim parameters before each run
"""

# Gui Imports and file system imports
import tkinter as tk
from tkinter import font as tkfont
from tkinter import PhotoImage
import os

# Simulation imports
import pychrono.core as chrono
import numpy as np
import timeit

# Custom imports
import grab_sim_objects as simobj
import grab_sim_plot_objects as plotobj

# Class for creating tooltips
class CreateToolTip(object):
    """
    create a tooltip for a given widget
    """
    def __init__(self, widget, text='widget info'):
        self.waittime = 250     #miliseconds
        self.wraplength = 200   #pixels
        self.widget = widget
        self.text = text
        self.widget.bind("<Enter>", self.enter)
        self.widget.bind("<Leave>", self.leave)
        self.widget.bind("<ButtonPress>", self.leave)
        self.id = None
        self.tw = None

    def enter(self, event=None):
        self.schedule()

    def leave(self, event=None):
        self.unschedule()
        self.hidetip()

    def schedule(self):
        self.unschedule()
        self.id = self.widget.after(self.waittime, self.showtip)

    def unschedule(self):
        id = self.id
        self.id = None
        if id:
            self.widget.after_cancel(id)

    def showtip(self, event=None):
        x = y = 0
        x, y, cx, cy = self.widget.bbox("insert")
        x += self.widget.winfo_rootx() + 25
        y += self.widget.winfo_rooty() + 20
        # creates a toplevel window
        self.tw = tk.Toplevel(self.widget)
        # Leaves only the label and removes the app window
        self.tw.wm_overrideredirect(True)
        self.tw.wm_geometry("+%d+%d" % (x, y))
        label = tk.Label(self.tw, text=self.text, justify='left',
                       background="#ffffff", relief='solid', borderwidth=1,
                       wraplength = self.wraplength)
        label.pack(ipadx=1)

    def hidetip(self):
        tw = self.tw
        self.tw= None
        if tw:
            tw.destroy()

class UserInputs:
    def __init__(self):
        self.simulation=None
        self.params=[]

    # Def to load the data
    def run_jameboa(self):
        start = timeit.default_timer()          # Start the timer
        params=[]                               # Array to store all inputs from user
        self.params=[]
    ### Load params array with inputs from gui ###
        # Column 1
        params.append(variable1.get())          # 0: Interior generation mode
        params.append(variable2.get())          # 1: Interior granular mode
        params.append(float(ttstep.get()))      # 2: Time step
        params.append(float(ttend.get()))       # 3: End time
        params.append(float(muf.get()))         # 4: Sliding friction
        params.append(float(mub.get()))         # 5: Friction Damping
        params.append(float(mur.get()))         # 6: Rolling friction
        params.append(float(mus.get()))         # 7: Spinning Friction
        params.append(float(ct.get()))          # 8: Tangential compliance
        params.append(float(c.get()))           # 9: Normal compliance
        params.append(float(cr.get()))          # 10: Rolling compliance
        params.append(float(cs.get()))          # 11: Spinning compilance
        params.append(float(rmass.get()))       # 12: Robot mass
        params.append(int(rnum.get()))          # 13: Number of robots
        params.append(float(rheight.get()))     # 14: Robot height
        params.append(float(rdiam.get()))       # 15: Robot diameter
        params.append(float(imass.get()))       # 16: Interior particle mass
        params.append(float(idiam.get()))       # 17: Interior particle diameter
        
        # Column 2
        params.append(variable3.get())          # 18: Visualization method
        params.append(variable4.get())          # 19: Is system fixed?
        params.append(simnum.get())             # 20: Sim number
        params.append(datapath.get())           # 21: Data path
        params.append(variable5.get())          # 22: Control mode
        params.append(variable6.get())          # 23: Control shape
        params.append(float(alphag.get()))      # 24: Control alpha
        params.append(float(betag.get()))       # 25: Control beta
        params.append(float(magg.get()))        # 26: Control force mag
        params.append(float(bfx.get()))         # 27: Ball fx
        params.append(float(bfz.get()))         # 28: Ball fz
        params.append(variable7.get())          # 29: Spring mode
        params.append(float(springk.get()))     # 30: Spring constant
        params.append(float(springrl.get()))    # 31: Spring resting length
        params.append(float(springrlmax.get())) # 32: Max spring resting length
        params.append(float(floorlen.get()))    # 33: Floor length
        params.append(float(floortall.get()))   # 34: Floor height
        
        # Column 3
        params.append(plotpos.get())            # 35: Save position
        params.append(plotvel.get())            # 36: Save velocity
        params.append(plotfor.get())            # 37: Save net forces
        params.append(plotcon.get())            # 38: Save control forces
        params.append(ploterr.get())            # 39: Save control error 
        params.append(plotspr.get())            # 40: Save spring positions
        params.append(plotcont.get())           # 41: Save contact forces
        params.append(plotptpos.get())          # 42: Save part positions
        params.append(plotptvel.get())          # 43: Save part velocities
        params.append(plotptfor.get())          # 44: Save part forces
        self.params=params
        
    ### Initialize the actual variables from params array ###
        visual=params[18]
        sim=params[20]
        fixed=params[19]
        obj=[]
        data_path=params[21]
        control_type=params[22]
        mode=params[0]
        granmode=params[1]
        tstep=params[2]
        tend=params[3]
        mu_f=params[4]
        mu_b=params[5]
        mu_r=params[6]
        mu_s=params[7]
        Ct=params[8]
        C=params[9]
        Cr=params[10]
        Cs=params[11]     
        mr=params[12]
        nb=params[13]
        height=params[14]
        diameter=params[15]
        volume=np.pi*.25*height*(diameter)**2   # calculate volume
        rowr=mr/volume                          # calculate density of robot
        R=(diameter*nb/(np.pi*2))+.1 
        actbots=np.arange(0,nb,1)
        active=np.zeros(nb)
        for i  in range(len(active)):
            if any(actbots==i):
                active[i]=1
    
        mp=params[16]
        diameter2=params[17]
        volume2=np.pi*.25*height*(diameter2)**2     # calculate volume
        rowp=mp/volume2                             # density
        k=params[30]
        rl=params[31]
        rlmax=params[32]
        type_spring=params[29]
        length=params[33]
        tall=params[34] 
        mag=params[26]
        pathind=0
        
        # Load default parameters
        balls=None
        #####################################################################################
        if params[22]=="path_following":
            #### Path variables ####
            (ax,ay)=simobj.Path.create_line()
            paths=simobj.Path(ax,ay)    
            pathind=0
            vref=1
            poseind=np.arange(1,nb,1)
            args=(paths,pathind,vref,poseind)
        ##########################################################################################################    
        if params[22]=="pot_field_grab": 
            shape=params[23]
            alpha=params[24]
            beta=params[25]
            balls=0
            p1=.95
            p2=0
            bl=-9
            br=9
            Rd=0.2
            if shape=="circle":
                nr=[0,1,2,3,4] # circle
            if shape=="Square":
                nr=[0,1,2,3] # square
            if shape=="grab":
                nr=[]
            ##### Ball variables #####
            mb=1
            Rb=.4
            volume3=np.pi*.25*height*(Rb)**2   # calculate volume
            rowb=mb/volume3 # density
            xball=0
            zball=p1   
            shapes=simobj.Points_for_shape(shape,xball,zball,nb,diameter,bl,br,R,nr,Rd)
            args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball
        ######################################################################################
        if params[22]=="grab_drag":
            (ax,ay)=simobj.Path.create_line()
            paths=simobj.Path(ax,ay)
            pathind=0
            vref=1
            poseind=np.arange(1,nb,1)
        
            shape="circle"
            alpha=100
            beta=1
            balls=0
            p1=.95
            p2=0
            bl=-9
            br=9
            Rd=0.2
            if shape=="circle":
                nr=[0,1,2,3,4] # circle
            if shape=="Square":
                nr=[0,1,2,3] # square
            if shape=="grab":
                nr=[]
            ##### Ball variables #####
            mb=1
            Rb=.4
            volume3=np.pi*.25*height*(Rb)**2   # calculate volume
            rowb=mb/volume3 # density
            xball=0
            zball=p1    
            shapes=simobj.Points_for_shape(shape,xball,zball,nb,diameter,bl,br,R,nr,Rd)
            args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball,paths,pathind,vref,poseind
            
        ########################################################################################################### 
        
        ##### Save variables #####
        save_data=[params[35],params[36],params[37],params[38],params[39],\
                   params[40],params[41],params[42],params[43],params[44]]
        
        ### Run the simulation ###
        
        # Create system
        chrono.SetChronoDataPath(data_path)
        my_system = chrono.ChSystemNSC()
        my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
        my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 
        
        # Create interiors floor material and robots
        material=simobj.Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs)
        
        # Create floor
        body_floor=simobj.Floor(material,length,tall)
        my_system.Add(body_floor)
        
        # Create robots
        boundary=simobj.robot(nb,diameter,height,rowr,material,k,rl,body_floor,my_system,\
                       fixed,type_spring,obj,mag,R,active,pathind)
        (my_system,Springs,bots,obj,force)=boundary.return_system()
        
        # Create interior
        inter=simobj.Interiors(nb,diameter,diameter2,rowp,height,my_system,obj,body_floor,\
                        material,fixed,mode,granmode)
        (my_system,particles,obj,fbound)=inter.return_system()
        
        # Add the balls to be grabbed
        if params[22]=="grab_drag":
            balls=simobj.Ball(control_type,my_system,body_floor,obj,material,args)
        
        controller=None
        # Add the controller
        if params[22]!="nothing":
            controller=simobj.Controls(force,bots,particles,fbound,Springs,my_system,k,rl,\
                            rlmax,type_spring,mag,nb,actbots,active,control_type,balls,args)
            
        ## collect contact points
        my_rep = simobj.MyReportContactCallback()
        #
        ## Create simulation
        self.simulation=simobj.simulate(my_system,boundary,inter,balls,controller,Springs,obj,\
                            my_rep,sim,tstep,tend,visual,data_path)
          
        ## run simulation
        (boundary,time,controller,cx,cy,cz,Fxct,Fyct,Fzct,nc)=self.simulation.simulate()
        
        stop = timeit.default_timer()          # Stop the timer
        print('Time spent in simulation: ', stop - start,'s') 
        
    def end_sim(self):
        self.simulation.myapplication.GetDevice().closeDevice()
        
    def plot(self):
        plots=[]        # Array to store user inputs from gui
        
        ### Fill plots array with whether or not to plot ###
        plots.append(plotpos)           # 0: Position
        plots.append(plotvel)           # 1: Velocity
        plots.append(plotfor)           # 2: Forces
        plots.append(plotcon)           # 3: Controller forces
        plots.append(ploterr)           # 4: Control error
        plots.append(plotspr)           # 5: Spring Lengths
        plots.append(plotcont)          # 6: Contact forces
        plots.append(plotptpos)         # 7: ?? Particle positions ??
        plots.append(plotptvel)         # 8: ?? Particle velocities ??
        plots.append(plotptfor)         # 9: ?? Particle forces ??
        plots.append(simnum.get())      # 10: Sim number
        
        # Plot the plots
        results_dir = os.path.join('plots'+plots[10])
        
        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)
            
        results=plotobj.robot_plots(self.params[13],results_dir)
        
        if plots[0]==True:
            results.plot_path()
            
        if plots[1]==True:
            # Velocity plot seems to be missing
            a=1
            
        if plots[2]==True:
            results.Plot_fxyz()
            
        if plots[3]==True:
            results.Plot_fxyzcontroller()
            results.Control_force_sesmic_x()
            results.Control_force_sesmic_z()
            results.Control_force_sesmic()
            results.Control_force_sesmic_animation()
        
        if plots[4]==True:
            results.plot_error()
            
        if plots[5]==True:
            results.Plot_spring_force_length()
            
        if plots[6]==True:
            # Not sure how contact force plots work rn
            a=1
            
        if plots[7]==True:
            # Not sure how particle position plots work 
            a=1
        
        if plots[8]==True:
            # Not sure how particle velocity plots work
            a=1
            
        if plots[9]==True:
            # Not sure how particle force plots work
            a=1
        

# Init the user input class
usrinputs=UserInputs()

# Create the window
win = tk.Tk()
win.title('Sim Parameters')
win.config(borderwidth=10,cursor='tcross')

# %% Dropdown menus
# Fields
intmodes=["empty","nonhomo","max","nmax","nonhnmax","homo"]
granmodes=["homo","nonhomo"]
visuals=["irr","pov"]
fixed=[False,True]
fixedd=['False','True']
contmodes=["nothing","force_right","pot_field","pot_field_grab","pot_field_drag","pot_field_dragA","path_following","grab_drag"]
conshapes=["Square","grab","circle","circleA"]
springmodes=["const","var"]

# Defaults
variable1 = tk.StringVar(win)
variable1.set(intmodes[4])
variable2 = tk.StringVar(win)
variable2.set(granmodes[0])
variable3 = tk.StringVar(win)
variable3.set(visuals[0])
variable4 = tk.BooleanVar(win)
variable4.set(fixed[0])
variable5 = tk.StringVar(win)
variable5.set(contmodes[6])
variable6 = tk.StringVar(win)
variable6.set(conshapes[2])
variable7 = tk.StringVar(win)
variable7.set(springmodes[1])

# %% Labels for fields
bold_font = tkfont.Font(family="Helvetica", size=10, weight="bold")

tk.Label( win, text='Interior Generation Mode:').grid(row=0, sticky='E')
tk.Label( win, text='Granular mode: ').grid(row=1, sticky='E')

tk.Label( win, text='Time Step:', anchor='e').grid(row=2, sticky='E')
tk.Label( win, text='Time End:', anchor='e').grid(row=3, sticky='E')

tk.Label( win, text= 'Friction Coefficients',font=bold_font).grid(row=4,columnspan=2)
tk.Label( win, text='Normal:').grid(row=5, sticky='E')
tk.Label( win, text='Damping:').grid(row=6, sticky='E')
tk.Label( win, text='Rolling:').grid(row=7, sticky='E')
tk.Label( win, text='Spinning:').grid(row=8, sticky='E')

tk.Label( win, text='Compliance Properties',font=bold_font).grid(row=9,sticky='S',columnspan=2)
tk.Label( win, text='Tangential:').grid(row=10, sticky='E')
tk.Label( win, text='Normal:').grid(row=11, sticky='E')
tk.Label( win, text='Rolling:').grid(row=12, sticky='E')
tk.Label( win, text='Spinning:').grid(row=13, sticky='E')

tk.Label( win, text='Robot Properties',font=bold_font).grid(row=14,sticky='S',columnspan=2)
tk.Label( win, text='Mass:', anchor='e').grid(row=15, sticky='E')
tk.Label( win, text='How many:', anchor='e').grid(row=16, sticky='E')
tk.Label( win, text='Height:', anchor='e').grid(row=17, sticky='E')
tk.Label( win, text='Diameter:', anchor='e').grid(row=18, sticky='E')

tk.Label( win, text='Interior Properties',font=bold_font).grid(row=19,sticky='S',columnspan=2)
tk.Label( win, text='Mass:', anchor='e').grid(row=20, sticky='E')
tk.Label( win, text='Diameter:', anchor='e').grid(row=21, sticky='E')

# Column 2
tk.Label( win, text='Visualization Mode:').grid(column=2,row=0, sticky='E')
tk.Label( win, text='Fixed?').grid(column=2,row=1, sticky='E')
tk.Label( win, text='Sim Number:').grid(column=2,row=2, sticky='E')
tk.Label( win, text='Data Path:').grid(column=2,row=3, sticky='E')

tk.Label( win, text='Controller Parameters',font=bold_font).grid(column=2,row=4,columnspan=2)
tk.Label( win, text='Controller Type:', anchor='e').grid(column=2,row=5, sticky='E')
tk.Label( win, text='Target Shape Formation:', anchor='e').grid(column=2,row=6, sticky='E')

tk.Label( win, text='Gains',font=bold_font).grid(column=2,row=7,columnspan=2)
tk.Label( win, text='Alpha:', anchor='e').grid(column=2,row=8, sticky='E')
tk.Label( win, text='Beta:', anchor='e').grid(column=2,row=9, sticky='E')
tk.Label( win, text='Magnitude:', anchor='e').grid(column=2,row=10, sticky='E')

tk.Label( win, text='Ball Forces',font=bold_font).grid(column=2,row=11, columnspan=2)
tk.Label( win, text='Fx:', anchor='e').grid(column=2,row=12, sticky='E')
tk.Label( win, text='Fz:', anchor='e').grid(column=2,row=13, sticky='E')

tk.Label( win, text='Springs',font=bold_font).grid(column=2,row=14, columnspan=2)
tk.Label( win, text='Spring Mode:', anchor='e').grid(column=2,row=15, sticky='E')
tk.Label( win, text='Spring Constant:', anchor='e').grid(column=2,row=16, sticky='E')
tk.Label( win, text='Rest Length:', anchor='e').grid(column=2,row=17, sticky='E')
tk.Label( win, text='Max Rest Length:', anchor='e').grid(column=2,row=18, sticky='E')

tk.Label( win, text='Floor',font=bold_font).grid(column=2,row=19, columnspan=2)
tk.Label( win, text='Length:', anchor='e').grid(column=2,row=20, sticky='E')
tk.Label( win, text='Height:', anchor='e').grid(column=2,row=21, sticky='E')

tk.Label( win, text='Plots',font=bold_font).grid(column=4,row=0, columnspan=2, rowspan=2, sticky='N')

# %% The actual fields and their positions

# Interior modes
intmode = tk.OptionMenu(win, variable1, *intmodes)
intmode.grid(row=0,column=1, sticky='W')

# Granular mode
granmode =  tk.OptionMenu(win,variable2, *granmodes)
granmode.grid(row=1,column=1, sticky='W')

# Time
ttstep = tk.Entry(win,textvariable=tk.StringVar(win,value='2e-3'))
ttstep.grid(row=2,column=1)
ttend = tk.Entry(win,textvariable=tk.StringVar(win,value='1.5'))
ttend.grid(row=3,column=1)

# Friction
muf = tk.Entry (win,textvariable=tk.StringVar(win,value='0.1'))
muf.grid(row=5,column=1)
mub = tk.Entry (win,textvariable=tk.StringVar(win,value='0.01'))
mub.grid(row=6,column=1)
mur = tk.Entry (win,textvariable=tk.StringVar(win,value='0.01'))
mur.grid(row=7,column=1)
mus = tk.Entry (win,textvariable=tk.StringVar(win,value='0.01'))
mus.grid(row=8,column=1)

# Compliance
ct = tk.Entry (win,textvariable=tk.StringVar(win,value='1e-4'))
ct.grid(row=10,column=1)
c = tk.Entry (win,textvariable=tk.StringVar(win,value='1e-5'))
c.grid(row=11,column=1)
cr = tk.Entry (win,textvariable=tk.StringVar(win,value='1e-4'))
cr.grid(row=12,column=1)
cs = tk.Entry (win,textvariable=tk.StringVar(win,value='1e-4'))
cs.grid(row=13,column=1)

# Robot
rmass = tk.Entry (win,textvariable=tk.StringVar(win,value='0.120'))
rmass.grid(row=15,column=1)
rnum = tk.Entry (win,textvariable=tk.StringVar(win,value='75'))
rnum.grid(row=16,column=1)
rheight = tk.Entry (win,textvariable=tk.StringVar(win,value='0.06'))
rheight.grid(row=17,column=1)
rdiam = tk.Entry (win,textvariable=tk.StringVar(win,value='0.035'))
rdiam.grid(row=18,column=1)

# Interior Particles
imass = tk.Entry (win,textvariable=tk.StringVar(win,value='0.03'))
imass.grid(row=20,column=1)
idiam = tk.Entry (win,textvariable=tk.StringVar(win,value='0.035'))
idiam.grid(row=21,column=1)

### Column 2 ###

# Visualization
visual = tk.OptionMenu(win,variable3,*visuals)
visual.grid(row=0,column=3, sticky='W')

# Fixed/Unfixed
fix = tk.OptionMenu(win,variable4,*fixedd)
fix.grid(row=1,column=3, sticky='W')

# Sim Number
simnum = tk.Entry (win)
simnum.grid(row=2,column=3)

# Data Path
datapath = tk.Entry (win,textvariable=tk.StringVar(win,'C:/Chrono/Builds/chrono-develop/bin/data'))
datapath.grid(row=3,column=3)

# Control Type
intmode = tk.OptionMenu(win, variable5, *contmodes)
intmode.grid(row=5,column=3, sticky='W')

# Control shape
intmode = tk.OptionMenu(win, variable6, *conshapes)
intmode.grid(row=6,column=3, sticky='W')

# Gains
alphag = tk.Entry (win,textvariable=tk.StringVar(win,value='100'))
alphag.grid(row=8,column=3)
betag = tk.Entry (win,textvariable=tk.StringVar(win,value='1'))
betag.grid(row=9,column=3)
magg = tk.Entry (win,textvariable=tk.StringVar(win,value='10'))
magg.grid(row=10,column=3)

# Forces on ball
bfx = tk.Entry(win,textvariable=tk.StringVar(win,value='0'))
bfx.grid(row=12,column=3)
bfz = tk.Entry(win,textvariable=tk.StringVar(win,value='0'))
bfz.grid(row=13,column=3)

# Springs
springmode = tk.OptionMenu(win, variable7, *springmodes)
springmode.grid(row=15,column=3, sticky='W')
springk = tk.Entry (win,textvariable=tk.StringVar(win,value='700'))
springk.grid(row=16,column=3)
springrl = tk.Entry (win,textvariable=tk.StringVar(win,value='0'))
springrl.grid(row=17,column=3)
springrlmax = tk.Entry (win,textvariable=tk.StringVar(win,value='0.003'))
springrlmax.grid(row=18,column=3)

# Floor
floorlen = tk.Entry (win,textvariable=tk.StringVar(win,value='40')) 
floorlen.grid(row=20,column=3)
floortall = tk.Entry (win,textvariable=tk.StringVar(win,value='1'))
floortall.grid(row=21,column=3)

### Column 3 ###

# Plots
plotpos = tk.BooleanVar(value=False)
posplot = tk.Checkbutton(win,text='Position',variable=plotpos)#,onvalue='True',offvalue='False')
posplot.grid(row=1,column=4, sticky='W')

plotvel = tk.BooleanVar(value=False)
velplot = tk.Checkbutton(win,text='Velocity',variable=plotvel)#,onvalue='True',offvalue='False')
velplot.grid(row=2,column=4, sticky='W')

plotfor = tk.BooleanVar(value=False)
forplot = tk.Checkbutton(win,text='Bot Forces',variable=plotfor)#,onvalue='True',offvalue='False')
forplot.grid(row=3,column=4, sticky='W')

plotcon = tk.BooleanVar(value=False)
conplot = tk.Checkbutton(win,text='Control Forces',variable=plotcon)#,onvalue='True',offvalue='False')
conplot.grid(row=4,column=4, sticky='W')

ploterr = tk.BooleanVar(value=False)
errplot = tk.Checkbutton(win,text='Control Error',variable=ploterr)#,onvalue='True',offvalue='False')
errplot.grid(row=5,column=4, sticky='W')

### Column 4 ###

plotspr = tk.BooleanVar(value=False)
sprplot = tk.Checkbutton(win,text='Spring Positions',variable=plotspr)#,onvalue='True',offvalue='False')
sprplot.grid(row=1,column=5, sticky='W')

plotcont = tk.BooleanVar(value=False)
contplot = tk.Checkbutton(win,text='Contact Forces',variable=plotcont)#,onvalue='True',offvalue='False')
contplot.grid(row=2,column=5, sticky='W')

plotptpos = tk.BooleanVar(value=False)
ptposplot = tk.Checkbutton(win,text='Particle Positions',variable=plotptpos)#,onvalue='True',offvalue='False')
ptposplot.grid(row=3,column=5, sticky='W')

plotptvel = tk.BooleanVar(value=False)
ptvelplot = tk.Checkbutton(win,text='Particle Velocities',variable=plotptvel)#,onvalue='True',offvalue='False')
ptvelplot.grid(row=4,column=5, sticky='W')

plotptfor = tk.BooleanVar(value=False)
ptforplot = tk.Checkbutton(win,text='Particle Forces',variable=plotptfor)#,onvalue='True',offvalue='False')
ptforplot.grid(row=5,column=5, sticky='W')

# %% User actions
photo1=PhotoImage(file='plot_plots.png') 
plot_plots=tk.Button(win, text='Plot Plots!', font=bold_font, anchor='s', \
                     height=100, width=100 ,image=photo1, command=usrinputs.plot)
plot_plots.grid(row=6, column=4, columnspan=2, rowspan=5)

photo2=PhotoImage(file='run_jameoba.png')
runsim=tk.Button(win, text='Run Jamoeba!', font=bold_font, anchor='s', image=photo2,command=usrinputs.run_jameboa)
runsim.grid( row=11, column=4, columnspan=2, rowspan=8 )

photo3=PhotoImage(file='end_sim.png')
endsim=tk.Button(win,text='End Sim!\n', font=bold_font,anchor='s', image=photo3, height=100, width=100)
endsim.grid( row=19, column=4, columnspan=2, rowspan=3)

# %% Tooltips

run_tip=CreateToolTip(runsim,'Run the simulation')

end_tip=CreateToolTip(endsim,"This button doesn't do anything! \n \n"\
                              "Hit x on the Chrono application window "\
                                "to end the simulation. \n \n" \
                                    "You can then use the run button to create another simulation")
tk.mainloop()