""""
plotters.py: 
    
This file contains the classes and
defs for plotting of simulation data

Author: Qiyuan Zhou
Date (MM/DD/YY): 09/13/20
Wavelab | Illinois Institute of Technology
"""
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from itertools import count
import pychrono as chrono
plt.ioff()

class plotter:
    def __init__(self, robots,config):
        self.cf=config
        self.robots=robots
        self.time=[]
        self.outdir=self.cf.sim_id+'/save_data/'
        if not os.path.exists(self.outdir):
            os.makedirs(self.outdir)
        
        ### Arrays to store data, shared b/t live and post plots ###
        
        # IMU ground truth, data (x accel, z accel, gyro)
        if(self.cf.save_sens): self.sens=np.zeros(6*len(robots))
        
        # Position ground truth and prediction for stacked subplots (x,z,theta)
        if(self.cf.save_pred[0]): self.pred_s=np.zeros(6*len(robots))
        
        # Position prediction for overview plot (x,z)
        if(self.cf.save_pred[1]): self.pred_o=np.zeros(2*len(robots))
        
        # Predicted distance and real distance subplots (L,R)
        if(self.cf.save_pred_d): self.pred_d=np.zeros(4*len(robots))
        
        # Prediction errors (x,z,theta)
        if(self.cf.save_pred_err): self.pred_err=np.zeros(3*len(robots))
        
        # Position info global from chrono (x,y,z)
        if(self.cf.save_pos or self.cf.live_pos or self.cf.save_pred[1]): self.pos=np.zeros(3*len(robots))
        
        # Position info local frame (x,y,z)
        if(self.cf.save_lpos): self.lpos=np.zeros(3*len(robots))
        
        # Position info global, reconstructed from local frame
        if(self.cf.save_lposa): self.lposa=np.zeros(3*len(robots))
        
        # Velocity info (x,y,z)
        if(self.cf.save_vel or self.cf.live_vel[0]): self.vel=np.zeros(2*len(robots))
            
        # Distance info
        if(self.cf.save_dist or self.cf.live_dist[0]): self.dist=np.zeros(len(robots)*(len(robots)-1))
        
        # Spring tensions (L,R)
        if(self.cf.save_tens or self.cf.live_tens[0]): self.tens=np.zeros(2*len(robots))
        
        # Contact info
        if(self.cf.save_cont or self.cf.live_cont[0]): self.cont=[]
        
        # Control inputs mag*(x,z)
        if(self.cf.save_control or self.cf.live_control[0]): self.control=np.zeros(2*len(robots))
        
        # Area 
        if(self.cf.save_area or self.cf.live_area[0]): self.area=[]
        
        # Convexity
        if(self.cf.save_conv or self.cf.live_conv[0]): self.conv=[]
        
        # Internal pressure
        if(self.cf.save_pres or self.cf.live_pres[0]): self.pres=[]
        
        # Object grasping mapping (x,z) sink coordinates, contact coordinates (local)
        if(self.cf.save_lmap or self.cf.live_map[0]): 
            self.lmap=np.zeros(2); self.gtmap=np.zeros(2)
            self.posmap = np.zeros((2*len(robots),2)); self.posmapa = np.zeros((2*len(robots),2))
            if not os.path.exists(self.outdir+'lmap_frames/'):
                os.makedirs(self.outdir+'lmap_frames/')
            
        # Object grasping mapping (x,z) sink coordinates, contact coordinates (global)
        if(self.cf.save_gmap or self.cf.live_map[0]): 
            self.gmap=np.zeros(2); self.posmap = np.zeros((2*len(robots),2))
            if not os.path.exists(self.outdir+'gmap_frames/'):
                os.makedirs(self.outdir+'gmap_frames/')
        
    # Convert quaternion to euler angle about y axis
    def quat2eulerCh(self,quat):
        return quat.Q_to_Rotv().y
    
    def quat2eulerV(self,x,y,z,w):
        quat = chrono.ChQuaternionD(x,y,z,w)
        
        return quat.Q_to_Rotv().y
    
    # Upper level data collecting def
    def collect(self,time,area,convexity,pressure,contacts,elliptic):
        self.time.append(time)
        self.contacts=contacts
        self.elliptic = elliptic
        
        ## Save post-run plots
        
        # Sensor measurements vs ground truth
        if(self.cf.save_sens): self.get_sens()
        
        # Predcited positions and orientation (subplots)
        if(self.cf.save_pred[0]): self.get_pred_s()
        
        # Predicted positions (overview)
        if(self.cf.save_pred[1]): self.get_pred_o()
        
        # Predicted distance and real distance subplots (L,R)
        if(self.cf.save_pred_d): self.get_pred_d()
        
        # Prediction errors (x,z,theta)
        if(self.cf.save_pred_err): self.get_pred_err()
        
        # Position info global from chrono (x,y,z)
        if(self.cf.save_pos or self.cf.live_pos[0] or self.cf.save_pred[1]): self.get_pos()
        
        # Position info local frame (x,y,z)
        if(self.cf.save_lpos): self.get_lpos()
        
        # Position info global, reconstructed from local frame
        if(self.cf.save_lposa): self.get_lposa()
        
        # Velocity info (x,y,z,mag)
        if(self.cf.save_vel or self.cf.live_vel[0]): self.get_vel()
        
        # Distance info
        if(self.cf.save_dist or self.cf.live_dist[0]): self.get_dist()
        
        # Spring tensions (L,R)
        if(self.cf.save_tens or self.cf.live_tens[0]): self.get_tens()
        
        # Contact info
        if(self.cf.save_cont or self.cf.live_cont[0]): self.get_cont()
        
        # Control inputs (x,z,mag)
        if(self.cf.save_control or self.cf.live_control[0]): self.get_control()
        
        # Area
        if(self.cf.save_area or self.cf.live_area[0]): self.area.append(area)
        
        # Convexity
        if(self.cf.save_conv or self.cf.live_conv[0]): self.conv.append(convexity)
        
        # Internal pressure
        if(self.cf.save_pres or self.cf.live_pres[0]): self.pres.append(pressure)
        
        # Object mapping (local)
        if(self.cf.save_lmap or self.cf.live_map[0]): self.get_lmap()
        
        # Object mapping (global)
        if(self.cf.save_gmap or self.cf.live_map[0]): self.get_gmap()
            
    # Update data arrays
    def get_sens(self):
        temprow=[]
        for robot in self.robots:
            xt,zt,tt,x,z,t = robot.save_sens()
            temprow.append(xt)
            temprow.append(zt)
            temprow.append(tt)
            temprow.append(x)
            temprow.append(z)
            temprow.append(t)
        temprow=np.asarray(temprow)
        self.sens=np.vstack((self.sens,temprow))
        
    def get_pred_s(self):
        temprow=[]
        for robot in self.robots:
            rot = robot.theta
            X = robot.kalman.X
            temprow.append(robot.body.GetPos().x)
            temprow.append(robot.body.GetPos().z)
            temprow.append((180/np.pi)*self.quat2eulerCh(rot))
            #temprow.append(self.quat2eulerCh(rot))
            temprow.append(robot.kalman.X[0][0])
            temprow.append(robot.kalman.X[2][0])
            temprow.append((180/np.pi)*self.quat2eulerV(X[4][0],X[5][0],X[6][0],X[7][0]))
            #temprow.append(self.quat2eulerV(X[4][0],X[5][0],X[6][0],X[7][0]))
        temprow=np.asarray(temprow)
        self.pred_s=np.vstack((self.pred_s,temprow))
        
    def get_pred_o(self):
        temprow=[]
        for robot in self.robots:
            temprow.append(robot.kalman.X[0][0])
            temprow.append(robot.kalman.X[2][0])
        temprow=np.asarray(temprow)
        self.pred_o=np.vstack((self.pred_o,temprow))
        
    def get_pred_d(self):
        temprow=[]
        for robot in self.robots:
            temprow.append(robot.distR[1])        # Ref Right
            temprow.append(robot.distR[0])        # Ref Left
            temprow.append(robot.distE[1])        # Pred Right
            temprow.append(robot.distE[0])        # Pred left
        temprow=np.asarray(temprow)
        self.pred_d=np.vstack((self.pred_d,temprow))
        
    def get_pred_err(self):
        temprow=[]
        for robot in self.robots:
            rot = robot.theta
            X = robot.kalman.X
            temprow.append(robot.body.GetPos().x-robot.kalman.X[0][0])
            temprow.append(robot.body.GetPos().z-robot.kalman.X[2][0])
            temprow.append((180/np.pi)*(self.quat2eulerCh(rot)-self.quat2eulerV(X[4][0],X[5][0],X[6][0],X[7][0])))
        temprow=np.asarray(temprow)
        self.pred_err=np.vstack((self.pred_err,temprow))
        
    def get_pos(self):
        temprow=[]
        for robot in self.robots:
            x,y,z=robot.save_pos()
            temprow.append(x)
            temprow.append(y)
            temprow.append(z)
        temprow=np.asarray(temprow)
        self.pos=np.vstack((self.pos,temprow))
        
    def get_lpos(self):
        temprow=[]
        for robot in self.robots:
            temprow.append(robot.local_pos[0])
            temprow.append(0)
            temprow.append(robot.local_pos[1])
        temprow=np.asarray(temprow)
        self.lpos=np.vstack((self.lpos,temprow))
        
    def get_lposa(self):
        temprow=[]
        x,y,z = self.robots[0].save_pos()
        for robot in self.robots:
            t=robot.coord_rot
            if(t==None): t=0
            x1=robot.local_pos[0]
            y1=robot.local_pos[1]
            temprow.append(x1*np.cos(t)-y1*np.sin(t)+x)
            temprow.append(y)
            temprow.append(x1*np.sin(t)+y1*np.cos(t)+z)
        temprow=np.asarray(temprow)
        self.lposa=np.vstack((self.lposa,temprow))
                                 
    def get_vel(self):
        temprow=[]
        for robot in self.robots:
            x,y,z,mag=robot.save_vel()
            temprow.append(x)
            temprow.append(z)
        temprow=np.asarray(temprow)
        self.vel=np.vstack((self.vel,temprow))
        
    def get_dist(self):
        temprow=[]
            
    def get_tens(self):
        temprow=[]
        for robot in self.robots:
            temprow.append(robot.tension[0])
            temprow.append(robot.tension[1])
        temprow=np.asarray(temprow)
        self.tens=np.vstack((self.tens,temprow))
    
    def get_control(self):
        temprow=[]
        for robot in self.robots:
            x,z=robot.save_control()
            temprow.append(x)
            temprow.append(z)
        temprow=np.asarray(temprow)
        self.control=np.vstack((self.control,temprow))
        
    # Save only the contacts for all the time steps
    def get_lmap(self):
        for index in self.contacts[:,0]:
            # Estimated contact locations
            self.lmap=np.vstack((self.lmap,self.robots[index].local_pos))
            
            # Actual contact locations
            pos = np.asarray([self.robots[index].body.GetPos().x,self.robots[index].body.GetPos().z])
            self.gtmap=np.vstack((self.gtmap,pos))
                
        temprow1=[]; temprow2=[]; temprow3=[]; temprow4=[]
        for robot in self.robots:
            temprow1.append(robot.local_pos[0])
            temprow2.append(robot.local_pos[1])
            temprow3.append(robot.body.GetPos().x)
            temprow4.append(robot.body.GetPos().z)
        temprow=np.column_stack((temprow1,temprow2))
        temp_row = np.column_stack((temprow3,temprow4))
        self.posmap=np.vstack((self.posmap,temprow))
        self.posmapa=np.vstack((self.posmapa,temp_row))
        
    # Save only the contacts for all the time steps
    def get_gmap(self):
        t=self.robots[0].coord_rot
        rob=np.asarray([self.robots[0].body.GetPos().x,self.robots[0].body.GetPos().z])
        trans=np.asarray([[np.cos(t), -np.sin(t)],[np.sin(t), np.cos(t)]])
        
        for index in self.contacts[:,0]:
            self.gmap=np.vstack((self.gmap,trans.dot(self.robots[index].local_pos)+rob))
            
        temprow1=[]; temprow2=[]
        for robot in self.robots:
            x,y,z=robot.save_pos()
            temprow1.append(x)
            temprow2.append(z)
        temprow=np.column_stack((temprow1,temprow2))
        self.posmap=np.vstack((self.posmap,temprow))

    # Save data arrays to disk
    def save_to_disk(self):
        np.save(self.outdir+'time.npy',np.asarray(self.time))
        if(self.cf.save_sens):       # Save sensor ground truth and measurement
            self.sens=self.sens[1:,:]
            np.save(self.outdir+'sens.npy',self.sens)
        if(self.cf.save_pred[0]):    # Save kalman ground truth vs predictions (subplots)
            self.pred_s=self.pred_s[1:,:]
            np.save(self.outdir+'pred_s.npy',self.pred_s)
        if(self.cf.save_pred[1]):    # Save kalman predictions (overview)
            self.pred_o=self.pred_o[1:,:]
            np.save(self.outdir+'pred_o.npy',self.pred_o)
        if(self.cf.save_pred_d):     # Save spring distance predictions
            self.pred_d=self.pred_d[1:,:]
            np.save(self.outdir+'pred_d.npy',self.pred_d)
        if(self.cf.save_pred_err):     # Save prediction errors
            self.pred_err=self.pred_err[1:,:]
            self.pred_err[:,0:-1:3]=100.*self.pred_err[:,0:-1:3]
            self.pred_err[:,1:-1:3]=100.*self.pred_err[:,1:-1:3]
            np.save(self.outdir+'pred_err.npy',self.pred_err)
        if(self.cf.save_pos or self.cf.save_pred[1]):        # Save global position info from chrono (x,y,z)
            self.pos=self.pos[1:,:]
            np.save(self.outdir+'pos.npy',self.pos)
        if(self.cf.save_lpos):       # Save local position info (x,y,z)
            self.lpos=self.lpos[1:,:]
            np.save(self.outdir+'lpos.npy',self.lpos)
        if(self.cf.save_lposa):      # Save global reconstructed position info (x,y,z)
            self.lposa=self.lposa[1:,:]
            np.save(self.outdir+'lposa.npy',self.lposa)
        if(self.cf.save_vel):        # Save velocity info (x,y,z,mag)
            self.vel=self.vel[1:,:]    
            np.save(self.outdir+'vel.npy',self.vel)
        if(self.cf.save_dist):       # Save distance info
            self.dist=self.dist[1:,:]  
            np.save(self.outdir+'dist.npy',self.dist)
        if(self.cf.save_tens):       # Save spring tensions (L,R)
            self.tens=self.tens[1:,:]  
            np.save(self.outdir+'tens.npy',self.tens)
        if(self.cf.save_cont):       # Save contact info
            self.cont=self.cont[1:,:] 
            np.save(self.outdir+'cont.npy',self.cont)
        if(self.cf.save_control):    # Save control inputs (x,z,mag)
            self.control=self.control[1:,:] 
            np.save(self.outdir+'control.npy',self.control)
        if(self.cf.save_area):       # Save area
            self.area=self.area[1:]
            np.save(self.outdir+'area.npy',self.area)
        if(self.cf.save_conv):       # Save convexity
            self.conv=self.conv[1:]
            np.save(self.outdir+'conv.npy',self.conv)
        if(self.cf.save_pres):       # Save pressure
            self.pres=self.pres[1:]
            np.save(self.outdir+'pres.npy',self.pres)
        
    # Create live plots
    def make_live_plots(self):
        outdir=self.cf.sim_id+'/live_animations/'
        if not os.path.exists(outdir):
            os.makedirs(outdir)
            
        # Plot position info (x,y,z)
        if(self.cf.live_pos[0]):        
            index_pos=count()
            
        # Plot velocity info (x,y,z,mag)
        if(self.cf.live_vel[0]):        
            index_vel=count()
            
        # Plot distance info
        if(self.cf.live_dist[0]):       
            index_dist=count()
            
        # Plot spring tensions (L,R)
        if(self.cf.live_tens[0]):       
            index_tens=count()
        
        # Plot contact info
        if(self.cf.live_cont[0]):       
            index_cont=count()
            
        # Plot control inputs (x,z,mag)
        if(self.cf.live_control[0]):    
            index_control=count()
            
        # Plot areas
        if(self.cf.live_area[0]): 
            index_area=count()
            xs_area=[]; ys_area=[]
            fig_area=plt.figure()
            ax_area=fig_area.add_subplot(1,1,1)
            self.ani_area=animation.FuncAnimation(fig_area, self.animate_line, blit=False, interval=self.cf.live_interval,\
                                            fargs=(index_area,ax_area,xs_area,ys_area,'area','Area','area [m^2]'))
            if(self.cf.live_area[1]): self.ani_area.save(outdir+'area.mp4',writer='ffmpeg',fps=15)
        
        # Plot convexity
        if(self.cf.live_conv[0]): 
            index_conv=count()
            xs_conv=[]; ys_conv=[]
            fig_conv=plt.figure()
            ax_conv=fig_conv.add_subplot(1,1,1)
            self.ani_conv=animation.FuncAnimation(fig_conv, self.animate_line, blit=False, interval=self.cf.live_interval,\
                                            fargs=(index_conv,ax_conv,xs_conv,ys_conv,'conv','Convexity','convexity [1]'))
            if(self.cf.live_conv[1]): self.ani_conv.save(outdir+'convexity.mp4',writer='ffmpeg',fps=15)
        
        # Plot internal pressure
        if(self.cf.live_pres[0]): 
            index_pres=count()
            xs_pres=[]; ys_pres=[]
            fig_pres=plt.figure()
            ax_pres=fig_pres.add_subplot(1,1,1)
            self.ani_pres=animation.FuncAnimation(fig_pres, self.animate_line, blit=False, interval=self.cf.live_interval,\
                                            fargs=(index_pres,ax_pres,xs_pres,ys_pres,'pres','Pressure','pressure [Pa]'))
            if(self.cf.live_pres[1]): self.ani_pres.save(outdir+'pressure.mp4',writer='ffmpeg',fps=15)
            
        # Plot boundary mapping
        if(self.cf.live_map[0]):
            fig_map = plt.figure()
            plot_map,= plt.plot([],[],'o')
            self.ani_map=animation.FuncAnimation(fig_map, self.animate_scatter,fargs=(plot_map))
        
    # Create and save post-processing plots
    def make_post_plots(self):
        outdir=self.cf.sim_id+'/save_data/'
        
        # Plot sensor ground truth and measurements (x,z,theta)
        if(self.cf.save_sens):
            ylabel=['m/s^2','m/s^2','rad/s']
            title=["X''","Z''","Theta'"]
            self.post_sub_plots_overlap(6,2,outdir,'IMU',self.sens,title,ylabel)
            
        # Plot ground truth positions and predictions subplots(x,z,theta)
        if(self.cf.save_pred[0]):
            ylabel=['m','m','deg']
            title=['X','Z','Theta']
            self.post_sub_plots_overlap(6,2,outdir,'Positions',self.pred_s,title,ylabel)
            
        # Plot ground truth positions and predictions overlay (x,z)
        if(self.cf.save_pred[1]):
            self.post_pos_plots(outdir,'global',self.pos,'Global Positions',self.pred_o)
        
        # Plot spring distances and estimates
        if(self.cf.save_pred_d):
            ylabel=['m','m']
            title=['Left','Right']
            self.post_sub_plots_overlap(4,2,outdir,'Spring Lengths',self.pred_d,title,ylabel)
            
        # Plot prediction error
        if(self.cf.save_pred_err):        
            ylabel=['Error [cm]','Error [cm]','Error [deg]']
            title=['Error X','Error Y', 'Error theta']
            self.post_sub_plots(1, 3, outdir,'Prediction Errors',self.pred_err,title,ylabel)
            
        # Plot global position info from chrono (x,y,z)
        if(self.cf.save_pos):                    
            self.post_pos_plots(outdir,'global',self.pos,'Global Positions')
            
        # Plot local position info (x,y,z)
        if(self.cf.save_lpos):                    
            self.post_pos_plots(outdir,'global',self.lpos,'Local Positions')
            
        # Plot reconstructed global position info from chrono (x,y,z)
        if(self.cf.save_lposa):                    
            self.post_pos_plots(outdir,'global',self.lposa,'Reconstructed Global Positions')
            
        # Plot velocity info (x,z)
        if(self.cf.save_vel):        
            ylabel=['Velocity [m/s]','Velocity [m/s]']
            title=['X','Z']
            self.post_sub_plots(1, 2, outdir,'Velocity',self.vel,title,ylabel)
            
        # Plot distance info
        if(self.cf.save_dist):       
            np.save(outdir+'dist.npy',self.dist)
            
        # Plot spring tensions (L,R)
        if(self.cf.save_tens):       
            ylabel=['Tension [N]','Tension [N]']
            title=['Left','Right']
            self.post_sub_plots(1, 2, outdir,'Tension',self.tens,title,ylabel)
        
        # Plot contact info
        if(self.cf.save_cont):       
            np.save(outdir+'cont.npy',self.cont)
            
        # Plot control inputs (x,z,mag)
        if(self.cf.save_control):    
            ylabel=['Control Force [N]','Control Force [N]']
            title=['X','Z']
            self.post_sub_plots(1, 2, outdir,'Control Force',self.control,title,ylabel)
            
        # Plot areas
        if(self.cf.save_area): self.post_single_plots(outdir,'Area',self.area,'Area [m]')
        
        # Plot convexity
        if(self.cf.save_conv): self.post_single_plots(outdir,'Convexity',self.conv,'Convexity')
        
        # Plot internal pressure
        if(self.cf.save_pres): self.post_single_plots(outdir,'Internal Pressure',self.pres,'Pressure [Pa]')
            
    # Upper level update def for plots with frames saved at each step
    def save_plot_frames(self,step):
        if(self.cf.save_lmap):
            self.save_mapping(step,1,'x pos [m]','y pos [m]','Obstacle Mapping (Local)')
            
        if(self.cf.save_gmap):
            self.save_mapping(step,2,'x pos [m]','y pos [m]','Obstacle Mapping (Global)')
        
    # Def used to create overlapping subplots
    def post_sub_plots_overlap(self,dimsz,nstack,outdir,figname,data,title,ylab):
        
        # Each set of plots gets its own foler
        if not os.path.exists(outdir+'/'+figname+'_subplots/'):
            os.makedirs(outdir+'/'+figname+'_subplots/')
            
        # Each bot gets its own figure
        for i in range(len(self.robots)):
            fig, axs = plt.subplots(len(ylab),sharex=True,dpi=150)
            fig.suptitle(figname+' Bot ' + str(i))
            
            # Cycle through each dimension of each bot's data
            for j in range(len(ylab)):
                axs[j].set_title(title[j])
                axs[j].set_ylabel(ylab[j])
                for k in range(nstack):
                    axs[j].plot(self.time,data[:,i*dimsz+j+k*int(dimsz/nstack)])
                    # if k%2==0: axs[j].plot(self.time,data[:,i*dimsz+j+k*int(dimsz/nstack)])
                    # if k%2==1: axs[j].scatter(self.time,data[:,i*dimsz+j+k*int(dimsz/nstack)])
                    
            # Save each robot's plot
            fig.tight_layout(pad=2)
            fig.text(0.5, 0.04, 'Time[s]', ha='center')
            plt.close()
            fig.savefig(outdir+'/'+figname+'_subplots/'+'bot'+str(i)+self.cf.save_format,dpi=150)
            
    # Def used for making plots for each robot, with subplots for each data column        
    def post_sub_plots(self,mode,dimsz,outdir,figname,data,title,ylab):
        plt.ioff()
        # Each set of plots gets its own folder
        if not os.path.exists(outdir+'/'+figname+'_subplots/'):
            os.makedirs(outdir+'/'+figname+'_subplots/')
            
        # Each bot gets its own figure
        for i in range(len(self.robots)):
            # Mode==1: each dimension of data gets its own subplot
            if (mode==1):
                fig, axs = plt.subplots(len(ylab),sharex=True,dpi=150)
            # Mode==2: each dimension of data plotted on top of each other
            if (mode==2):
                fig, axs = plt.subplots(1,sharex=True,dpi=150)
            fig.suptitle(figname+' Bot ' + str(i))
            
            # Cycle through each dimension of each bot's data
            if mode==1:
                for j in range(len(ylab)):
                    axs[j].set_title(title[j])
                    axs[j].set_ylabel(ylab[j])
                    axs[j].plot(self.time,data[:,i*dimsz+j])
            if mode==2:
                for j in range(len(ylab)):
                    axs[j].plot(self.time,data[:,j])
                
            # Save each robot's plot
            fig.tight_layout(pad=2)
            fig.text(0.5, 0.04, 'Time[s]', ha='center')
            plt.close()
            fig.savefig(outdir+'/'+figname+'_subplots/'+'bot'+str(i)+self.cf.save_format,dpi=150)
            
    # Def used to make position trajectory plots
    def post_pos_plots(self,outdir,figname,data,title,data2=np.zeros(2)):
        plt.ioff()
        if not os.path.exists(outdir+'/'+figname+'_positions/'):
            os.makedirs(outdir+'/'+figname+'_positions/')
            
        data[abs(data)<=5e-4]=np.nan
        
        fig, axs = plt.subplots(1,sharex=True,dpi=150)
        fig.suptitle(title)
        
        for i in range(len(self.robots)):
            plt.plot(data[1:-1,i*3],data[1:-1,i*3+2],label='Robot %s'%i)
            if (len(np.shape(data2))>1):
                plt.plot(data[1:-1,i*2],data[1:-1,i*2+1])
            axs.axis('equal')
            
        fig.tight_layout(pad=2)
        fig.text(0.5,0.04,'X Position [m]', ha='center')
        axs.set_ylabel('Y position [m]')
        axs.legend()
    
    # Def used for making plots which have a single quantity
    def post_single_plots(self, outdir, figname, data, ylabel):
        plt.ioff()
        fig=plt.figure()
        plt.plot(self.time[1:],data)
        plt.tight_layout()
        plt.xlabel('Time [s]')
        plt.ylabel(ylabel)
        plt.title(figname)
        plt.close()
        plt.savefig(outdir+'/'+figname+self.cf.save_format,dpi=150)

    # Def used to create ufunc for animated line graphs for a single quantity
    def animate_line(self,i,index,ax,xs,ys,ymode,title,ylabel):
        counter=next(index)
        
        # Update data arrays
        xs=self.time
        
        if(ymode=='area'):
            ys=self.area
        if(ymode=='conv'):
            ys=self.conv
        if(ymode=='pres'):
            ys=self.pres
        
        #Trim axis and data arrays
        if counter>self.cf.live_npts:
            ax.clear()
            xs=xs[-self.cf.live_npts:-1]
            ys=ys[-self.cf.live_npts:-1]
            
        if(ymode=='conv'):
            plt.ylim(0, 1)
        
        plt.tight_layout()
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.plot(xs, ys, 'b-')
        
    # Def used to create ufunc for animated bar graphs for a single quantity
    def animate_bars(self,i,index,ax,xs,ys,ymode,title,ylabel):
        a=1
    
    # Def used to create ufunc for animated scatter graphs for things like position
    def animate_scatter(self,i,plot):
        
        # Stack the data we're plotting
        data=np.vstack(self.sink,self.map)
        
        # Plot the data
        plot.set_data(data[:,0], data[:,1])
        
        plt.tight_layout()
        plt.xlabel('Position X [m]')
        plt.ylabel('Position Y [m]')
        plt.title('Mapped contacts')
        return plot
    
    # Def used to export data arrays for mapping at each time step
    def save_mapping(self,step,mode,xlabel,ylabel,title):
        
        # Plotting local contact mapping
        if(mode==1):
            data1=self.lmap
            outdir=self.outdir+'lmap_frames/'
            data3 = self.gtmap
            
            np.save(outdir + 'gtmap' + str(step) + '.npy',data3)
        
        # Plotting global contact mapping
        if(mode==2):
            data1=self.gmap
            outdir=self.outdir+'gmap_frames/'
        
        data2 = self.posmap
        data4 = self.posmapa
        data5 = np.asarray((self.elliptic.xt,self.elliptic.yt)).T

        np.save(outdir + 'map' + str(step) + '.npy',data1)
        np.save(outdir + 'efd' + str(step) + '.npy',data5)
            
        if len(data2[:,0]>100*self.cf.rob_nb):
            np.save(outdir + 'pos' + str(step) + '.npy',data2[-100*self.cf.rob_nb:,:])
            np.save(outdir + 'posa' + str(step) + '.npy',data4[-100*self.cf.rob_nb:,:])
        else:
            np.save(outdir + 'pos' + str(step) + '.npy',data2)
            np.save(outdir + 'posa' + str(step) + '.npy',data4)
            