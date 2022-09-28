# -*- coding: utf-8 -*-
"""
"""

import numpy as np
from numpy import cos, sin
import os
import glob
from math import degrees
import matplotlib.pyplot as plt
from matplotlib import animation
from IPython.display import HTML
import cv2
import matplotlib.font_manager as fm
from shutil import copyfile
import matplotlib.font_manager as fm
import time
from tqdm import tqdm
# Rebuild the matplotlib font cache
# fm._rebuild()
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size'] = 6
plt.rcParams['axes.linewidth'] = .1



class GrabbingDataManager:
    def __init__(self, data_loc,
                botRadius=0.025, skinRadius=0.015, inRadius=0.025, square_length=0.185,
                wxmin=0, wxmax=1.4, wymin=1.5, wymax=2.2,
                fig_width= 2.5
                ):
        """
        Class which takes data of a simulation and creates frames.
        Methods are also available to create a video with the data

        data_loc: Data_Location
        botRadius: Bot radius in simulation
        skinRadius: Skin radius in simulation
        inRadius: Radius of interior particles in simulation
        wxmin: Lowest x-value of environment in simulation
        wxmax: Highest x-value of environment in simulation
        wymin: Lowest y-value of environment in simulation
        wymax: Highest y-value of environment in simulation
        """
        self.data_loc=data_loc
        
        path = os.path.dirname(__file__)
        os.chdir(path)

        data_loc = os.path.join(path, self.data_loc)
        if data_loc[-1] != '/': data_loc += '/'

        # Loading bot positions
        bot_positions = np.load(data_loc + 'bot_coords.npy')
        _, nb = bot_positions.shape
        self.nb=int((nb-1)/2)
        self.Time = bot_positions[:,0]
        self.bot_position_x = bot_positions[:,1:2*self.nb+1:2]
        self.bot_position_y = bot_positions[:,2:2*self.nb+1:2]
        
        # Loading interior particle positions
        in_particle_positions=np.load(data_loc + 'interior_coords.npy')
        _, self.ni = in_particle_positions.shape
        self.ni=int((self.ni-1)/2)
        self.particle_position_x=in_particle_positions[:,1:2*self.ni+1:2]
        self.particle_position_y=in_particle_positions[:,2:2*self.ni+1:2]
        
        # Loading skin particle positions
        skin_positions=np.load(data_loc + 'skin_coords.npy')
        _, ns = skin_positions.shape
        self.ns=int((ns-1)/2)
        self.skin_position_x = skin_positions[:,1:2*self.ns+1:2]
        self.skin_position_y=skin_positions[:,2:2*self.ns+1:2]
        
        # Loading object data
        self.square_position=np.load(data_loc + 'square_loc.npy')
        
        self.slot_Points=np.load(data_loc + 'slot Points.npy')
        # Loading target location data
        self.data5=np.load(data_loc + 'target_loc.npy')
        self.xtarget=self.data5[0]
        self.ytarget=self.data5[1]
        
        # Setting system parameters
        self.botRadius=botRadius
        self.skinRadius=skinRadius
        self.inRadius=inRadius
        self.squareLength = square_length
        
        # Setting parameters for plotting
        self.wxmin=wxmin
        self.wxmax=wxmax
        self.wymin=wymin
        self.wymax=wymax
        self.fx=fig_width
        # self.fy=fig_width
        self.fy=self.fx*((self.wymax-self.wymin)/(self.wxmax-self.wxmin))



    def create_animation(self, save_loc = 'frames/', video_name="NO_NAME"):
        """
        Creates frames for making a MatPlotLib animation of the system

        save_loc: Location to save the frames used to create video
        """
        os.makedirs(save_loc, exist_ok=True)
            
        count=0
        for i, Time in enumerate(tqdm(self.Time)): 
            fig = plt.figure(dpi=300)
            fig.set_size_inches(self.fx, self.fy)
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
            plt.tight_layout()
            fig.subplots_adjust(top=0.85,
                                bottom=0.191,
                                left=0.21,
                                right=0.919,
                                hspace=0.2,
                                wspace=0.2)
            
            # Plotting the bots
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[i,j],self.bot_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.botRadius, fc='black')
                ax.add_patch(patch)
                
            # Plotting the interior particles
            for j in range(0,self.ni):
                x0,y0=self.particle_position_x[i,j],self.particle_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.inRadius, fc='tab:green')
                ax.add_patch(patch)    
                
            # Plotting the skin radii
            for j in range(0,self.ns):
                x0,y0=self.skin_position_x[i,j],self.skin_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.skinRadius, fc='tab:red')
                ax.add_patch(patch)    
                    
            # Plotting the object
            x, y, angle = self.square_position[i,1], self.square_position[i,2], self.square_position[i,3]
            square_pos = np.array([x,y])
            # Note angle is in RADIANS

            # We have to get the properly rotated object in-order to plot it
            c = cos(angle)
            s = sin(angle)
            R = np.array([[c, -s], [s, c]])
            T = np.array([-self.squareLength/2, -self.squareLength/2]) # Square translation matrix

            # Rotate, translate, and re-rotate the square. We need to get the new coordinates!
            square_pos_prime = np.matmul(np.linalg.inv(R), square_pos)
            square_pos_prime += T
            square_pos_prime = np.matmul(R, square_pos_prime)
            x,y = square_pos_prime

            # Plot the square
            patch = plt.Rectangle((x,y), self.squareLength, self.squareLength, angle=degrees(angle), fc='tab:gray')
            ax.add_patch(patch)

            # Plotting the target
            ax.plot(self.xtarget, self.ytarget,"*", c='tab:red',markersize=.1)   

            # Making the plot cleaner 
            plt.title('Time= ' + str(np.round(Time,2)),fontsize=8,pad=-1)
            x_ticks = np.linspace(self.wxmin, self.wxmax, 5, endpoint=True)
            y_ticks = np.linspace(self.wymin, self.wymax, 5, endpoint=True)
            ax.set_xticks(np.round(x_ticks,2))
            ax.set_yticks(np.round(y_ticks,2))
            #ax.xaxis.set_tick_params(width=.25,length=2)
            #ax.yaxis.set_tick_params(width=.25,length=2)
            ax.set_ylabel("y (Meters)")
            ax.set_xlabel("x (Meters)")
        
            plt.savefig(save_loc+"frame%06d.jpg" % count)
                         
            count=count+1 
                    
            plt.close('all') 
            
        self.create_video(save_loc, video_name)
        
        
    def create_motion_snaps(self, entry, name, axis_off=False):
        """
        Will create a series of pictures and paste them together.

        This is NOT for creating a video!!
        """
        from warnings import warn
        warn("Method still requires tuning")
        count=0
        
        # Creating folder if needed
        dirname = os.path.dirname(name)
        os.makedirs(dirname, exist_ok=True)

        fig = plt.figure(dpi=300)
        fig.set_size_inches(self.fx, self.fy)
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        plt.tight_layout()
        if axis_off:
            plt.axis('off')
        # fig.subplots_adjust(top=0.85,
        #                     bottom=0.191,
        #                     left=0.21,
        #                     right=0.919,
        #                     hspace=0.2,
        #                     wspace=0.2)        

        times = self.Time[entry]
        for i in entry: 

            # Plot the bots
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[i,j],self.bot_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.botRadius, fc='black')
                ax.add_patch(patch)
                
            # Plot the interior particles
            for j in range(0,self.ni):
                x0,y0=self.particle_position_x[i,j],self.particle_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.inRadius, fc='tab:green')
                ax.add_patch(patch)    
                
            # Plot the skin particles
            for j in range(0,self.ns):
                x0,y0=self.skin_position_x[i,j],self.skin_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.skinRadius, fc='tab:red')
                ax.add_patch(patch)    

            # Plotting the object
            x, y, angle = self.square_position[i,1], self.square_position[i,2], self.square_position[i,3]
            square_pos = np.array([x,y])
            # Note angle is in RADIANS

            # We have to get the properly rotated object in-order to plot it
            c = cos(angle)
            s = sin(angle)
            R = np.array([[c, -s], [s, c]])
            T = np.array([-self.squareLength/2, -self.squareLength/2]) # Square translation matrix

            # Rotate, translate, and re-rotate the square. We need to get the new coordinates!
            square_pos_prime = np.matmul(np.linalg.inv(R), square_pos)
            square_pos_prime += T
            square_pos_prime = np.matmul(R, square_pos_prime)
            x,y = square_pos_prime
            
            # Plot the square
            patch = plt.Rectangle((x,y), self.squareLength, self.squareLength, angle=degrees(angle), fc='tab:grey')
            ax.add_patch(patch)

        for ii in range(len(self.slot_Points)):
            obj=self.slot_Points[ii]
            ax.plot(obj[:,0],obj[:,1],c='tab:grey',linewidth=.5)
        
        
        ax.plot(self.xtarget, self.ytarget,"*", c='tab:blue',markersize=.1)    
        #plt.title('Time= ' + str(np.round(self.Time[i],1)),fontsize=8,pad=-1)
        x_ticks = np.linspace(self.wxmin, self.wxmax,5,endpoint=True)
        y_ticks = np.linspace(self.wymin, self.wymax,3,endpoint=True)
        ax.set_xticks(np.round(x_ticks,1))
        ax.set_yticks(np.round(y_ticks,1))
        #ax.xaxis.set_tick_params(width=.25,length=2)
        #ax.yaxis.set_tick_params(width=.25,length=2)
        ax.set_ylabel("y (meters)")
        ax.set_xlabel("x (meters)")
        plt.tight_layout()  
        plt.savefig(name+'.png', pad_inches=0)
        plt.savefig(name+'.svg', pad_inches=0)
        plt.savefig(name+'.pdf', pad_inches=0)
        np.savetxt(name + " Time Entries.csv", times, delimiter=',')
  
                         
        #count=count+1 
        #plt.close('all')         
        

    def create_video(self, file, save_name):
        img_array = []
        for filename in glob.glob(file+'/*.jpg'):
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width,height)
            img_array.append(img)
        out = cv2.VideoWriter(save_name+'.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)    
    
        for img in img_array:
            out.write(img)
        out.release()  



class ObstacleFieldDataManager:
    def __init__(self, data_loc,
                botRadius=0.025, skinRadius=0.015, inRadius=0.025,
                wxmin=-1, wxmax=9, wymin=2, wymax=7,
                fig_width = 2.5
                ):
        """
        Class which takes data of a simulation and creates frames.
        Methods are also available to create a video with the data

        data_loc: Data_Location
        botRadius: Bot radius in simulation
        skinRadius: Skin radius in simulation
        inRadius: Radius of interior particles in simulation
        wxmin: Lowest x-value of environment in simulation
        wxmax: Highest x-value of environment in simulation
        wymin: Lowest y-value of environment in simulation
        wymax: Highest y-value of environment in simulation
        """
        self.data_loc=data_loc
        
        path = os.path.dirname(__file__)
        os.chdir(path)

        data_loc = os.path.join(path, self.data_loc)
        if data_loc[-1] != '/': data_loc += '/'

        # Loading bot positions
        bot_positions = np.load(data_loc + 'bot_coords.npy')
        _, nb = bot_positions.shape
        self.nb=int((nb-1)/2)
        self.Time = bot_positions[:,0]
        self.bot_position_x = bot_positions[:,1:2*self.nb+1:2]
        self.bot_position_y = bot_positions[:,2:2*self.nb+1:2]
        
        # Loading interior particle positions
        in_particle_positions=np.load(data_loc + 'interior_coords.npy')
        _, self.ni = in_particle_positions.shape
        self.ni=int((self.ni-1)/2)
        self.particle_position_x=in_particle_positions[:,1:2*self.ni+1:2]
        self.particle_position_y=in_particle_positions[:,2:2*self.ni+1:2]
        
        # Loading skin particle positions
        skin_positions=np.load(data_loc + 'skin_coords.npy')
        _, ns = skin_positions.shape
        self.ns=int((ns-1)/2)
        self.skin_position_x = skin_positions[:,1:2*self.ns+1:2]
        self.skin_position_y=skin_positions[:,2:2*self.ns+1:2]
        
        skin_position=np.load(data_loc + 'skin_coords.npy')
        (n3,self.ns)=np.shape(skin_position)
        self.ns=int((self.ns-1)/2)
        self.skin_position_x=skin_position[:,1:2*self.ns+1:2]
        self.skin_position_y=skin_position[:,2:2*self.ns+1:2]
        
        self.star_coords = np.load(data_loc + 'star_coords.npy')
        self.data5=np.load(data_loc + 'target_loc.npy')
        
        self.xtarget=self.data5[0]
        self.ytarget=self.data5[1]
        
        # Setting system parameters
        self.botRadius=botRadius
        self.skinRadius=skinRadius
        self.inRadius=inRadius
        
        # Setting parameters for plotting
        self.wxmin=wxmin
        self.wxmax=wxmax
        self.wymin=wymin
        self.wymax=wymax
        self.fx=fig_width
        self.fy=self.fx*((self.wymax-self.wymin)/(self.wxmax-self.wxmin))



    def create_animation(self, save_loc = 'frames/', video_name="NO_NAME"):
        """
        Creates frames for making a MatPlotLib animation of the system

        save_loc: Location to save the frames used to create video
        """
        os.makedirs(save_loc, exist_ok=True)
        # fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 6
        plt.rcParams['axes.linewidth'] = .1  
        plt.rcParams["text.usetex"] = True             
        count=0
        for i, Time in enumerate(tqdm(self.Time)): 
            fig = plt.figure(dpi=300)
            fig.set_size_inches(self.fx, self.fy)
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
            plt.tight_layout()
            fig.subplots_adjust(top=0.85,
                                bottom=0.191,
                                left=0.21,
                                right=0.919,
                                hspace=0.2,
                                wspace=0.2)
            
            # Plotting the bots
            for j in range(self.nb):
                x0,y0=self.bot_position_x[i,j],self.bot_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.botRadius, fc='black')
                ax.add_patch(patch)
                
            # Plotting the interior particles
            for j in range(self.ni):
                x0,y0=self.particle_position_x[i,j],self.particle_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.inRadius, fc='tab:green')
                ax.add_patch(patch)    
                
            # Plotting the skin radii
            for j in range(self.ns):
                x0,y0=self.skin_position_x[i,j],self.skin_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.skinRadius, fc='tab:red')
                ax.add_patch(patch)    

            # Plotting the stars        
            for star in self.star_coords:
                ax.fill(star[:,0], star[:,1],color='tab:grey',alpha=1)  
        
            # Plotting the target
            ax.plot(self.xtarget, self.ytarget,"*", c='tab:red',markersize=.1)   
            # Making the plot cleaner
            plt.title('Time= ' + str(np.round(Time,2)),fontsize=8,pad=-1)
            x_ticks = np.linspace(self.wxmin, self.wxmax,5,endpoint=True)
            y_ticks = np.linspace(self.wymin, self.wymax,5,endpoint=True)
            ax.set_xticks(np.round(x_ticks,2))
            ax.set_yticks(np.round(y_ticks,2))
            #ax.xaxis.set_tick_params(width=.25,length=2)
            #ax.yaxis.set_tick_params(width=.25,length=2)
            ax.set_ylabel("$y$ (meters)")
            ax.set_xlabel("$x$ (meters)")
        
            plt.savefig(save_loc+"frame%06d.jpg" % count)
                         
            count=count+1 
                    
            plt.close('all') 
            
        self.create_video(save_loc, video_name)
        
        
    def create_motion_snaps(self, entry, name, axis_off = False):
        """
        Will create a series of pictures and paste them together.

        This is NOT for creating a video!!
        """
        from warnings import warn
        warn("Method still requires tuning")

        # Creating folder if needed
        dirname = os.path.dirname(name)
        os.makedirs(dirname, exist_ok=True)

        fig = plt.figure(dpi=300)
        fig.set_size_inches(self.fx, self.fy)
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        plt.tight_layout()
        if axis_off:
            plt.axis('off')
        #fig.subplots_adjust(top=0.85,
        #                    bottom=0.191,
        #                    left=0.21,
        #                    right=0.919,
        #                    hspace=0.2,
        #                    wspace=0.2)        

        times = self.Time[entry]
        for i in entry:
            for j in range(self.nb):
                x0,y0=self.bot_position_x[i,j],self.bot_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.botRadius, fc='black')
                ax.add_patch(patch)
                
            for j in range(self.ni):
                x0,y0=self.particle_position_x[i,j],self.particle_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.inRadius, fc='tab:green')
                ax.add_patch(patch)    
                
                
            for j in range(self.ns):
                x0,y0=self.skin_position_x[i,j],self.skin_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.skinRadius, fc='tab:red')
                ax.add_patch(patch)    
                    
        # Plotting the stars        
        # This only needs to be done once.
        for star in self.star_coords:
            ax.fill(star[:,0], star[:,1],color='tab:grey',alpha=1,edgecolor='black', linewidth=.5)  
        
        ax.plot(self.xtarget, self.ytarget,"*", c='tab:red',markersize=1)    
        #plt.title('Time= ' + str(np.round(self.Time[i],1)),fontsize=8,pad=-1)
        x_ticks = np.linspace(self.wxmin, self.wxmax,5,endpoint=True)
        y_ticks = np.linspace(self.wymin, self.wymax,5,endpoint=True)
        ax.set_xticks(np.round(x_ticks,2))
        ax.set_yticks(np.round(y_ticks,2))
        ax.set_xlim([self.wxmin,self.wxmax])
        ax.set_ylim([self.wymin,self.wymax])
        ax.xaxis.set_tick_params(width=.25,length=2)
        ax.yaxis.set_tick_params(width=.25,length=2)
        ax.set_ylabel("$y$ (meters)")
        ax.set_xlabel("$x$ (meters)")
        
        plt.tight_layout()
        plt.savefig(name+'.png', pad_inches=0, bbox_inches='tight')
        plt.savefig(name+'.svg', pad_inches=0, bbox_inches='tight')
        plt.savefig(name+'.pdf', pad_inches=0, bbox_inches='tight')
        np.savetxt(name + 'Time Entries.csv', times, delimiter=',')         
        #plt.close('all')         
        

    def create_motion_snaps_SYSTEMONLY(self, entry, name, axis_off = False):
        """
        Will create a series of pictures and paste them together.

        This is NOT for creating a video!!
        """
        assert len(entry)==1,'Entry should only contain one value!!'
        from warnings import warn
        warn("Method still requires tuning")

        # Creating folder if needed
        dirname = os.path.dirname(name)
        os.makedirs(dirname, exist_ok=True)
        
        fig = plt.figure(dpi=300)
        fig.set_size_inches(self.fx, self.fy)
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        plt.tight_layout()
        if axis_off:
            plt.axis('off')

        times = self.Time[entry]
        for i in entry:
        
            # Get points for plotting membrane
            points = []
            ratio = 2
            skin_count = 0
            for k in range(self.nb):
                x0,y0=self.bot_position_x[i,k],self.bot_position_y[i,k]
                points.append([x0,y0])
                for _ in range(ratio):
                    x1,y1 = self.skin_position_x[i,skin_count],self.skin_position_y[i,skin_count]
                    points.append([x1,y1])
                    skin_count+=1

            # Iterate through points and plot them
            last = len(points)-1
            for index, point in enumerate(points):
                if index==last: # Plot the last point to the first one
                    x0, y0 = point
                    x1, y1 = points[0]
                else: # Plot this point to the next one
                    x0, y0 = point
                    x1, y1 = points[index+1]
                
                # Plot the connections as a line
                # ax.plot([x0,x1], [y0,y1], linewidth=1, c='tab:red', zorder=0)

                # Get the points and plot as a spring
                xSpring, ySpring = spring([x0,y0], [x1,y1],8,.03)
                ax.plot(xSpring, ySpring, linewidth=1, c='tab:red', alpha =0.8, zorder=0)
                
            
            # Plot bots
            for j in range(self.nb):
                x0,y0=self.bot_position_x[i,j],self.bot_position_y[i,j]  
                patch = plt.Circle((x0, y0), self.botRadius, fc='black')
                ax.add_patch(patch)

            # Plot interior particles 
            for j in range(self.ni):
                x0,y0=self.particle_position_x[i,j],self.particle_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.inRadius, fc='tab:green')
                ax.add_patch(patch)    
                
            # Plot skin positions
            for j in range(self.ns):
                x0,y0=self.skin_position_x[i,j],self.skin_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.skinRadius, fc='tab:red')
                ax.add_patch(patch)   
        
            
        ax.plot(self.xtarget, self.ytarget,"*", c='tab:red',markersize=1)    
        x_ticks = np.linspace(self.wxmin, self.wxmax,5,endpoint=True)
        y_ticks = np.linspace(self.wymin, self.wymax,5,endpoint=True)
        ax.set_xticks(np.round(x_ticks,2))
        ax.set_yticks(np.round(y_ticks,2))
        ax.set_xlim([self.wxmin,self.wxmax])
        ax.set_ylim([self.wymin,self.wymax])
        ax.xaxis.set_tick_params(width=.25,length=2)
        ax.yaxis.set_tick_params(width=.25,length=2)
        ax.set_ylabel("$y$ (meters)")
        ax.set_xlabel("$x$ (meters)")
        
        plt.tight_layout()
        plt.savefig(name+'.png', pad_inches=0, bbox_inches='tight')
        plt.savefig(name+'.svg', pad_inches=0, bbox_inches='tight')
        plt.savefig(name+'.pdf', pad_inches=0, bbox_inches='tight')
        np.savetxt(name + 'Time Entries.csv', times, delimiter=',')              
        

    def create_video(self, file, save_name):
        img_array = []
        for filename in glob.glob(file+'/*.jpg'):
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width,height)
            img_array.append(img)
        out = cv2.VideoWriter(save_name+'.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)    
    
        for img in img_array:
            out.write(img)
        out.release()  


class TunnelDataManager:
    def __init__(self, data_loc,
                botRadius=0.025, skinRadius=0.015, inRadius=0.025, square_length=0.185,
                wxmin=0, wxmax=3.75, wymin=1, wymax=4.5,
                fig_width = 2.5
                ):
        """
        Class which takes data of a simulation and creates frames.
        Methods are also available to create a video with the data

        data_loc: Data_Location
        botRadius: Bot radius in simulation
        skinRadius: Skin radius in simulation
        inRadius: Radius of interior particles in simulation
        wxmin: Lowest x-value of environment in simulation
        wxmax: Highest x-value of environment in simulation
        wymin: Lowest y-value of environment in simulation
        wymax: Highest y-value of environment in simulation
        """
        self.data_loc=data_loc
        
        # The following verifies that the data location is relative to the location of this script
        path = os.path.dirname(__file__)
        os.chdir(path)
        data_loc = os.path.join(path, self.data_loc)
        if data_loc[-1] != '/': data_loc += '/'

        # Loading bot positions
        bot_positions = np.load(data_loc + 'bot_coords.npy')
        _, nb = bot_positions.shape
        self.nb=int((nb-1)/2)
        self.Time = bot_positions[:,0]
        self.bot_position_x = bot_positions[:,1:2*self.nb+1:2]
        self.bot_position_y = bot_positions[:,2:2*self.nb+1:2]
        
        # Loading interior particle positions
        in_particle_positions=np.load(data_loc + 'interior_coords.npy')
        _, self.ni = in_particle_positions.shape
        self.ni=int((self.ni-1)/2)
        self.particle_position_x=in_particle_positions[:,1:2*self.ni+1:2]
        self.particle_position_y=in_particle_positions[:,2:2*self.ni+1:2]
        
        # Loading skin particle positions
        skin_positions=np.load(data_loc + 'skin_coords.npy')
        _, ns = skin_positions.shape
        self.ns=int((ns-1)/2)
        self.skin_position_x = skin_positions[:,1:2*self.ns+1:2]
        self.skin_position_y=skin_positions[:,2:2*self.ns+1:2]
        
        # Loading object data
        self.tunnel_points=np.load(data_loc + 'tunnel_points.npy')

        # Loading target location data
        self.data5=np.load(data_loc + 'target_loc.npy')
        self.xtarget=self.data5[0]
        self.ytarget=self.data5[1]
        
        # Setting system parameters
        self.botRadius=botRadius
        self.skinRadius=skinRadius
        self.inRadius=inRadius
        self.squareLength = square_length
        
        # Setting parameters for plotting
        self.wxmin=wxmin
        self.wxmax=wxmax
        self.wymin=wymin
        self.wymax=wymax
        self.fx=fig_width
        self.fy=self.fx*((self.wymax-self.wymin)/(self.wxmax-self.wxmin))



    def create_animation(self, save_loc = 'frames/', video_name="NO_NAME"):
        """
        Creates frames for making a MatPlotLib animation of the system

        save_loc: Location to save the frames used to create video
        """
        os.makedirs(save_loc, exist_ok=True)
            
        count=0
        for i, Time in enumerate(tqdm(self.Time)): 
            fig = plt.figure(dpi=300)
            fig.set_size_inches(self.fx, self.fy)
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
            plt.tight_layout()
            fig.subplots_adjust(top=0.85,
                                bottom=0.191,
                                left=0.21,
                                right=0.919,
                                hspace=0.2,
                                wspace=0.2)
            
            # Plotting the bots
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[i,j],self.bot_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.botRadius, fc='black')
                ax.add_patch(patch)
                
            # Plotting the interior particles
            for j in range(0,self.ni):
                x0,y0=self.particle_position_x[i,j],self.particle_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.inRadius, fc='tab:green')
                ax.add_patch(patch)    
                
            # Plotting the skin radii
            for j in range(0,self.ns):
                x0,y0=self.skin_position_x[i,j],self.skin_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.skinRadius, fc='tab:red')
                ax.add_patch(patch)    
                    
            # Plotting the object
            for ii in range(len(self.tunnel_points)):
                obj=self.tunnel_points[ii]
                ax.plot(obj[:,0],obj[:,1],c='tab:gray',width=1)

            # Plotting the target
            ax.plot(self.xtarget, self.ytarget,"*", c='tab:red',markersize=.1)   

            # Making the plot cleaner 
            plt.title('Time= ' + str(np.round(Time,2)),fontsize=8,pad=-1)
            x_ticks = np.linspace(self.wxmin, self.wxmax, 5, endpoint=True)
            y_ticks = np.linspace(self.wymin, self.wymax, 5, endpoint=True)
            ax.set_xticks(np.round(x_ticks,1))
            ax.set_yticks(np.round(y_ticks,1))
            #ax.xaxis.set_tick_params(width=.25,length=2)
            #ax.yaxis.set_tick_params(width=.25,length=2)
            ax.set_ylabel("$y$ (meters)")
            ax.set_xlabel("$x$ (meters)")
        
            plt.savefig(save_loc+"frame%06d.jpg" % count)
                         
            count=count+1 
                    
            plt.close('all') 
            
        self.create_video(save_loc, video_name)
        
        
    def create_motion_snaps(self, entry, name, axis_off = False):
        """
        Will create a series of pictures and paste them together.

        This is NOT for creating a video!!
        """
        from warnings import warn
        warn("Method still requires tuning")
        count=0
        
        # Creating folder if needed
        dirname = os.path.dirname(name)
        os.makedirs(dirname, exist_ok=True)

        fig = plt.figure(dpi=300)
        fig.set_size_inches(self.fx, self.fy)
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        plt.tight_layout()
        if axis_off:
            plt.axis('off')
        #fig.subplots_adjust(top=0.85,
        #                    bottom=0.191,
        #                    left=0.21,
        #                    right=0.919,
        #                   hspace=0.2,
        #                    wspace=0.2)        
        print(self.Time[entry])
        times = self.Time[entry]
        for i in entry: 
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[i,j],self.bot_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.botRadius, fc='black')
                ax.add_patch(patch)
                
            for j in range(0,self.ni):
                x0,y0=self.particle_position_x[i,j],self.particle_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.inRadius, fc='tab:green')
                ax.add_patch(patch)    
                
                
            for j in range(0,self.ns):
                x0,y0=self.skin_position_x[i,j],self.skin_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.skinRadius, fc='tab:red')
                ax.add_patch(patch)    
                
        ax.plot(self.xtarget, self.ytarget,"*", c='tab:blue',markersize=.1)  
        
        for ii in range(len(self.tunnel_points)):
            obj=self.tunnel_points[ii]
            ax.plot(obj[:,0],obj[:,1],c='tab:gray',linewidth=1)
        

        #plt.title('Time= ' + str(np.round(self.Time[i],1)),fontsize=8,pad=-1)
        x_ticks = np.linspace(self.wxmin, self.wxmax,5,endpoint=True)
        y_ticks = np.linspace(self.wymin, self.wymax,5,endpoint=True)
        ax.set_xticks(np.round(x_ticks,1))
        ax.set_yticks(np.round(y_ticks,1))
        #ax.xaxis.set_tick_params(width=.25,length=2)
        #ax.yaxis.set_tick_params(width=.25,length=2)
        ax.set_ylabel(r"$y$ (meters)")
        ax.set_xlabel(r"$x$ (meters)")
        plt.tight_layout() 
        plt.savefig(name+'.png', pad_inches=0)
        plt.savefig(name+'.svg', pad_inches=0)
        plt.savefig(name+'.pdf', pad_inches=0)
        np.savetxt(name+' Time Entries.csv', times, delimiter=',')
                      
        #count=count+1 
        #plt.close('all')         
        

    def create_video(self, file, save_name):
        img_array = []
        for filename in glob.glob(file+'/*.jpg'):
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width,height)
            img_array.append(img)
        out = cv2.VideoWriter(save_name+'.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)    
    
        for img in img_array:
            out.write(img)
        out.release()  



class CooridorDataManager:
    def __init__(self, data_loc,
                botRadius=0.025, skinRadius=0.015, inRadius=0.025, square_length=0.185,
                wxmin=0, wxmax=4.5, wymin=3.0, wymax=6,
                fig_width = 2.5
                ):
        """
        Class which takes data of a simulation and creates frames.
        Methods are also available to create a video with the data

        data_loc: Data_Location
        botRadius: Bot radius in simulation
        skinRadius: Skin radius in simulation
        inRadius: Radius of interior particles in simulation
        wxmin: Lowest x-value of environment in simulation
        wxmax: Highest x-value of environment in simulation
        wymin: Lowest y-value of environment in simulation
        wymax: Highest y-value of environment in simulation
        """
        self.data_loc=data_loc
        
        path = os.path.dirname(__file__)
        os.chdir(path)

        data_loc = os.path.join(path, self.data_loc)
        if data_loc[-1] != '/': data_loc += '/'

        # Loading bot positions
        bot_positions = np.load(data_loc + 'bot_coords.npy')
        _, nb = bot_positions.shape
        self.nb=int((nb-1)/2)
        self.Time = bot_positions[:,0]
        self.bot_position_x = bot_positions[:,1:2*self.nb+1:2]
        self.bot_position_y = bot_positions[:,2:2*self.nb+1:2]
        
        # Loading interior particle positions
        in_particle_positions=np.load(data_loc + 'interior_coords.npy')
        _, self.ni = in_particle_positions.shape
        self.ni=int((self.ni-1)/2)
        self.particle_position_x=in_particle_positions[:,1:2*self.ni+1:2]
        self.particle_position_y=in_particle_positions[:,2:2*self.ni+1:2]
        
        # Loading skin particle positions
        skin_positions=np.load(data_loc + 'skin_coords.npy')
        _, ns = skin_positions.shape
        self.ns=int((ns-1)/2)
        self.skin_position_x = skin_positions[:,1:2*self.ns+1:2]
        self.skin_position_y=skin_positions[:,2:2*self.ns+1:2]
        
        # Loading object data
        self.tunnel_points=np.load(data_loc + 'tunnel_coords.npy')

        # Loading target location data
        self.data5=np.load(data_loc + 'target_loc.npy')
        self.xtarget=self.data5[0]
        self.ytarget=self.data5[1]
        
        # Setting system parameters
        self.botRadius=botRadius
        self.skinRadius=skinRadius
        self.inRadius=inRadius
        self.squareLength = square_length
        
        # Setting parameters for plotting
        self.wxmin=wxmin
        self.wxmax=wxmax
        self.wymin=wymin
        self.wymax=wymax
        self.fx=fig_width
        self.fy=self.fx*((self.wymax-self.wymin)/(self.wxmax-self.wxmin))



    def create_animation(self, save_loc = 'frames/', video_name="NO_NAME"):
        """
        Creates frames for making a MatPlotLib animation of the system

        save_loc: Location to save the frames used to create video
        """
        os.makedirs(save_loc, exist_ok=True)
            
        count=0
        for i, Time in enumerate(tqdm(self.Time)): 
            fig = plt.figure(dpi=300)
            fig.set_size_inches(self.fx, self.fy)
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
            plt.tight_layout()
            fig.subplots_adjust(top=0.85,
                                bottom=0.191,
                                left=0.21,
                                right=0.919,
                                hspace=0.2,
                                wspace=0.2)
            
            # Plotting the bots
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[i,j],self.bot_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.botRadius, fc='black')
                ax.add_patch(patch)
                
            # Plotting the interior particles
            for j in range(0,self.ni):
                x0,y0=self.particle_position_x[i,j],self.particle_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.inRadius, fc='tab:green')
                ax.add_patch(patch)    
                
            # Plotting the skin radii
            for j in range(0,self.ns):
                x0,y0=self.skin_position_x[i,j],self.skin_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.skinRadius, fc='tab:red')
                ax.add_patch(patch)    
                    
            # Plotting the object
            for ii in range(len(self.tunnel_points)):
                obj=self.tunnel_points[ii]
                ax.plot(obj[:,0],obj[:,1],c='tab:gray',width=1)
            #ax.add_patch(patch)

            # Plotting the target
            ax.plot(self.xtarget, self.ytarget,"*", c='tab:red',markersize=.1)   

            # Making the plot cleaner 
            plt.title('Time= ' + str(np.round(Time,2)),fontsize=8,pad=-1)
            x_ticks = np.linspace(self.wxmin, self.wxmax, 5, endpoint=True)
            y_ticks = np.linspace(self.wymin, self.wymax, 5, endpoint=True)
            ax.set_xticks(np.round(x_ticks,1))
            ax.set_yticks(np.round(y_ticks,1))
            #ax.xaxis.set_tick_params(width=.25,length=2)
            #ax.yaxis.set_tick_params(width=.25,length=2)
            ax.set_ylabel("$y$ (meters)")
            ax.set_xlabel("$x$ (meters)")
        
            plt.savefig(save_loc+"frame%06d.jpg" % count)
                         
            count=count+1 
                    
            plt.close('all') 
            
        self.create_video(save_loc, video_name)
        
        
    def create_motion_snaps(self, entry, name, axis_off = False):
        """
        Will create a series of pictures and paste them together.

        This is NOT for creating a video!!
        """
        from warnings import warn
        warn("Method still requires tuning")
        count=0
        
        # Creating folder if needed
        dirname = os.path.dirname(name)
        os.makedirs(dirname, exist_ok=True)

        fig = plt.figure(dpi=300)
        fig.set_size_inches(self.fx, self.fy)
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        plt.tight_layout()
        if axis_off:
            plt.axis('off')
        #fig.subplots_adjust(top=0.85,
        #                    bottom=0.191,
        #                    left=0.21,
        #                    right=0.919,
        #                   hspace=0.2,
        #                    wspace=0.2)        
        print(self.Time[entry])
        times = self.Time[entry]
        for i in entry: 
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[i,j],self.bot_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.botRadius, fc='black')
                ax.add_patch(patch)
                
            for j in range(0,self.ni):
                x0,y0=self.particle_position_x[i,j],self.particle_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.inRadius, fc='tab:green')
                ax.add_patch(patch)    
                
                
            for j in range(0,self.ns):
                x0,y0=self.skin_position_x[i,j],self.skin_position_y[i,j]  
                patch = plt.Circle((x0, y0),self.skinRadius, fc='tab:red')
                ax.add_patch(patch)    
                    
            # TODO: Write script which plots the square object

        ax.plot(self.xtarget, self.ytarget,"*", c='tab:blue',markersize=.1)  
        
        for ii in range(len(self.tunnel_points)):
            obj=self.tunnel_points[ii]
            ax.plot(obj[:,0],obj[:,1],c='tab:gray',linewidth=1)
        
        
        #plt.title('Time= ' + str(np.round(self.Time[i],1)),fontsize=8,pad=-1)
        x_ticks = np.linspace(self.wxmin, self.wxmax,5,endpoint=True)
        y_ticks = np.linspace(self.wymin, self.wymax,5,endpoint=True)
        ax.set_xticks(np.round(x_ticks,1))
        ax.set_yticks(np.round(y_ticks,1))
        ax.xaxis.set_tick_params(width=.25,length=2)
        ax.yaxis.set_tick_params(width=.25,length=2)
        ax.set_ylabel(r"$y$ (meters)")
        ax.set_xlabel(r"$x$ (meters)")
        plt.tight_layout() 
        plt.savefig(name+'.png', pad_inches=0)
        plt.savefig(name+'.svg', pad_inches=0)
        plt.savefig(name+'.pdf', pad_inches=0)
        np.savetxt(name + ' Time Entries.csv', times, delimiter=',')
                      
        #count=count+1 
        #plt.close('all')         
        

    def create_video(self, file, save_name):
        img_array = []
        for filename in glob.glob(file+'/*.jpg'):
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width,height)
            img_array.append(img)
        out = cv2.VideoWriter(save_name+'.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)    
    
        for img in img_array:
            out.write(img)
        out.release()  


import math
def spring(start, end, nodes, width):
    """!
    Return a list of points corresponding to a spring.
    @param r1 (array-like) The (x, y) coordinates of the first endpoint.
    @param r2 (array-like) The (x, y) coordinates of the second endpoint.
    @param nodes (int) The number of spring "nodes" or coils.
    @param width (int or float) The diameter of the spring.
    @return An array of x coordinates and an array of y coordinates.
    """

    # Check that nodes is at least 1.
    nodes = max(int(nodes), 1)

    # Convert to numpy array to account for inputs of different types/shapes.
    start, end = np.array(start).reshape((2,)), np.array(end).reshape((2,))

    # If both points are coincident, return the x and y coords of one of them.
    if (start == end).all():
        return start[0], start[1]

    # Calculate length of spring (distance between endpoints).
    length = np.linalg.norm(np.subtract(end, start))

    # Calculate unit vectors tangent (u_t) and normal (u_t) to spring.
    u_t = np.subtract(end, start) / length
    u_n = np.array([[0, -1], [1, 0]]).dot(u_t)

    # Initialize array of x (row 0) and y (row 1) coords of the nodes+2 points.
    spring_coords = np.zeros((2, nodes + 2))
    spring_coords[:,0], spring_coords[:,-1] = start, end

    # Check that length is not greater than the total length the spring
    # can extend (otherwise, math domain error will result), and compute the
    # normal distance from the centerline of the spring.
    normal_dist = math.sqrt(max(0, width**2 - (length**2 / nodes**2))) / 2

    # Compute the coordinates of each point (each node).
    for i in range(1, nodes + 1):
        spring_coords[:,i] = (
            start
            + ((length * (2 * i - 1) * u_t) / (2 * nodes))
            + (normal_dist * (-1)**i * u_n))

    return spring_coords[0,:], spring_coords[1,:]