# -*- coding: utf-8 -*-
"""
Created on Fri Jan 28 08:55:25 2022

@author: dmulr
"""


from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np
import os
import glob
import cv2
from tqdm import tqdm
import matplotlib.font_manager as fm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection




class import_data:
    def __init__(self,name,path,xmin,xmax,ymin,ymax,zmin,zmax):

        self.name=name
        self.path=path
        self.xmin=xmin
        self.xmax=xmax
        self.ymin=ymin
        self.ymax=ymax
        self.zmin=zmin
        self.zmax=zmax
        self.xticks=np.linspace(self.xmin,self.xmax,3)
        self.yticks=np.linspace(self.ymin,self.ymax,3)
        self.zticks=np.linspace(self.zmin,self.zmax,3)
        self.count=0
        self.parameters = np.load(self.path+'\\'+self.name+'\\Parameters.npy',allow_pickle=True)
        self.parameters = self.parameters.tolist()
        self.particle_width = self.parameters['particle_width']
        self.control_sphere_width = 2*self.parameters['control_sphere_radius']
        self.data=np.load(self.path+self.name+'/'+'data.npz',allow_pickle=True)
        self.Particle_positions_savex=self.data['Particle_positions_savex']
        self.Particle_positions_savey=self.data['Particle_positions_savey']
        self.Particle_positions_savez=self.data['Particle_positions_savez']
        self.bot_positions_savex=self.data['bot_positions_savex']
        self.bot_positions_savey=self.data['bot_positions_savey']
        self.bot_positions_savez=self.data['bot_positions_savez']
        self.Force_savex=self.data['Force_savex']
        self.Force_savey=self.data['Force_savey']
        self.Force_savez=self.data['Force_savez']
        self.Time=self.data['Time']
        self.DATA=self.data['DATA']
        self.faces=self.data['faces']
        self.F_value=self.data['F_value']
        (self.nb,self.timesteps)=np.shape(self.bot_positions_savex)
        (self.ni,timesteps)=np.shape(self.Particle_positions_savex)
        u = np.linspace(0, np.pi, 4)
        v = np.linspace(0, 2 * np.pi, 4)
        self.X = self.particle_width*np.outer(np.sin(u), np.cos(v))
        self.Y = self.particle_width*np.outer(np.sin(u), np.sin(v))
        self.Z = self.particle_width*np.outer(np.cos(u), np.ones_like(v))
        
        self.direct1=0
        self.direct2=0
        self.direct3=0
        self.direct4=0


    def create_snapshot__(self,entry):
        i=entry
        import matplotlib.font_manager as fm
        # Rebuild the matplotlib font cache
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        plt.rcParams["text.usetex"] = True 

        
        fig, ax = plt.subplots(nrows=1, ncols=1,subplot_kw={'projection':'3d'},figsize=(3,3),dpi=300)         # Set the figure size

        ss=.05

        #### VIEW 1
        ax.view_init(elev=0, azim=0)
        #ax[0,0].set_aspect('auto')
        ax._axis3don = False
        ax.set_box_aspect((1, 1, 1)) 
        val = [.5,0,0]
        labels = ['$x$','$y$', '$z$']

        v=1
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax.plot(x,y,z,'k-', linewidth=.25)

        v=2
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax.plot(x,y,z,'k-', linewidth=.25)
        
        ax.text(0,.5,0, labels[1], color='k', fontsize=8)
        ax.text(0,0,.5, labels[2], color='k', fontsize=8)
        
            
        ax.scatter3D(self.bot_positions_savex[:,i],self.bot_positions_savey[:,i],self.bot_positions_savez[:,i], c='k',s=ss)

        
        datas=self.DATA[i]
        vertices = np.asarray(datas[1])
        ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = self.faces,edgecolor=[[0,0,0]], linewidth=.1, alpha=0.25,color='tab:red', shade=False)
        #plt.gca().set_aspect('equal', adjustable='box')
        plt.title('Time = ' + str(np.round(self.Time[i],1))+" s",fontsize=8)
        #plt.savefig(self.name+"/View.png")
        #plt.savefig(self.name+"/View.pdf") 
        #plt.savefig(self.name+"/View.svg")
        plt.savefig(self.name+'/'+'time'+str(np.round(self.Time[i],2))+'.jpg')
        plt.savefig(self.name+'/'+'time'+str(np.round(self.Time[i],2))+'.svg')    
        plt.savefig(self.name+'/'+'time'+str(np.round(self.Time[i],2))+'.pdf')

               
        #count=count+1       
        #plt.close('all') 
    
    






















    def create_snapshot(self):
        i=-1
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 6
        plt.rcParams['axes.linewidth'] = .1  
        plt.rcParams["text.usetex"] = True 
        #plt.rcParams["figure.autolayout"] = True
        #ax.tick_params(axis='both', width=10, labelsize=10, pad=0)
              
        
        fig, ax = plt.subplots(nrows=2, ncols=2,subplot_kw={'projection':'3d'},figsize= (3, 3),dpi=300)         # Set the figure size
        # sl=0.25
        # # side 1
        # x = [sl, -sl, -sl, sl]
        # y = [sl, sl, -sl, -sl]
        # z = [-sl, -sl, -sl, -sl]
        # vertices1 = [list(zip(x,y,z))]
        # poly10 = Poly3DCollection(vertices1, alpha=0.2)
        # poly11 = Poly3DCollection(vertices1, alpha=0.2)
        # poly12 = Poly3DCollection(vertices1, alpha=0.2)
        # poly13 = Poly3DCollection(vertices1, alpha=0.2)
        # # side 2
        # x = [sl, sl, sl, sl]
        # y = [sl, -sl, -sl, sl]
        # z = [sl, sl, -sl, -sl]
        # vertices2 = [list(zip(x,y,z))]
        # poly20 = Poly3DCollection(vertices2, alpha=0.2)
        # poly21 = Poly3DCollection(vertices2, alpha=0.2)
        # poly22 = Poly3DCollection(vertices2, alpha=0.2)
        # poly23 = Poly3DCollection(vertices2, alpha=0.2)
        # # side3
        # x = [-sl, -sl, -sl, -sl]
        # y = [sl, -sl, -sl, sl]
        # z = [sl, sl, -sl, -sl]
        # vertices3 = [list(zip(x,y,z))]
        # poly30 = Poly3DCollection(vertices3, alpha=0.2)
        # poly31 = Poly3DCollection(vertices3, alpha=0.2)
        # poly32 = Poly3DCollection(vertices3, alpha=0.2)
        # poly33 = Poly3DCollection(vertices3, alpha=0.2)        
        # # side4
        # x = [sl, -sl, -sl, sl]
        # y = [sl, sl, -sl, -sl]
        # z = [sl, sl, sl, sl]
        # vertices4 = [list(zip(x,y,z))]
        # poly40 = Poly3DCollection(vertices4, alpha=0.2)
        # poly41 = Poly3DCollection(vertices4, alpha=0.2)
        # poly42 = Poly3DCollection(vertices4, alpha=0.2)
        # poly43 = Poly3DCollection(vertices4, alpha=0.2)        
        # # side 5
        # x = [sl, -sl, -sl, sl]
        # y = [sl,sl, sl, sl]
        # z = [sl, sl, -sl, -sl]
        # vertices5 = [list(zip(x,y,z))]
        # poly50 = Poly3DCollection(vertices5, alpha=0.2)
        # poly51 = Poly3DCollection(vertices4, alpha=0.2)
        # poly52 = Poly3DCollection(vertices4, alpha=0.2)
        # poly53 = Poly3DCollection(vertices4, alpha=0.2)        
        
        # # side 6
        # x = [sl, -sl, -sl, sl]
        # y = [-sl, -sl, -sl, -sl]
        # z = [sl, sl, -sl, -sl]
        # vertices6 = [list(zip(x,y,z))]
        # poly60 = Poly3DCollection(vertices6, alpha=0.2)
        # poly61 = Poly3DCollection(vertices6, alpha=0.2)
        # poly62 = Poly3DCollection(vertices6, alpha=0.2)
        # poly63 = Poly3DCollection(vertices6, alpha=0.2)
                
        
        ss=.05
        sss=1
        #### VIEW 1
        ax[0,0].view_init(elev=0, azim=0)
        #ax[0,0].set_aspect('auto')
        ax[0,0]._axis3don = False
        ax[0,0].set_box_aspect((1, 1, 1)) 
        val = [.5,0,0]
        labels = ['$x$','$y$', '$z$']

        v=1
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax[0,0].plot(x,y,z,'k-', linewidth=.25)

        v=2
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax[0,0].plot(x,y,z,'k-', linewidth=.25)
        
        ax[0,0].text(0,.5,0, labels[1], color='k', fontsize=6)
        ax[0,0].text(0,0,.5, labels[2], color='k', fontsize=6)
        
            
        ax[0,0].scatter3D(self.bot_positions_savex[:,i],self.bot_positions_savey[:,i],self.bot_positions_savez[:,i], c='k',s=ss)
        # for j in range(self.ni):
        #      x = self.X + self.Particle_positions_savex[j,i] 
        #      y = self.Y + self.Particle_positions_savey[j,i] 
        #      z = self.Z + self.Particle_positions_savez[j,i] 
        #      ax[0,0].plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
        # ax[0,0].add_collection3d(poly10)
        # ax[0,0].add_collection3d(poly20)
        # ax[0,0].add_collection3d(poly30)
        # ax[0,0].add_collection3d(poly40)
        # ax[0,0].add_collection3d(poly50)
        # ax[0,0].add_collection3d(poly60)
        
        
        datas=self.DATA[i]
        vertices = np.asarray(datas[1])
        ax[0,0].plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = self.faces,edgecolor=[[0,0,0]], linewidth=.1, alpha=0.25,color='tab:red', shade=False)
        
        
        

        
        #### VIEW 2
        #ax[1,0].set_aspect('auto')  
        ax[1,0]._axis3don = False 
        ax[1,0].set_box_aspect((1, 1, 1))         
        val = [.5,0,0]
        labels = ['$x$','$y$', '$z$']

        v=0
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax[1,0].plot(x,y,z,'k-', linewidth=.25)

        v=2
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax[1,0].plot(x,y,z,'k-', linewidth=.25)
        
        ax[1,0].text(.5,0,0, labels[0], color='k', fontsize=6)
        ax[1,0].text(0,0,.5, labels[2], color='k', fontsize=6)
        
        
        ax[1,0].view_init(elev=0, azim=90)
        ax[1,0].scatter3D(self.bot_positions_savex[:,i],self.bot_positions_savey[:,i],self.bot_positions_savez[:,i], c='k',s=ss)
        # for j in range(self.ni):
        #     x = self.X + self.Particle_positions_savex[j,i] 
        #     y = self.Y + self.Particle_positions_savey[j,i] 
        #     z = self.Z + self.Particle_positions_savez[j,i] 
        #     ax[1,0].plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
        # ax[1,0].add_collection3d(poly11)
        # ax[1,0].add_collection3d(poly21)
        # ax[1,0].add_collection3d(poly31)
        # ax[1,0].add_collection3d(poly41)
        # ax[1,0].add_collection3d(poly51)
        # ax[1,0].add_collection3d(poly61)        
        
        
        datas=self.DATA[i]
        vertices = np.asarray(datas[1])
        ax[1,0].plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = self.faces,edgecolor=[[0,0,0]], linewidth=.1, alpha=0.25,color='tab:red', shade=False)
                  
    
        #### VIEW 3 
        ax[1,1]._axis3don = False
        #ax[1,1].set_aspect('auto')  
        ax[1,1].set_box_aspect((1, 1, 1)) 
        val = [.5,0,0]
        labels = ['$x$','$y$', '$z$']

        v=0
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax[1,1].plot(x,y,z,'k-', linewidth=.25)

        v=1
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax[1,1].plot(x,y,z,'k-', linewidth=.25)
        
        ax[1,1].text(.5,0,0, labels[0], color='k', fontsize=6)
        ax[1,1].text(0,.5,0, labels[1], color='k', fontsize=6)
        
          
        
        ax[1,1].view_init(elev=90, azim=0)
        ax[1,1].scatter3D(self.bot_positions_savex[:,i],self.bot_positions_savey[:,i],self.bot_positions_savez[:,i], c='k',s=ss)
        # for j in range(self.ni):
        #      x = self.X + self.Particle_positions_savex[j,i] 
        #      y = self.Y + self.Particle_positions_savey[j,i] 
        #      z = self.Z + self.Particle_positions_savez[j,i] 
        #      ax[1,1].plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
        # ax[1,1].add_collection3d(poly12)
        # ax[1,1].add_collection3d(poly22)
        # ax[1,1].add_collection3d(poly32)
        # ax[1,1].add_collection3d(poly42)
        # ax[1,1].add_collection3d(poly52)
        # ax[1,1].add_collection3d(poly62)  

        
        datas=self.DATA[i]
        vertices = np.asarray(datas[1])
        ax[1,1].plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = self.faces,edgecolor=[[0,0,0]], linewidth=.1, alpha=0.25,color='tab:red', shade=False)
        
  
        
         #### VIEW 4
        ax[0,1]._axis3don = False
        #ax[0,1].set_aspect('auto')
        ax[0,1].set_box_aspect((1, 1, 1)) 
        labels = ['$x$','$y$', '$z$']
        v=0
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax[0,1].plot(x,y,z,'k-', linewidth=.25)

        v=1
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax[0,1].plot(x,y,z,'k-', linewidth=.25)
         
        v=2
        x = [val[v-0], -val[v-0]]
        y = [val[v-1], -val[v-1]]
        z = [val[v-2], -val[v-2]]
        ax[0,1].plot(x,y,z,'k-', linewidth=.25)
         
                  
        
        ax[0,1].text(.5,0,0, labels[0], color='k', fontsize=6)
        ax[0,1].text(0,.5,0, labels[1], color='k', fontsize=6)   
        ax[0,1].text(0,0,.5, labels[2], color='k', fontsize=6)   
    
        ax[0,1].scatter3D(self.bot_positions_savex[:,i],self.bot_positions_savey[:,i],self.bot_positions_savez[:,i], c='k',s=ss,marker='o')
        # for j in range(self.ni):
        #     x = self.X + self.Particle_positions_savex[j,i] 
        #     y = self.Y + self.Particle_positions_savey[j,i] 
        #     z = self.Z + self.Particle_positions_savez[j,i] 
        #     ax[0,1].plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
        # ax[0,1].add_collection3d(poly13)
        # ax[0,1].add_collection3d(poly23)
        # ax[0,1].add_collection3d(poly33)
        # ax[0,1].add_collection3d(poly43)
        # ax[0,1].add_collection3d(poly53)
        # ax[0,1].add_collection3d(poly63)          
        datas=self.DATA[i]
        vertices = np.asarray(datas[1])
        ax[0,1].plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = self.faces,edgecolor=[[0,0,0]], linewidth=.1, alpha=.25,color='tab:red', shade=False)
        
   
        
         # ax.set_title("Time="+str(np.round(self.Time[i],1)))
         #ax[0,1].set_xticks(self.xticks)
         #ax[0,1].set_yticks(self.yticks)
         #ax[0,1].set_zticks(self.zticks)
         # ax.set_xlabel("$x$")
         # ax.set_ylabel("$y$")
         # ax.set_zlabel("$z$")    
         #ax[0,1].set_xlim3d(self.xmin,self.xmax)
         #ax[0,1].set_ylim3d(self.ymin,self.ymax)
         #ax[0,1].set_zlim3d(self.zmin,self.zmax)             
            
        
         #plt.savefig(self.name+"/View.png",bbox_inches='tight')
         #plt.savefig(self.name+"/View.pdf",bbox_inches='tight') 
         #plt.savefig(self.name+"/View.svg",bbox_inches='tight')      
        plt.savefig(self.name+"/View.png")
        plt.savefig(self.name+"/View.pdf") 
        plt.savefig(self.name+"/View.svg")               
        #count=count+1       
        #plt.close('all') 
    
    

    def create_animation(self):
        
        
        self.direct1 = os.path.join(self.name+'/_frames1')    
        if not os.path.isdir(self.direct1):
             os.makedirs(self.direct1) 
             
        self.direct2 = os.path.join(self.name+'/_frames2')    
        if not os.path.isdir(self.direct2):
             os.makedirs(self.direct2) 
             
        self.direct3 = os.path.join(self.name+'/_frames3')    
        if not os.path.isdir(self.direct3):
             os.makedirs(self.direct3)   
             
        self.direct4 = os.path.join(self.name+'/_frames4')    
        if not os.path.isdir(self.direct4):
             os.makedirs(self.direct4)              
             
             
        count=0 
        T=0
        for i in tqdm(range(self.timesteps)):
            T+=1
        
            if T%self.timesteps==0:
                pass
                print(T)
            if T>self.timesteps:
                break
            pass   
            #if animate==True:
              
                
              
            # #### VIEW 1
            # fm._rebuild()
            # plt.rcParams['font.family'] = 'Times New Roman'
            # plt.rcParams['mathtext.fontset'] = 'dejavuserif'
            # plt.rcParams['font.size'] = 10
            # plt.rcParams['axes.linewidth'] = .1  
            
            # fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
            # ax = fig.gca(projection='3d')
            # ax.view_init(elev=0, azim=0)
            # ax.scatter3D(self.bot_positions_savex[:,i],self.bot_positions_savey[:,i],self.bot_positions_savez[:,i], c='k')
            # # for j in range(self.ni):
            # #     x = self.X + self.Particle_positions_savex[j,i] 
            # #     y = self.Y + self.Particle_positions_savey[j,i] 
            # #     z = self.Z + self.Particle_positions_savez[j,i] 
            # #     ax.plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
            
            # datas=self.DATA[i]
            # vertices = np.asarray(datas[1])
            # ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = self.faces,edgecolor=[[0,0,0]], linewidth=.25, alpha=0.25,color='tab:red', shade=False)
            # ax.set_title("Time="+str(np.round(self.Time[i],1)))
            # ax.xaxis.set_ticklabels([])
            # ax.set_yticks(self.yticks)
            # ax.set_zticks(self.zticks)
            # #ax.set_xlabel("$x$")
            # ax.set_ylabel("$y$")
            # ax.set_zlabel("$z$")    
            # ax.set_xlim3d(self.xmin,self.xmax)
            # ax.set_ylim3d(self.ymin,self.ymax)
            # ax.set_zlim3d(self.zmin,self.zmax)   
            
            # for line in ax.xaxis.get_ticklines():
            #     line.set_visible(False)
            # plt.savefig(self.direct1+"/frame%04d.jpg" % count)              
            # #count=count+1       
            # plt.close('all') 
        
        
            # #### VIEW 2
            # fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
            # ax = fig.gca(projection='3d')
            # ax.view_init(elev=0, azim=90)
            # ax.scatter3D(self.bot_positions_savex[:,i],self.bot_positions_savey[:,i],self.bot_positions_savez[:,i], c='k')
            # # for j in range(self.ni):
            # #     x = self.X + self.Particle_positions_savex[j,i] 
            # #     y = self.Y + self.Particle_positions_savey[j,i] 
            # #     z = self.Z + self.Particle_positions_savez[j,i] 
            # #     ax.plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
            
            # datas=self.DATA[i]
            # vertices = np.asarray(datas[1])
            # ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = self.faces,edgecolor=[[0,0,0]], linewidth=.25, alpha=0.25,color='tab:red', shade=False)
            # ax.set_title("Time="+str(np.round(self.Time[i],2)))
            # ax.set_xticks(self.xticks)
            # #ax.set_yticks(self.yticks)
            # ax.yaxis.set_ticklabels([])
            # ax.set_zticks(self.zticks)
            # ax.set_xlabel("$x$")
            # #ax.set_ylabel("$y$")
            # ax.set_zlabel("$z$")    
            # ax.set_xlim3d(self.xmin,self.xmax)
            # ax.set_ylim3d(self.ymin,self.ymax)
            # ax.set_zlim3d(self.zmin,self.zmax)    
            # for line in ax.yaxis.get_ticklines():
            #     line.set_visible(False)
                
            # plt.savefig(self.direct2+"/frame%04d.jpg" % count)              
            # #count=count+1       
            # plt.close('all')
            
            
            #### VIEW 3
            fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
            ax = fig.gca(projection='3d')
            ax.view_init(elev=90, azim=0)
            ax.scatter3D(self.bot_positions_savex[:,i],self.bot_positions_savey[:,i],self.bot_positions_savez[:,i], c='k')
            # for j in range(self.ni):
            #     x = self.X + self.Particle_positions_savex[j,i] 
            #     y = self.Y + self.Particle_positions_savey[j,i] 
            #     z = self.Z + self.Particle_positions_savez[j,i] 
            #     ax.plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
            
            datas=self.DATA[i]
            vertices = np.asarray(datas[1])
            ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = self.faces,edgecolor=[[0,0,0]], linewidth=.25, alpha=0.25,color='tab:red', shade=False)
            ax.set_title("Time="+str(np.round(self.Time[i],1)))
            
            ax.set_axis_off() 
            #ax.set_xticks(self.xticks)
            #ax.set_yticks(self.yticks)
            #ax.zaxis.set_ticklabels([])
            #ax.set_zticks(self.zticks)
            ax.set_xlabel("$x$")
            ax.set_ylabel("$y$")
            #ax.set_zlabel("$z$")    
            ax.set_xlim3d(self.xmin,self.xmax)
            ax.set_ylim3d(self.ymin,self.ymax)
            for line in ax.zaxis.get_ticklines():
                line.set_visible(False)
            #ax.set_zlim3d(self.zmin,self.zmax)    
            plt.savefig(self.direct3+"/frame%04d.jpg" % count)              
            count=count+1       
            plt.close('all')


    
            # #### VIEW 4
            # fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
            # ax = fig.gca(projection='3d')
            # #ax.view_init(elev=0, azim=0)
            # ax.scatter3D(self.bot_positions_savex[:,i],self.bot_positions_savey[:,i],self.bot_positions_savez[:,i], c='k')
            # # for j in range(self.ni):
            # #     x = self.X + self.Particle_positions_savex[j,i] 
            # #     y = self.Y + self.Particle_positions_savey[j,i] 
            # #     z = self.Z + self.Particle_positions_savez[j,i] 
            # #     ax.plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
            
            # datas=self.DATA[i]
            # vertices = np.asarray(datas[1])
            # ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = self.faces,edgecolor=[[0,0,0]], linewidth=.25, alpha=0.25,color='tab:red', shade=False)
            # ax.set_title("Time="+str(np.round(self.Time[i],1)))
            # ax.set_xticks(self.xticks)
            # ax.set_yticks(self.yticks)
            # ax.set_zticks(self.zticks)
            # ax.set_xlabel("$x$")
            # ax.set_ylabel("$y$")
            # ax.set_zlabel("$z$")    
            # ax.set_xlim3d(self.xmin,self.xmax)
            # ax.set_ylim3d(self.ymin,self.ymax)
            # ax.set_zlim3d(self.zmin,self.zmax)    
            # plt.savefig(self.direct4+"/frame%04d.jpg" % count)              
            #count=count+1       
            # plt.close('all')            
    
        #self.create_video(self.name,'_frames1',self.name+'1')    
        #self.create_video(self.name,'_frames2',self.name+'2') 
        self.create_video(self.name,'_frames3',self.name+'3')
        #self.create_video(self.name,'_frames4',self.name+'4')
     
     
     
     
            
    def create_video(self,name,file,save_name):
    #import pdb    
        img_array = []
        for index, filename in enumerate(glob.glob(name+'/'+file+'/'+'/*.jpg')):
    #pdb.set_trace()
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width,height)
            img_array.append(img)
        out = cv2.VideoWriter(name+'/'+save_name+'video.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, (width,height))    
    
        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()                

    def field_value(self):
        
        Fv=[]
        
        for i in range(self.timesteps):
            Fv.append(np.sum(abs(self.F_value[:,i])))
            
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 6
        plt.rcParams['axes.linewidth'] = .1    
            
            
        fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3),dpi=300)
        axs.plot(self.Time,Fv,linewidth=1,color='k')
        x_ticks = np.linspace(self.Time[0], self.Time[-1],5,endpoint=True)
        y_ticks = np.linspace(np.min(Fv), np.max(Fv),5,endpoint=True)
        axs.set_xticks(np.round(x_ticks,2))
        axs.set_yticks(np.round(y_ticks,2))
        axs.xaxis.set_tick_params(width=.25,length=2)
        axs.yaxis.set_tick_params(width=.25,length=2)
        axs.set_ylabel(r"$\Sigma \phi$")
        axs.set_xlabel("Time (seconds)")
        axs.set_title("Field Values")
        axs.grid(True)
        plt.tight_layout()
        plt.savefig(self.name+"/Field_Values_"+self.name+'.png')
        plt.savefig(self.name+"/Field_Values_"+self.name+'.pdf')
        plt.savefig(self.name+"/Field_Values_"+self.name+'.svg')
        plt.close('all') 



            
# xticks=np.linspace(xmin,xmax,5)
# yticks=np.linspace(ymin,ymax,5)
# zticks=np.linspace(zmin,zmax,5)


# count=0

# parameters = np.load(path+'\\'+name+'\\Parameters.npy',allow_pickle=True)
# parameters = parameters.tolist()

# particle_width = parameters['particle_width']
# control_sphere_width = 2*parameters['control_sphere_radius']

# data=np.load(path+name+'/'+'data.npz',allow_pickle=True)
# Particle_positions_savex=data['Particle_positions_savex']
# Particle_positions_savey=data['Particle_positions_savey']
# Particle_positions_savez=data['Particle_positions_savez']
# bot_positions_savex=data['bot_positions_savex']
# bot_positions_savey=data['bot_positions_savey']
# bot_positions_savez=data['bot_positions_savez']
# Force_savex=data['Force_savex']
# Force_savey=data['Force_savey']
# Force_savez=data['Force_savez']
# (nb,n)=np.shape(Force_savez)

# Time=data['Time']
# DATA=data['DATA']
# faces=data['faces']
# F_value=data['F_value']

# (nb,timesteps)=np.shape(bot_positions_savex)
# (ni,timesteps)=np.shape(Particle_positions_savex)
# #print(ni)




# direct1 = os.path.join(name+'/_frames1')    
# if not os.path.isdir(direct1):
#      os.makedirs(direct1)
    

# direct2 = os.path.join(name+'/_frames2')    
# if not os.path.isdir(direct2):
#      os.makedirs(direct2)
    
# direct3 = os.path.join(name+'/_frames3')    
# if not os.path.isdir(direct3):
#      os.makedirs(direct3)
        
# direct4 = os.path.join(name+'/_frames4')    
# if not os.path.isdir(direct4):
#      os.makedirs(direct4)
    

# direct5 = os.path.join(name+'/_frames5')    
# if not os.path.isdir(direct5):
#      os.makedirs(direct5)
    
# direct6 = os.path.join(name+'/_frames6')    
# if not os.path.isdir(direct6):
#      os.makedirs(direct6)


    
# animate=True
# Field=True
# com=True
# Force=True
# vector_field=True
# #xf=np.linspace(-d,d,20)
# #yf=np.linspace(-d,d,20)  
# #X,Y=np.meshgrid(xf,yf)
# #Z=np.ones((len(yf),len(xf)))
# #Z=-0.25*Z

# u = np.linspace(0, np.pi, 4)
# v = np.linspace(0, 2 * np.pi, 4)
# X = particle_width*np.outer(np.sin(u), np.cos(v))
# Y = particle_width*np.outer(np.sin(u), np.sin(v))
# Z = particle_width*np.outer(np.cos(u), np.ones_like(v))

# # T=0
# # if animate==True:
#     for i in tqdm(range(timesteps)):
#         T+=1
    
#         if T%timesteps==0:
#             pass
#             print(T)
#         if T>timesteps:
#             break
#         pass   
#         #if animate==True:
            
#         fm._rebuild()
#         plt.rcParams['font.family'] = 'Times New Roman'
#         plt.rcParams['mathtext.fontset'] = 'dejavuserif'
#         plt.rcParams['font.size'] = 10
#         plt.rcParams['axes.linewidth'] = .1  
#         fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
#         ax = fig.gca(projection='3d')
#         ax.view_init(elev=0, azim=0)
#         ax.scatter3D(bot_positions_savex[:,i],bot_positions_savey[:,i],bot_positions_savez[:,i], c='k')
#         for j in range(ni):
#             x = X + Particle_positions_savex[j,i] 
#             y = Y + Particle_positions_savey[j,i] 
#             z = Z + Particle_positions_savez[j,i] 
#             ax.plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
        
#         datas=DATA[i]
#         vertices = np.asarray(datas[1])
#         ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = faces,edgecolor=[[0,0,0]], linewidth=.25, alpha=0.25,color='tab:red', shade=False)
#         ax.set_title("Time="+str(np.round(Time[i],3)))
#         ax.set_xticks(xticks)
#         ax.set_yticks(yticks)
#         ax.set_zticks(zticks)
#         ax.set_xlabel("$x$")
#         ax.set_ylabel("$y$")
#         ax.set_zlabel("$z$")    
#         ax.set_xlim3d(xmin,xmax)
#         ax.set_ylim3d(ymin,ymax)
#         ax.set_zlim3d(zmin,zmax)    
#         plt.savefig(direct1+"/frame%04d.jpg" % count)              
#         #count=count+1       
#         plt.close('all') 
    
    
#         fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
#         ax = fig.gca(projection='3d')
#         ax.view_init(elev=0, azim=90)
#         ax.scatter3D(bot_positions_savex[:,i],bot_positions_savey[:,i],bot_positions_savez[:,i], c='k')
#         for j in range(ni):
#             x = X + Particle_positions_savex[j,i] 
#             y = Y + Particle_positions_savey[j,i] 
#             z = Z + Particle_positions_savez[j,i] 
#             ax.plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
        
#         datas=DATA[i]
#         vertices = np.asarray(datas[1])
#         ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = faces,edgecolor=[[0,0,0]], linewidth=.25, alpha=0.25,color='tab:red', shade=False)
#         ax.set_title("Time="+str(np.round(Time[i],3)))
#         ax.set_xticks(xticks)
#         ax.set_yticks(yticks)
#         ax.set_zticks(zticks)
#         ax.set_xlabel("$x$")
#         ax.set_ylabel("$y$")
#         ax.set_zlabel("$z$")    
#         ax.set_xlim3d(xmin,xmax)
#         ax.set_ylim3d(ymin,ymax)
#         ax.set_zlim3d(zmin,zmax)    
#         plt.savefig(direct2+"/frame%04d.jpg" % count)              
#         #count=count+1       
#         plt.close('all') 
        
#         fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
#         ax = fig.gca(projection='3d')
#         #ax.view_init(elev=0, azim=90)
#         ax.scatter3D(bot_positions_savex[:,i],bot_positions_savey[:,i],bot_positions_savez[:,i], c='k')
#         for j in range(ni):
#             x = X + Particle_positions_savex[j,i] 
#             y = Y + Particle_positions_savey[j,i] 
#             z = Z + Particle_positions_savez[j,i] 
#             ax.plot_surface(x, y, z, color=np.random.choice(['tab:green']), alpha=1)
        
#         datas=DATA[i]
#         vertices = np.asarray(datas[1])
#         ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = faces,edgecolor=[[0,0,0]], linewidth=.25, alpha=0.25,color='tab:red', shade=False)
#         ax.set_title("Time="+str(np.round(Time[i],3)))
#         ax.set_xticks(xticks)
#         ax.set_yticks(yticks)
#         ax.set_zticks(zticks)
#         ax.set_xlabel("$x$")
#         ax.set_ylabel("$y$")
#         ax.set_zlabel("$z$")    
#         ax.set_xlim3d(xmin,xmax)
#         ax.set_ylim3d(ymin,ymax)
#         ax.set_zlim3d(zmin,zmax)    
#         plt.savefig(direct3+"/frame%04d.jpg" % count)              
#         count=count+1       
#         plt.close('all')             
    
#     create_video(name,'_frames1',name+'1')    
#     create_video(name,'_frames2',name+'2') 
#     create_video(name,'_frames3',name+'3')
    
    
    
 
# # if vector_field==True:
# #     T=0
# #     for i in tqdm(range(timesteps)):
# #         T+=1
    
# #         if T%timesteps==0:
# #             pass
# #             print(T)
# #         if T>timesteps:
# #             break
# #         pass   
# #         #if animate==True:
            
# #         fm._rebuild()
# #         plt.rcParams['font.family'] = 'Times New Roman'
# #         plt.rcParams['mathtext.fontset'] = 'dejavuserif'
# #         plt.rcParams['font.size'] = 10
# #         plt.rcParams['axes.linewidth'] = .1  
# #         fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
# #         ax = fig.gca(projection='3d')
# #         ax.view_init(elev=0, azim=0)
# #         ax.scatter3D(bot_positions_savex[:,i],bot_positions_savey[:,i],bot_positions_savez[:,i], c='k')
# #         ax.quiver(bot_positions_savex[:,i],bot_positions_savey[:,i],bot_positions_savez[:,i], Force_savex[:,i], Force_savey[:,i], Force_savez[:,i], length=0.01)  
# #         ax.set_title("Time="+str(np.round(Time[i],3)))
# #         ax.set_xticks(xticks)
# #         ax.set_yticks(yticks)
# #         ax.set_zticks(zticks)
# #         ax.set_xlabel("$x$")
# #         ax.set_ylabel("$y$")
# #         ax.set_zlabel("$z$")    
# #         ax.set_xlim3d(xmin,xmax)
# #         ax.set_ylim3d(ymin,ymax)
# #         ax.set_zlim3d(zmin,zmax)    
# #         plt.savefig(direct4+"/frame%04d.jpg" % count)              
# #         #count=count+1       
# #         plt.close('all') 
    
    
# #         fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
# #         ax = fig.gca(projection='3d')
# #         ax.view_init(elev=0, azim=90)
# #         ax.scatter3D(bot_positions_savex[:,i],bot_positions_savey[:,i],bot_positions_savez[:,i], c='k')
# #         ax.quiver(bot_positions_savex[:,i],bot_positions_savey[:,i],bot_positions_savez[:,i], Force_savex[:,i], Force_savey[:,i], Force_savez[:,i], length=0.01)  
# #         ax.set_title("Time="+str(np.round(Time[i],3)))
# #         ax.set_xticks(xticks)
# #         ax.set_yticks(yticks)
# #         ax.set_zticks(zticks)
# #         ax.set_xlabel("$x$")
# #         ax.set_ylabel("$y$")
# #         ax.set_zlabel("$z$")    
# #         ax.set_xlim3d(xmin,xmax)
# #         ax.set_ylim3d(ymin,ymax)
# #         ax.set_zlim3d(zmin,zmax)    
# #         plt.savefig(direct5+"/frame%04d.jpg" % count)              
# #         #count=count+1       
# #         plt.close('all') 
    
        
# #         fig = plt.figure(figsize = (6, 6),dpi=300)          # Set the figure size
# #         ax = fig.gca(projection='3d')
# #         #ax.view_init(elev=0, azim=90)
# #         ax.scatter3D(bot_positions_savex[:,i],bot_positions_savey[:,i],bot_positions_savez[:,i], c='k')
# #         ax.quiver(bot_positions_savex[:,i],bot_positions_savey[:,i],bot_positions_savez[:,i], Force_savex[:,i], Force_savey[:,i], Force_savez[:,i], length=0.01)  
# #         ax.set_title("Time="+str(np.round(Time[i],3)))
# #         ax.set_xticks(xticks)
# #         ax.set_yticks(yticks)
# #         ax.set_zticks(zticks)
# #         ax.set_xlabel("$x$")
# #         ax.set_ylabel("$y$")
# #         ax.set_zlabel("$z$")    
# #         ax.set_xlim3d(xmin,xmax)
# #         ax.set_ylim3d(ymin,ymax)
# #         ax.set_zlim3d(zmin,zmax)    
# #         plt.savefig(direct6+"/frame%04d.jpg" % count)              
# #         count=count+1       
# #         plt.close('all') 
               
    
# #     create_video(name,'_frames4',name+'4')    
# #     create_video(name,'_frames5',name+'5') 
# #     create_video(name,'_frames6',name+'6')
    
        
# # if Field==True:
# #     Fv=[]
    
# #     for i in range(timesteps):
# #         Fv.append(np.sum(abs(F_value[:,i])))
        
# #     fm._rebuild()
# #     plt.rcParams['font.family'] = 'Times New Roman'
# #     plt.rcParams['mathtext.fontset'] = 'dejavuserif'
# #     plt.rcParams['font.size'] = 6
# #     plt.rcParams['axes.linewidth'] = .1    
        
        
# #     fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3),dpi=300)
# #     axs.plot(Time,Fv,linewidth=1,color='k')
# #     x_ticks = np.linspace(Time[0], Time[-1],5,endpoint=True)
# #     y_ticks = np.linspace(np.min(Fv), np.max(Fv),5,endpoint=True)
# #     axs.set_xticks(np.round(x_ticks,2))
# #     axs.set_yticks(np.round(y_ticks,2))
# #     axs.xaxis.set_tick_params(width=.25,length=2)
# #     axs.yaxis.set_tick_params(width=.25,length=2)
# #     axs.set_ylabel(r"$\Sigma \phi$")
# #     axs.set_xlabel("Time (seconds)")
# #     axs.set_title("Field Values")
# #     axs.grid(True)
# #     plt.tight_layout()
# #     plt.savefig(name+"/Field_Values_"+name+'.png')
# #     plt.savefig(name+"/Field_Values_"+name+'.pdf')
# #     plt.close('all') 

# #     bot_num=np.arange(0,nb,1)
# #     T,NB=np.meshgrid(bot_num,Time)
# #     fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,5))
# #     im1=axs.pcolormesh(Time,bot_num,abs(F_value), cmap='jet')
# #     axs.invert_yaxis()
    
# #     fig.colorbar(im1, ax=axs)

# #     plt.savefig(name+"/Field_Valuesgrid_"+name+'.png')
# #     plt.savefig(name+"/Field_Valuesgrid_"+name+'.pdf')
# #     plt.close('all') 
# # if com==True:
     
# #     fm._rebuild()
# #     plt.rcParams['font.family'] = 'Times New Roman'
# #     plt.rcParams['mathtext.fontset'] = 'dejavuserif'
# #     plt.rcParams['font.size'] = 6
# #     plt.rcParams['axes.linewidth'] = .1    
    
    
# #     Xc=[]
# #     Yc=[]
# #     Zc=[]
# #     for i in range(timesteps):
# #         datas=DATA[i]    
# #         vertices = np.asarray(datas[1]) 
# #         Xc.append(np.sum(vertices[:,0])/len(vertices[:,0]))
# #         Yc.append(np.sum(vertices[:,1])/len(vertices[:,1]))
# #         Zc.append(np.sum(vertices[:,2])/len(vertices[:,2]))
        
        
# #     fig, axs = plt.subplots(nrows=3, ncols=1,figsize=(6,3),dpi=300)
    
# #     axs[0].plot(Time,Xc,linewidth=1,color='tab:red')
# #     axs[0].set_ylabel(r"$x$")
# #     axs[0].set_xlabel("Time (seconds)")
# #     axs[0].grid(True)
    
# #     axs[1].plot(Time,Yc,linewidth=1,color='tab:green')
# #     axs[1].set_ylabel(r"$y$")
# #     axs[1].set_xlabel("Time (seconds)")
# #     axs[1].grid(True)
    
# #     axs[2].plot(Time,Zc,linewidth=1,color='tab:blue')
# #     axs[2].set_ylabel(r"$z$")
# #     axs[2].set_xlabel("Time (seconds)")
# #     axs[2].grid(True)    
    
    
# #     plt.tight_layout()
    
    
    
# #     plt.savefig(name+"/COM_"+name+'.png')
# #     plt.savefig(name+"/COM_"+name+'.pdf')
# #     plt.close('all') 
    
    
# # if Force==True:
     
# #     fm._rebuild()
# #     plt.rcParams['font.family'] = 'Times New Roman'
# #     plt.rcParams['mathtext.fontset'] = 'dejavuserif'
# #     plt.rcParams['font.size'] = 6
# #     plt.rcParams['axes.linewidth'] = .1    
    
    
# #     fig, axs = plt.subplots(nrows=3, ncols=1,figsize=(6,10),dpi=300) 
# #     #for i in range(nb):
# #        #axs[0].plot(Time,Force_savex[i,:],linewidth=1,color='tab:red')
# #     bot_num=np.arange(0,nb,1)
# #     #fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,5))
# #     im1=axs[0].pcolormesh(Time,bot_num,Force_savex, cmap='jet')
# #     fig.colorbar(im1, ax=axs[0])
    
# #     axs[0].set_ylabel(r"$Fx$")
# #     axs[0].set_xlabel("Time (seconds)")
# #     #axs[0].grid(True)
    
# #     #for i in range(nb):
# #     #    axs[1].plot(Time,Force_savey[i,:],linewidth=1,color='tab:blue')
    

# #     im1=axs[1].pcolormesh(Time,bot_num,Force_savey, cmap='jet')
# #     fig.colorbar(im1, ax=axs[1])
    
# #     axs[1].set_ylabel(r"$Fy$")
# #     axs[1].set_xlabel("Time (seconds)")
    
# #     #for i in range(nb):
# #         #axs[2].plot(Time,Force_savez[i,:],linewidth=1,color='tab:green')
    
# #     im1=axs[2].pcolormesh(Time,bot_num,Force_savez, cmap='jet')
# #     fig.colorbar(im1, ax=axs[2])
    
# #     axs[2].set_ylabel(r"$Fz$")
# #     axs[2].set_xlabel("Time (seconds)")
# #     plt.tight_layout()
    
    
    
# #     plt.savefig(name+"/Force_"+name+'.png')
# #     plt.savefig(name+"/Force_"+name+'.pdf')
# #     plt.close('all')             
            