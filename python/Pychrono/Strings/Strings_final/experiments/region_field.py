import numpy as np
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import matplotlib.pyplot as plt


class Region_fields:
    def __init__(self,R,px,py,b,res,shape):
        self.px=px # center x
        self.py=py # center y
        self.R1=R
        self.R2=self.R1-5


        
        self.b=b # range of field
        self.res=res # resolution of the field so how big are the squares 
        self.shape=shape
        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        self.nr=[0,10,20,30]
        self.nr2=[-10,-20]
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field    
        (self.x,self.y,self.z,self.x2,self.y2,self.z2)=self.points_for_region()
        
        self.rbf = Rbf(self.x,self.y,self.z,function='thin_plate')  # radial basis function interpolator instance    
        self.rbf2 = Rbf(self.x2,self.y2,self.z2,function='thin_plate')  # radial basis function interpolator instance
        self.zz1 = self.rbf(self.xx, self.yy)
        self.zz1=np.nan_to_num(self.zz1/np.max(self.zz1))
        self.zz1=np.maximum(0,self.zz1)
        
        # inner region of the field 
        self.zz2 = self.rbf2(self.xx, self.yy)
        self.zz2=np.nan_to_num(self.zz2/np.max(self.zz2))
        self.zz2=np.maximum(0,self.zz2)
        #add them up and find fields and gradients
        self.zz=self.zz2+self.zz1
        U, S, VT = np.linalg.svd(self.zz,full_matrices=False)
        S = np.diag(S)
        #r=5
        #self.zz = U[:,:r] @ S[0:r,:r] @ VT[:r,:]  
        self.f = RegularGridInterpolator((self.yp,self.xp),self.zz)
        self.fy,self.fx=np.gradient(self.zz)
        self.fnx = RegularGridInterpolator((self.yp,self.xp),self.fx)
        self.fny = RegularGridInterpolator((self.yp,self.xp),self.fy)        

    
    def points_for_region(self):
        if self.shape=='Square':
            (x,y,z,x2,y2,z2)=self.points_square_region()
            
        if self.shape=='Triangle':
            (x,y,z,x2,y2,z2)=self.points_triangle_region()   
            
        if self.shape=='Circle':
            (x,y,z,x2,y2,z2)=self.points_circle_region()
        
        if self.shape=='oval':
            (x,y,z,x2,y2,z2)=self.points_oval_region()
            
        return(x,y,z,x2,y2,z2)
    
    def points_square_region(self):
        xt=np.array([1,1,1,1,1,.75,.5,.25,0,-.25,-.5,-.75,-1,-1,-1,-1,-1,-1,-1,-1,-1,-.75,-.5,-.25,0,.25,.5,.75,1,1,1,1])
        yt=np.array([0,.25,.5,.75,1,1,1,1,1,1,1,1,1,.75,.5,.25,0,-.25,-.5,-.75,-1,-1,-1,-1,-1,-1,-1,-1,-1,-.75,-.5,-.25])
        
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        
        x2=np.zeros(len(self.nr2)*len(xt))
        y2=np.zeros(len(self.nr2)*len(xt))
        z2=np.zeros(len(self.nr2)*len(xt))
        
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.R1+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(self.R1+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0                  
                else:
                    z[i+len(xt)*j]=j 
                    
        for j in range(len(self.nr2)):
            for i in range(len(xt)):
                x2[i+len(xt)*j]=(self.R1+self.nr2[j])*xt[i]
                y2[i+len(xt)*j]=(self.R1+self.nr2[j])*yt[i]
                if j==0:
                    z2[i+len(xt)*j]=0                  
                else:
                    z2[i+len(xt)*j]=j  
                    
        return (x,y,z,x2,y2,z2) 
    
    def points_triangle_region(self):
        #n=3
        m=3
        n1=4.5
        n2=10
        n3=10
        a=1
        b=1
        self.nr=[0,10,20]
        self.nr2=[0,5,10]
        theta=np.linspace(0,2*np.pi,30)
        #(xt,yt)=self.super_ellipse(m,n1,n2,n3,a,b,theta)
        n=3
        r=np.cos(np.pi/n)/np.cos((theta%(2*np.pi/n))-(np.pi/n))
        xt=r*np.cos(theta)
        yt=r*np.sin(theta)
        
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        
        x2=np.zeros(len(self.nr2)*len(xt))
        y2=np.zeros(len(self.nr2)*len(xt))
        z2=np.zeros(len(self.nr2)*len(xt))
        
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.R1+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(self.R1+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0                  
                else:
                    z[i+len(xt)*j]=j 
                    
        for j in range(len(self.nr2)):
            for i in range(len(xt)):
                x2[i+len(xt)*j]=(self.R2-self.nr2[j])*xt[i]
                y2[i+len(xt)*j]=(self.R2-self.nr2[j])*yt[i]
                if j==0:
                    z2[i+len(xt)*j]=0                  
                else:
                    z2[i+len(xt)*j]=j  
                           
        return (x,y,z,x2,y2,z2)  
    
    def points_oval_region(self):
        self.nr=[0,10,20,30]
        self.nr2=[0,5,10]
        theta=np.linspace(0,2*np.pi,30)

        xt=np.cos(theta)
        yt=np.sin(theta)
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        
        x2=np.zeros(len(self.nr2)*len(xt))
        y2=np.zeros(len(self.nr2)*len(xt))
        z2=np.zeros(len(self.nr2)*len(xt))
        
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.R1+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(self.R1-20+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0                  
                else:
                    z[i+len(xt)*j]=j 
                    
        for j in range(len(self.nr2)):
            for i in range(len(xt)):
                x2[i+len(xt)*j]=(self.R2-self.nr2[j])*xt[i]
                y2[i+len(xt)*j]=(self.R2-20-self.nr2[j])*yt[i]
                if j==0:
                    z2[i+len(xt)*j]=0                  
                else:
                    z2[i+len(xt)*j]=j  
                           
        return (x,y,z,x2,y2,z2)  

    
    
    
    def plot_2D(self,typed):
        fig = plt.figure(figsize = (8, 8))     # Set the figure size to square
        plt.pcolor(self.xx,self.yy, self.zz, cmap = 'jet')         # Pseudocolor plot of data and choose colormap
        plt.colorbar()  
        plt.title('$f(x,y)$ for a'+typed,fontname="Arial", fontsize=14 )
        plt.xlabel("$x$ meters",fontname="Arial", fontsize=14)
        plt.ylabel("$y$ meters",fontname="Arial", fontsize=14)
        plt.show()
        
    def plot_2D_lines(self):
        fig, ax = plt.subplots(figsize = (8, 8))
        CS = ax.contour(self.xx,self.yy,self.zz,10)
        ax.clabel(CS, inline=1, fontsize=10)
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$") 
        
        
        
    def plot_3D(self):
        fig = plt.figure(figsize = (8, 8))          # Set the figure size
        ax = fig.gca(projection='3d')                   # Include axes
        surf = ax.plot_surface(self.xx,self.yy,self.zz,cmap='jet', )   # Plot the 3-D surface using the "jet" color map
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.title('plot')
        fig.colorbar(surf)                              # Include color bar
        # Annotate text on figure at x, y, z coordinate
        plt.show() 
        
    def slice_plot(self,typed,R):
        fig = plt.figure(figsize = (8, 8))     # Set the figure size to square
        y=np.zeros(len(self.xp))
        z=self.f((y,self.xp))
        plt.plot(self.xp,z)
        plt.title('$f(x,y)$ for a '+typed,fontname="Arial", fontsize=14 )
        plt.xlabel("$x$ meters",fontname="Arial", fontsize=14)
        plt.ylabel("$y$ meters",fontname="Arial", fontsize=14)
        plt.grid(True)
        plt.xlim(R-10,R+10)
        plt.ylim(0,.01)