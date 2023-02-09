
import numpy as np
import random
import os
import csv
from csv import writer
import matplotlib.pyplot as plt


class R_functions():  
    """ R-function Class """
    def __init__(self,name):
        self.direct = os.path.dirname(__file__)
        self.name = name
        ###### Imported Variables #########
        self.mainDirectory = self.direct+"/Experiments/"
        parameters = np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters = parameters.tolist()
        self.control_mode=self.parameters['control_mode']  # control mode
        
        ######### SHAPE FORMATION #########
        if self.control_mode=="shape_formation":
            self.geometry = self.parameters['geometry'] 

            if self.geometry=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['R']
            
            else:
                data = np.load(self.direct+'/shapes/'+self.geometry+'.npz')
                self.segments = data['segments']
                self.scale = self.parameters['scale']
                self.segments=self.scale*self.segments

        ######### SHAPE MORPHING #########
        if self.control_mode=="shape_morphing":  
            self.p = self.parameters['p']
            self.m = 2 
            self.geometry1 = self.parameters['geometry1'] 
            self.geometry2 = self.parameters['geometry2'] 
            self.scale1 = self.parameters['scale1']
            self.scale2 = self.parameters['scale2']
            
            
            if self.geometry1=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['R']
                
                
            if self.geometry2=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['R']


            if self.geometry1!='circle':                 
                data = np.load(self.direct+'/shapes/'+self.geometry1+'.npz')
                self.segments = data['segments']
                self.segments = self.scale1*self.segments                   
                
            
            if self.geometry2!='circle':                 
                data = np.load(self.direct+'/shapes/'+self.geometry2+'.npz')
                self.segments = data['segments']
                self.segments = self.scale2*self.segments                   
                          
            
            
##############################################################################            
            
    def phi_circle(self,x,y,a,b,R):
        """ Normalized distance function of a circle """
        phi = (R**2 - (x-a)**2 - (y-b)**2)/(2*R)
        return(abs(phi))
     
        
    def dphix_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt x """
        return(R*(x-a)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def dphiy_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt y """
        return(R*(y-b)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
##############################################################################   

    def phi_line_(self,x,y,x1,y1,x2,y2):
        """ Distance function of a line """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)


    def dphix_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt x """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)


    def dphiy_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt y """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)


    def trim(self,x,y,x1,y1,x2,y2):
        """ Trim function """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc = np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)


    def dtrimx(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt x """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def dtrimy(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt y """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def phi_line(self,x,y,x1,y1,x2,y2):
        """ Trimmed line segment"""
        t = self.trim(x,y,x1,y1,x2,y2)
        f = self.phi_line_(x,y,x1,y1,x2,y2)
        rho = np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((rho-t)/2)**2))

    def dphix_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt x"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfx = self.dphix_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dtx = self.dtrimx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

    def dphiy_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt y"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfy = self.dphiy_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dty = self.dtrimy(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)   


    def phi_segments(self,x,y,segments):
        """ R equivelent of trimmed line segments"""
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi_line(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**self.m
        R = 1/R**(1/self.m)
        return(R)

    def dphix_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt x"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) * self.dphix_line(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)

    def dphiy_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt y"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**(-self.m)) * self.dphiy_line(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**(-self.m))**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**(-self.m)) + term3      
        R=(-term1*term2/term3)
        return(R)
    

    def FX(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def FY(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)    
    
#### MORPHING FUNCTIONS
    
    def F1(self,x,y):
        if self.geometry1=='circle':
            F1 = self.phi_circle(x,y,self.a,self.b,self.R)
        else:
            F1 = self.dphix_segments(x,y,self.segments1)
    
        return(F1)        
        
    def F2(self,x,y):
        if self.geometry2=='circle':
            F2 = self.phi_circle(x,y,self.a,self.b,self.R)
        else:
            F2 = self.phi_segments(x,y,self.segments)
    
        return(F2)      


    def F1X(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry1=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def F1Y(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry1=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)    


    def F2X(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry2=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def F2Y(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry2=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)  









    def tanh(self,t):
        """ tanh function """
        tanh=(np.exp(self.p*(t))-1)/(np.exp(self.p*(t))+1)
        #print('tanh=',tanh)
        return(tanh)   
    
    
    def g1(self,phi1,t):
        """ intersection of initial field and -f(t) """
        return(phi1 - t - np.sqrt(phi1**2 + t**2))
        
    def g2(self,phi2,t):
        """ intersection of final field and f(t)-1 """
        return(phi2 + (t-1) - np.sqrt(phi2**2+(t-1)**2))
    
    
    def dgx(self,phi,dphix,s):
        """ derivative of g1 or g2 wrt x """
        # s is either (t) or t-1
        return(dphix-(phi*dphix)/(np.sqrt(s**2 +phi**2)))
    
    
    def dgy(self,phi,dphiy,s):
        """ derivative of g1 or g2 wrt y """
        return(dphiy-(phi*dphiy)/(np.sqrt(s**2 +phi**2)))    
    
    
    def w1(self,g1,g2): 
        """ Weighted function 1 """
        return(g2/(g1+g2))
    
    
    def w2(self,g1,g2):
        """ Weighted function 2 """
        return(g1/(g1+g2))    
    
  
    def dw1x(self,dg1x,dg2x,g1,g2):
        """ derivative of weighted function 1 wrt x """
        return((dg2x) / (g1+g2) - (dg1x+dg2x)*g2 / (g1+g2)**2)
    
    
    def dw2x(self,dg1x,dg2x,g1,g2):
        """ derivative of weighted function 2 wrt x """
        return((dg1x) / (g1+g2) - (dg1x+dg2x)*g1 / (g1+g2)**2)
    
    
    def dw1y(self,dg1y,dg2y,g1,g2):
        """ derivative of weighted function 1 wrt y """
        return((dg2y) / (g1+g2) - (dg1y+dg2y)*g2 / (g1+g2)**2)
    
    
    def dw2y(self,dg1y,dg2y,g1,g2):
        """ derivative of weighted function 2 wrt y """
        return((dg1y) / (g1+g2) - (dg1y+dg2y)*g1 / (g1+g2)**2)  
    
  

    def C_morph(self,phi1,phi2,ft):
        """ Morphing function from phi1 to phi2 """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        
        return(W1*phi1+W2*phi2)
    
    

    def C_morphx(self,phi1,phi2,dphi1x,dphi2x,ft):
        """ Derivative Morphing function from phi1 to phi2 wrt x """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        
        dg1x = self.dgx(phi1,dphi1x,ft)
        dg2x = self.dgx(phi2,dphi2x,ft-1)
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        DW1X = self.dw1x(dg1x,dg2x,G1,G2)
        DW2X = self.dw2x(dg1x,dg2x,G1,G2)
        
        return(phi1*DW1X + phi2*DW2X + W1*dphi1x + W2*dphi2x)
    
    def C_morphy(self,phi1,phi2,dphi1y,dphi2y,ft):
        """ Derivative Morphing function from phi1 to phi2 wrt y """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        
        dg1y = self.dgy(phi1,dphi1y,ft)
        dg2y = self.dgy(phi2,dphi2y,ft-1)
    
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        
        DW1Y = self.dw1y(dg1y,dg2y,G1,G2)
        DW2Y = self.dw2y(dg1y,dg2y,G1,G2)
        
        return(phi1*DW1Y + phi2*DW2Y + W1*dphi1y + W2*dphi2y)




    def create_segment(self,x,y):
        """ Create segment matrix for points of R-function """
        seglen=len(x)
        segments=np.zeros((seglen-1,4))
        for i in range(seglen-1):
            #[x1,y1,x2,y2]
            #[x2,y2,x3,y3]
            segments[i,0]=x[i]
            segments[i,1]=y[i]
            segments[i,2]=x[i+1]
            segments[i,3]=y[i+1]
        return(segments)        
 
    def plot_R_function(self,X,Y,R,Rx,Ry,d):
        """ Plot R-function and its derivatives """
        xticks = np.linspace(-d, d,5,endpoint=True)
        fig, axs = plt.subplots(nrows=1, ncols=3,figsize=(10,3))
        fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)
        
        # Plot Phi
        im1=axs[0].contourf(X, Y,R,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        axs[0].contour(X,Y,R,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        #axs[0].plot(xp,yp,color='k',linewidth=3)
        axs[0].set_title('$\phi(x)$')
        axs[0].set_xticks(xticks)
        axs[0].set_yticks(xticks)
        fig.colorbar(im1, ax=axs[0])
        
        # Plot Phix
        im2=axs[1].contourf(X, Y,Rx,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        #axs[1].plot(xp,yp,color='k',linewidth=3)
        axs[1].contour(X,Y,Rx,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        axs[1].set_xticks(xticks)
        axs[1].set_yticks(xticks)
        axs[1].set_title(r"$\frac{\partial \phi}{\partial x}$")
        fig.colorbar(im2, ax=axs[1])
        
        # Plot Phiy
        im3=axs[2].contourf(X,Y,Ry,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        #axs[2].plot(xp,yp,color='k',linewidth=3)
        axs[2].contour(X,Y,Ry,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        axs[2].set_xticks(xticks)
        axs[2].set_yticks(xticks)
        axs[2].set_title(r"$\frac{\partial \phi}{\partial y}$")
        fig.colorbar(im3, ax=axs[2])
         
        
        
    def plot_R_function_morph(self,X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t):        
        
        C=[]
        CX=[]
        CY=[]
        for i in range(len(t)):
            C.append(self.C_morph(phi1,phi2,t[i]))
            CX.append(self.C_morphx(phi1,phi2,dphi1x,dphi2x,t[i]))
            CY.append(self.C_morphy(phi1,phi2,dphi1y,dphi2y,t[i]))
            
        fig, axs = plt.subplots(nrows=nr, ncols=nc,figsize=(6,6),dpi=150)
        count=0
        xticks=np.linspace(-d,d,5)
        yticks=xticks
        for i in range(nr):
            for j in range(nc):
                #print(np.min(C[count]))
                CS1 = axs[i,j].contour(X,Y,CX[count],levels = [0],colors=('tab:red'),linestyles=('-',),linewidths=(3,))
                CS2 = axs[i,j].contour(X,Y,CY[count],levels = [0],colors=('tab:blue'),linestyles=('--',),linewidths=(3,))
                #(xi,yi)=find_intersection(CS1,CS2)
                #axs[i,j].plot(xi,yi,'ko', ms=3)
                #CS2 = axs[i,j].contour(X,Y,CX[count],5,colors='black',zorder=0)
                #axs[i,j].set_xlim([-d,d])
                #axs[i,j].set_ylim([-d,d])
                axs[i,j].set_title(str(np.round(t[count],3)),fontsize=10,fontname="Arial")
                axs[i,j].set_xticks(xticks)
                axs[i,j].set_yticks(yticks)
                count=count+1
        fig.tight_layout()
        
        
        
    def plot_R_function_morph_color(self,X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t):        
        
        C=[]
        CX=[]
        CY=[]
        for i in range(len(t)):
            C.append(self.C_morph(phi1,phi2,t[i]))
            CX.append(self.C_morphx(phi1,phi2,dphi1x,dphi2x,t[i]))
            CY.append(self.C_morphy(phi1,phi2,dphi1x,dphi2x,t[i]))
            
        fig, axs = plt.subplots(nrows=nr, ncols=nc,figsize=(6,6),dpi=150)
        count=0
        xticks=np.linspace(-d,d,5)
        yticks=xticks        
        for i in range(nr):
            for j in range(nc):
                print(np.min(CX[count]))
                #axs[i,j].axis('equal')
                #CS1 = axs[i,j].contour(X,Y,C[count],linewidths=(1,))
                im1=axs[i,j].contourf(X, Y,C[count],cmap = 'jet',levels=50,alpha=1,linestyles='solid')  
                axs[i,j].set_title(str(np.round(t[count],3)),fontsize=10,fontname="Arial")
                axs[i,j].set_xticks(xticks)
                axs[i,j].set_yticks(yticks)
                count=count+1
                fig.colorbar(im1, ax=axs[i,j])
        plt.tight_layout()        
        
        