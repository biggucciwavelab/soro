# -*- coding: utf-8 -*-
"""
Created on Tue Feb 23 10:03:48 2021

@author: qiyua
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from multiprocessing import Pool
from itertools import product
import config as conf
import time
import os
from scipy.spatial import KDTree
from scipy import interpolate

class mp_plotter:
    def __init__(self,config,order):
        self.rad = config.rob_rad
        self.sim_id = config.sim_id
        self.ratio_meas = config.ratio_meas
        self.nstep = int((config.tend*config.sens_rate)-1)
        self.tstep = 1./float(config.sens_rate)
        self.nb = config.rob_nb
        self.order = order
        self.outdir = config.sim_id+'/save_data/'
        self.positions = np.zeros((self.nstep,self.nb,2)) # [tstep,bot,(x,y)] bot position info
        
        if not os.path.exists(self.outdir):
            os.makedirs(self.outdir)
            
    def load_positions(self,step):
        data = np.load(self.sim_id+'/save_data/lmap_frames/pos'+str(step+1)+'.npy')[-self.nb:,:]
        return data
        
    def load_pos_mp(self):
        nstep = self.nstep
        star = time.time()
        pool = Pool()
        self.positions = np.asarray(pool.map(self.load_positions, range(nstep)))
        pool.close()
        
        stop = time.time()
        mins = int(np.floor((stop - star)/60.0))
        secs = round((stop-star) - mins*60,3)
        print('Elapsed time loading positions from disk: ', mins, ' min ', secs, ' s')
        
    def plot_lmap(self,step):
        
        # Load data
        step+=1
        data1=np.load(self.sim_id+'/save_data/lmap_frames/map'+str(step)+'.npy')
        data2=np.load(self.sim_id+'/save_data/lmap_frames/pos'+str(step)+'.npy')
        data3=np.load(self.sim_id+'/save_data/lmap_frames/gtmap'+str(step)+'.npy')
        
        # Elliptic Fourier function
        contour = data2[-self.nb:-1:self.ratio_meas+1,:]
        xc = np.mean(contour[:,0])
        yc = np.mean(contour[:,1])
        phi = elliptic_fourier(contour,self.order,self.nb)
        phi.gen_elliptic()
        
        # Plot data
        plt.plot(data2[:,0], data2[:,1],'o',markerfacecolor='none',markeredgecolor='tab:green',markersize=4)
        plt.plot(data1[:,0], data1[:,1],'o',markerfacecolor='teal',markersize=9)
        plt.plot(data3[:,0], data3[:,1],'o',markerfacecolor='none',markeredgecolor='red',markersize=3)
        plt.plot(phi.xt,phi.yt,c='black',linewidth=1)
        plt.xlim((xc-2.0*self.rad, xc+2.0*self.rad))
        plt.ylim((yc-2.0*self.rad, yc+2.0*self.rad))
        plt.text(xc-1.75*self.rad, yc-1.75*self.rad, 'ellipic error: ' + str(round(100*phi.error,1)) + '[cm]')
        plt.xlabel('X Position [m]')
        plt.ylabel('Y Position [m]')
        plt.legend(('Robot Positions','Obstacles: Estimate','Obstacles: Truth','Ellipic Fourier'),loc='center left', bbox_to_anchor=(1.04, 0.5))
        plt.title('Mapped Obstacles')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.tight_layout()
        plt.savefig(self.sim_id+'/save_data/lmap_frames/plot'+str(step)+'.png')
        plt.close()
                
    def save_lmap(self):
        nstep = self.nstep
        start = time.time()
        pool = Pool()
        pool.map(self.plot_lmap, range(nstep))
        pool.close()
        
        end = time.time()
        mins = int(np.floor((end - start)/60.0))
        secs = round((end-start) - mins*60,3)
        print('Elapsed plotting time: ', mins, ' min ', secs, ' s')
    
    # Plot mean error vs time for elliptical fourier functions
    def plot_ellip_error_time(self,step):
        contour = self.positions[step,0::self.ratio_meas+1,:]
        phi = elliptic_fourier(contour,self.order,self.nb)
        phi.gen_elliptic()
        return [self.tstep*float(step),phi.error]
        
    # Plot time average error vs order    
    def plot_ellip_error_order(self,step,order):
        contour = self.positions[step,0::self.ratio_meas+1,:]
        phi = elliptic_fourier(contour,order+1,self.nb)
        phi.gen_elliptic()
        
        return [self.tstep*float(step),phi.error,order,phi.errorc]
        
    # Plot mean error vs time given # inactive
    def plot_ellip_error_inactive(self,order,inactive):
        avg_errors = np.zeros(self.nstep)
        avg_errorsc = np.zeros(self.nstep)
        
        for step in range(self.nstep):
            contour = self.positions[step,0::inactive+1,:]
            ref = self.positions[step,0:,:]
            phi = elliptic_fourier(contour,order+1,len(contour),ref)
            phi.gen_elliptic()
            avg_errors[step] = phi.error
            avg_errorsc[step] = phi.errorc
            
        return [order,inactive,np.mean(avg_errors),np.mean(avg_errorsc)]
        
    def save_ellip_error(self,mode):
        nstep = self.nstep
        start = time.time()
        pool = Pool()
        
        # Error vs time for single order
        if (mode == 1):
            error_time = np.asarray(pool.map(self.plot_ellip_error_time, range(nstep)))
            
            # Plot
            fig = plt.figure()
            plt.plot(error_time[:,0],100.*error_time[:,1])
            plt.xlabel('Time [s]')
            plt.ylabel('Average error [cm]')
            plt.title('Elliptical Fourier Descriptor Error vs Time')
            plt.tight_layout()
            plt.show()
            
        # Error vs order averaged over time
        if (mode == 2): 
            orders = 50
            params = product(range(nstep),range(orders))
            error_order = np.asarray(pool.starmap(self.plot_ellip_error_order, params))
            error_order = error_order[np.argsort(error_order[:,2])]
            
            errors = []
            for order in range(orders):
                average_error = np.mean(error_order[order*nstep:nstep*(order+1),1])
                errors.append(average_error)
            
            # Plot accuracy vs orders
            fig = plt.figure()
            plt.plot(range(orders),errors)
            plt.xlabel('Orders')
            plt.ylabel('Average error [cm]')
            plt.title('Elliptical Fourier Descriptor Error vs Order')
            plt.tight_layout()
            plt.show()
            
        # Subplots of error vs time and error vs order averaged over time 
        if (mode == 12):
            
            # Create time, error, order array
            nrows = 2
            ncols = 3
            orders = nrows*ncols
            params = product(range(nstep),range(orders))
            error_order = np.asarray(pool.starmap(self.plot_ellip_error_order, params))
            error_order = error_order[np.argsort(error_order[:,2])]
            colors = ['tab:red','tab:orange','gold','tab:green','tab:blue','tab:purple']
            
            fig = plt.figure(figsize=(8,6),dpi=100)
            fig.suptitle('EFD Order Errors', fontsize=20)
            gs = fig.add_gridspec(nrows, 2+ncols)
            
            counter = 0
            # Error vs time for each order subplots
            # for i in range(nrows):
            #     for j in range(ncols):
            #         sta = counter*nstep
            #         sto = nstep*(counter+1)
            #         vis = fig.add_subplot(gs[i,j])
            #         foo = vis.set_title('Order ' +str(counter+1))
            #         vis.plot(error_order[sta:sto,0],100.*error_order[sta:sto,1],'b.',markersize=0.25)
            #         vis.set_ylim((0,1+100*np.max((error_order[sta:sto,1]))))
            #         counter+=1
                    
            # Error vs time all on one plot
            vis = fig.add_subplot(gs[0:,0:ncols])
            foo = vis.set_title('Normalized error for COM vs time')
            for i in range(nrows):
                for j in range(ncols):
                    sta = counter*nstep
                    sto = nstep*(counter+1)
                    col = colors[counter]
                    lab = 'Order ' +str(counter+1)
                    vis.scatter(error_order[sta:sto,0],error_order[sta:sto,1]/0.0945,color=col,s=0.25,label=lab)
                    #vis.set_ylim((0,1+100*np.max((error_order[sta:sto,1]))))
                    
                    counter+=1
            vis.legend()
            vis.set_xlabel('Time[s]')
            vis.set_ylabel('Normalized Error')
            
            # Error vs order averaged over time            
            errors = []
            for order in range(orders):
                average_error = np.mean(error_order[order*nstep:nstep*(order+1),1])/0.0945
                errors.append(average_error)
            
            vis = fig.add_subplot(gs[0:,ncols:])
            foo = vis.set_title('Time averaged COM error vs order')
            gs.tight_layout(fig, rect=[0,0,1,0.95],pad=0.25)
            vis.plot(range(1,orders+1),errors,'b',linewidth=4,label='Elliptic Fourier')
            vis.set_xlabel('EFD Order')
            vis.set_ylabel('Normalized Error')
            
        # Subplots of elliptical function orders at single time step
        if (mode == 3):
            step = 2713
            contour = np.load(self.sim_id+'/save_data/lmap_frames/pos'+str(step)+'.npy')[-self.nb:,:]
            
            nrows = 2
            ncols = 4
            
            fig = plt.figure(figsize=(8,6),dpi=100)
            # fig.suptitle('EFD orders, Step '+str(step),fontsize=20)
            gs = fig.add_gridspec(nrows, ncols)
            
            counter = 0
            for i in range(nrows):
                for j in range(ncols):
                    # Elliptic Fourier Function
                    plotting = np.vstack((contour,contour[0,:]))
                    phi = elliptic_fourier(contour,counter+1,self.nb)
                    phi.gen_elliptic()
                    
                    vis = fig.add_subplot(gs[i,j])
                    foo = vis.set_title('Order ' +str(counter+1)); foo=None
                    vis.plot(plotting[:,0],plotting[:,1],'k')
                    vis.plot(phi.xt,phi.yt,'g:')
                    #vis.plot(phi.xi,phi.yi,'r:')
                    vis.set_axis_off()
                    counter+=1
                    
            gs.tight_layout(fig, rect=[0,0,1,0.95],pad=0.25)
          
        # Subplots of elliptical functions at different time steps and # of points
        if (mode == 4):
            tsteps = [750,1500,2715,3500,5000]
            order=5
            nrows = len(tsteps)
            ncols = 5
            fig = plt.figure(figsize=(8,6),dpi=100)
            fig.suptitle('EFD Inactive Bots, Order='+str(order),fontsize=20)
            gs = fig.add_gridspec(nrows, ncols)
            
            for i in range(nrows):
                for j in range(ncols):
                    # Data
                    step = tsteps[i]
                    ref = np.load(self.sim_id+'/save_data/lmap_frames/pos'+str(step)+'.npy')[-self.nb:,:]
                    ref = np.vstack((ref,ref[0,:]))
                    contour = np.load(self.sim_id+'/save_data/lmap_frames/pos'+str(step)+'.npy')[-self.nb::j+1,:]
                    plotting = np.vstack((contour,contour[0,:]))
                    
                    # Elliptic Fourier Function
                    plotting = np.vstack((contour,contour[0,:]))
                    phi = elliptic_fourier(contour,order,self.nb)
                    phi.gen_elliptic()
                    
                    vis = fig.add_subplot(gs[i,j])
                    percent = round(100*(1-(j/(j+1))))
                    foo = vis.set_title(str(percent) + '% active, Step '+str(step)); foo=None
                    vis.plot(ref[:,0],ref[:,1],'k')
                    vis.set_axis_off()
                    vis.plot(plotting[:,0],plotting[:,1],'g')
                    vis.plot(phi.xt,phi.yt,'b:')
                    vis.plot(phi.xi,phi.yi,'r:')
                    
            gs.tight_layout(fig, rect=[0,0,1,0.95],pad=0.25)
        
        # Phase graphs of elliptical function and cubic spline time average accuracy vs order and # of points
        if (mode == 5):
            # Create time, error, order array
            norders = 15
            nspaces = 6
            params = product(range(norders),range(nspaces))
            errors = np.asarray(pool.starmap(self.plot_ellip_error_inactive, params))
            
            self.plot_data  = np.zeros((norders,nspaces))
            self.plot_datac = np.zeros((norders,nspaces))
            counter = 0
            for space in range(nspaces):
                for order in range(norders):
                    self.plot_data [int(errors[counter,0]),int(errors[counter,1])]=errors[counter,2]
                    #self.plot_datac[int(errors[counter,0]),int(errors[counter,1])]=errors[counter,3]
                    counter+=1
                    
            # Ellip Error phase diagram
            percents = np.asarray(100*(1-1/(1+np.linspace(0,nspaces-1,nspaces))),dtype=int)
            fig = plt.figure()
            plt.title('Order and Number Inactive vs Time Average Error: Elliptical Fourier')
            #cmap = plt.get_cmap('summer')
            #plt.pcolormesh(np.linspace(1,norders,norders), percents, 100.*plot_data.T,cmap=cmap)
            plt.pcolormesh(np.linspace(1,norders,norders), np.linspace(0,nspaces-1,nspaces), self.plot_data.T/0.0945)
            plt.colorbar()
            plt.xlabel('Orders')
            plt.ylabel('% Inactive')
            
            # Cubic Interp error phase diagram
            if False:
                fig = plt.figure()
                plt.title('Number Inactive vs Time Average Error: Cubic Interpolation')
                #cmap = plt.get_cmap('summer')
                #plt.pcolormesh(np.linspace(1,norders,norders), percents, 100.*plot_datac.T,cmap=cmap)
                plt.pcolormesh(np.linspace(1,norders,norders), np.linspace(0,nspaces-1,nspaces), self.plot_datac.T/0.0945)
                plt.colorbar()
                plt.xlabel('Orders')
                plt.ylabel('% Inactive')
            
        pool.close()
        end = time.time()
        mins = int(np.floor((end - start)/60.0))
        secs = round((end-start) - mins*60,3)
        print('Elapsed plotting time for mode '+str(mode)+': ', mins, ' min ', secs, ' s')
        
    def heatmap(self,data, row_labels, col_labels, ax=None,
            cbar_kw={}, cbarlabel="", **kwargs):
        """
        Create a heatmap from a numpy array and two lists of labels.
    
        Parameters
        ----------
        data
            A 2D numpy array of shape (N, M).
        row_labels
            A list or array of length N with the labels for the rows.
        col_labels
            A list or array of length M with the labels for the columns.
        ax
            A `matplotlib.axes.Axes` instance to which the heatmap is plotted.  If
            not provided, use current axes or create a new one.  Optional.
        cbar_kw
            A dictionary with arguments to `matplotlib.Figure.colorbar`.  Optional.
        cbarlabel
            The label for the colorbar.  Optional.
        **kwargs
            All other arguments are forwarded to `imshow`.
        """
    
        if not ax:
            ax = plt.gca()
    
        # Plot the heatmap
        im = ax.imshow(data, **kwargs)
    
        # Create colorbar
        cbar = ax.figure.colorbar(im, ax=ax, **cbar_kw)
        cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")
    
        # We want to show all ticks...
        ax.set_xticks(np.arange(data.shape[1]))
        ax.set_yticks(np.arange(data.shape[0]))
        # ... and label them with the respective list entries.
        ax.set_xticklabels(col_labels)
        ax.set_yticklabels(row_labels)
    
        # Let the horizontal axes labeling appear on top.
        ax.tick_params(top=True, bottom=False,
                       labeltop=True, labelbottom=False)
    
        # Rotate the tick labels and set their alignment.
        plt.setp(ax.get_xticklabels(), rotation=-30, ha="right",
                 rotation_mode="anchor")
    
        # Turn spines off and create white grid.
        ax.spines[:].set_visible(False)
    
        ax.set_xticks(np.arange(data.shape[1]+1)-.5, minor=True)
        ax.set_yticks(np.arange(data.shape[0]+1)-.5, minor=True)
        ax.grid(which="minor", color="w", linestyle='-', linewidth=3)
        ax.tick_params(which="minor", bottom=False, left=False)
    
        return im, cbar


    def annotate_heatmap(self,im, data=None, valfmt="{x:.2f}",
                         textcolors=("black", "white"),
                         threshold=None, **textkw):
        """
        A function to annotate a heatmap.
    
        Parameters
        ----------
        im
            The AxesImage to be labeled.
        data
            Data used to annotate.  If None, the image's data is used.  Optional.
        valfmt
            The format of the annotations inside the heatmap.  This should either
            use the string format method, e.g. "$ {x:.2f}", or be a
            `matplotlib.ticker.Formatter`.  Optional.
        textcolors
            A pair of colors.  The first is used for values below a threshold,
            the second for those above.  Optional.
        threshold
            Value in data units according to which the colors from textcolors are
            applied.  If None (the default) uses the middle of the colormap as
            separation.  Optional.
        **kwargs
            All other arguments are forwarded to each call to `text` used to create
            the text labels.
        """
    
        if not isinstance(data, (list, np.ndarray)):
            data = im.get_array()
    
        # Normalize the threshold to the images color range.
        if threshold is not None:
            threshold = im.norm(threshold)
        else:
            threshold = im.norm(data.max())/2.
    
        # Set default alignment to center, but allow it to be
        # overwritten by textkw.
        kw = dict(horizontalalignment="center",
                  verticalalignment="center")
        kw.update(textkw)
    
        # Get the formatter in case a string is supplied
        if isinstance(valfmt, str):
            valfmt = matplotlib.ticker.StrMethodFormatter(valfmt)
    
        # Loop over the data and create a `Text` for each "pixel".
        # Change the text's color depending on the data.
        texts = []
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                kw.update(color=textcolors[int(im.norm(data[i, j]) > threshold)])
                text = im.axes.text(j, i, valfmt(data[i, j], None), **kw)
                texts.append(text)
    
        return texts
class elliptic_fourier:
    def __init__(self,contour,order,npoints,ref=None):
        self.order=order
        self.contour=np.asarray(contour)
        self.ref=ref
        
        # Close the loop if not closed
        if np.all(contour[0,:]!=contour[-1,:]):
            self.contour = np.vstack((self.contour,self.contour[0,:]))
            
        if np.all(self.ref)!=None:
            if np.all(self.ref[0,:]!=self.ref[-1,:]):
                self.ref = np.vstack((self.ref,self.ref[0,:]))
        self.npoints = npoints
        self.n = 360
        self.dxy = np.diff(self.contour, axis=0)
        self.dt = np.sqrt((self.dxy ** 2).sum(axis=1))
        self.t = np.concatenate([([0.]), np.cumsum(self.dt)])
        self.T = self.t[-1]
        self.orders = np.arange(1, self.order + 1)
        self.phi = (2 * np.pi * self.t) / self.T
        self.consts = self.T / (2 * self.orders * self.orders * np.pi * np.pi)
        self.phi = self.phi * self.orders.reshape((self.order, -1))
   
        self.d_cos_phi = np.cos(self.phi[:, 1:]) - np.cos(self.phi[:, :-1])
        self.d_sin_phi = np.sin(self.phi[:, 1:]) - np.sin(self.phi[:, :-1])
   
        self.a = self.consts * np.sum((self.dxy[:, 0] / self.dt) * self.d_cos_phi, axis=1)
        self.b = self.consts * np.sum((self.dxy[:, 0] / self.dt) * self.d_sin_phi, axis=1)
        self.c = self.consts * np.sum((self.dxy[:, 1] / self.dt) * self.d_cos_phi, axis=1)
        self.d = self.consts * np.sum((self.dxy[:, 1] / self.dt) * self.d_sin_phi, axis=1)
       
        self.coeffs = np.concatenate([self.a.reshape((self.order, 1)),self.b.reshape((self.order, 1)),self.c.reshape((self.order, 1)),self.d.reshape((self.order, 1)),],axis=1,)
        self.xi = np.cumsum(self.dxy[:, 0]) - (self.dxy[:, 0] / self.dt) * self.t[1:]
        self.delta = np.cumsum(self.dxy[:, 1]) - (self.dxy[:, 1] / self.dt) * self.t[1:]
        
        self.A0 = (1 / self.T) * np.sum(((self.dxy[:, 0] / (2 * self.dt)) * np.diff(self.t ** 2)) + self.xi * self.dt)
        self.C0 = (1 / self.T) * np.sum(((self.dxy[:, 1] / (2 * self.dt)) * np.diff(self.t ** 2)) + self.delta * self.dt)
        
        # Cubic interpolation things
        tck, u = interpolate.splprep([self.contour[:,0], self.contour[:,1]], s=0, per=True)
        self.xi, self.yi = interpolate.splev(np.linspace(0, 1, self.n), tck)
        
    def gen_elliptic(self):
        
        # Initialize data arrays
        self.t = np.linspace(0, np.pi, self.n)
        self.xt = np.ones((self.n,))*(self.A0+self.contour[0,0])
        self.yt = np.ones((self.n,))*(self.C0+self.contour[0,1])
        errors = np.zeros(len(self.contour)-1,)
        
        # Update elliptic fourier function points
        for n in range(self.coeffs.shape[0]):
            self.xt += (self.coeffs[n, 0] * np.cos(2 * (n + 1) * np.pi * self.t)) + \
                       (self.coeffs[n, 1] * np.sin(2 * (n + 1) * np.pi * self.t))
                  
            self.yt += (self.coeffs[n, 2] * np.cos(2 * (n + 1) * np.pi * self.t)) + \
                       (self.coeffs[n, 3] * np.sin(2 * (n + 1) * np.pi * self.t))
        
        # Calculate errors to nearest interpolation point
        points = np.asarray([self.xt,self.yt]).T
        points = KDTree(points)
        pointsc = np.asarray([self.xi,self.yi]).T
        pointsc = KDTree(pointsc)
        
        if np.all(self.ref)==None: dd,ii = points.query(self.contour[0:-1,:])
        if np.all(self.ref)!=None: dd,ii = points.query(self.ref[0:-1,:])
        if np.all(self.ref)==None: ddd,iii = pointsc.query(self.contour[0:-1,:])
        if np.all(self.ref)!=None: ddd,iii = pointsc.query(self.ref[0:-1,:])
        errors = dd
        errorsc = ddd

        self.error = np.mean(errors)
        self.errorc = np.mean(errorsc)
        
# Main program 
if __name__ == '__main__':
    
    # Initialize config object
    config = conf.configuration()
    config.sim_id = 'postercomp/nav3'
    config.rob_nb = 20
    config.ratio_meas = 1
    config.tend = 15
    config.sens_rate = 500
    
    # Initialize MP plotter object
    order = 20
    plotter = mp_plotter(config,order)
    
    # Save lmap
    if False:
        plotter.save_lmap()
        
    # Save ellip error
    if True:
        plotter.load_pos_mp()
        plotter.save_ellip_error(12)
        #plotter.save_ellip_error(3)
        #plotter.save_ellip_error(4)
        #plotter.save_ellip_error(5)
        
