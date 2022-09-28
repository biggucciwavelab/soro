import numpy as np
from tqdm import tqdm
import cv2
import glob
import matplotlib.pyplot as plt
from elliptic2 import elliptic_fourier_descriptors, reconstruct_contour
from MPC_Reference_Generators import ImageCurvature
from time import time
from scipy.optimize import minimize

def calc_JAMoEBA_Radius(skinRadius, skinRatio, botRadius, numBots):
    """
    Inputs:
        - skinRadius (float): The radius of skin particles on system
        - skinRatio (int): Ratio of number of skin particles per bot
        - botRadius (float): The radius of bot particles on system
        - numBots (int): Number of bots in the system

    Returns:
        - R (float): Radius of the system given parameters
    """
    startDistance = skinRadius # The start distance between bots
    arcLength = 2*botRadius+skinRatio*(2*skinRadius)+(skinRatio+1)*startDistance
    theta = 2*np.pi/numBots
    R = arcLength/theta #**
    return R

class Convert:
    def __init__(self, conversion_ratio=100):
        """
        Parameters
        ----------
        conversion_ratio : float
            The conversion ratio between pixels and meters in the form (pixels/meter).
        """
        self.ratio = conversion_ratio
        
    def Pixels2Meters(self, num_pixels):
        return (num_pixels*(1/self.ratio))
    
    def Meters2Pixels(self, meters):
        return (meters*self.ratio)
    
    def SpringK2Pixels(self, springK):
        return (springK*(1/self.ratio))
    
    def Pixels2SpringK(self, springKPixels):
        return (springKPixels*self.ratio)

# tmp = "Test_0004"
# createVideo2(tmp + '/', tmp + '_VideoImages/', tmp, fps=60)
def createVideo2(saveLoc, imgLoc, videoName, fps=40):
    flnm = glob.glob(imgLoc + '*.jpg')[0]
    img_tmp = cv2.imread(flnm)
    height, width, layers = img_tmp.shape
    size = (width, height)

    print('\nCreating video...')
    #out = cv2.VideoWriter(saveLoc + videoName + '.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, size)
    out = cv2.VideoWriter(saveLoc + videoName + '.mp4', cv2.VideoWriter_fourcc('m','p','4','v'), fps, size)
    for i, filename in enumerate(glob.glob(imgLoc + '*.jpg')):

        if i%4 == 0:
            img = cv2.imread(filename)
            out.write(img)
    out.release()

    #rmtree(imgLoc)
    print('Video Creation Complete')

def createVideo(saveLoc, imgLoc, videoName, imgShape, fps):
    print('\nCreating video...')
    import glob # For creating videos
    import cv2 # For creating videos
    from shutil import rmtree

    #out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, imgShape)
    out = cv2.VideoWriter(saveLoc + videoName + '.mp4', cv2.VideoWriter_fourcc('M','P','4','V'), fps, imgShape)
    for file in tqdm(glob.glob(imgLoc+'*.jpg')):
        img = cv2.imread(file)
        out.write(img)
    out.release()
    
    rmtree(imgLoc)
    print('Video Creation Complete')


def save_runtime(saveloc,  file_name, runtime):
    from datetime import datetime, timedelta
    """
    Inputs:
        - saveloc (str): Path (NOT file name!) where the runtime will be stored
        - file_name (str): Name to save the information under
        - runtime (float): Runtime (in seconds) of some program
    Given a runtime (in seconds), 
    will save the amount of time it took to run some program at location 'saveloc'
    """
    sec = timedelta(seconds=runtime)
    d = datetime(1,1,1) + sec

    with open(saveloc + file_name +'.txt','w') as f:
        f.write('DAYS:HOURS:MIN:SEC\n')
        f.write("%d:%d:%d:%d" % (d.day-1, d.hour, d.minute, d.second))


def animatePattern(file_name, dt=25/200, order=10, ref_figure=None):


    # Load data for x and y
    x = np.loadtxt(file_name + '/X_data.csv', delimiter=',')
    y = np.loadtxt(file_name + '/Y_data.csv', delimiter=',')

    # Create figure
    plt.figure()

    # Check if we have a figure to compare
    if type(ref_figure) != None:
        ref = ImageCurvature([0, 0], 0, np.array([1, 1]) * .6, [1500, 1500], 300)
        ref.generateRef(ref_figure, 1000, order)
        points = reconstruct_contour(ref.coefs, num_points=150)
        plt.plot(points[:, 0], points[:, 1], '--r')

    # Create lines for descriptor and bot positions
    line,  = plt.plot([], [])
    liner, = plt.plot([], [], '*')
    plt.xlim([-2, 2])
    plt.ylim([-2, 2])

    # Iterate for each timestep
    t = 0
    X_last = np.zeros(5)
    for xi, yi in zip(x, y):
        # Update timestep
        dt = xi[0] - t
        t = xi[0]
        t0 = time()

        # Calculate descriptors
        coefs = elliptic_fourier_descriptors(np.vstack((xi[1:], yi[1:])).T, order, normalize=True)
        points = reconstruct_contour(coefs, num_points=150)

        # Plot descriptors
        line.set_xdata(points[:, 0])
        line.set_ydata(points[:, 1])

        def scale_distance(X):
            # Plot bot positions:
            xnew = ((xi[1:] * X[2] + yi[1:] * X[3]) + X[0])
            ynew = ((-xi[1:] * X[3] + yi[1:] * X[2]) + X[1])
            idx = (np.linspace(0, 150-1, len(xi[1:]), dtype=np.int) + int(X[4] * 150)) % 150
            tmp = np.sum((xnew - points[idx, 0]) ** 2)
            tmp += np.sum((ynew - points[idx, 1]) ** 2)

            return tmp

        opt = minimize(scale_distance, X_last)

        X_last = opt.x

        liner.set_xdata(((xi[1:] * X_last[2] + yi[1:] * X_last[3]) + X_last[0]))
        liner.set_ydata(((-xi[1:] * X_last[3] + yi[1:] * X_last[2]) + X_last[1]))

        # Update plot
        plt.draw()

        # Let plot pause
        dt0 = time() - t0
        plt.pause(max(dt - dt0, 0.001))