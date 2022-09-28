import numpy as np
from tqdm import tqdm
import cv2
import glob


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
# createVideo2(tmp + '/', tmp + '_VideoImages/', tmp)
def createVideo2(saveLoc, imgLoc, videoName):
    flnm = glob.glob(imgLoc + '*.jpg')[0]
    img_tmp = cv2.imread(flnm)
    height, width, layers = img_tmp.shape
    size = (width, height)

    print('\nCreating video...')
    #out = cv2.VideoWriter(saveLoc + videoName + '.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, size)
    out = cv2.VideoWriter(saveLoc + videoName + '.mp4', cv2.VideoWriter_fourcc('m','p','4','v'), 40, size)
    for filename in glob.glob(imgLoc + '*.jpg'):
        img = cv2.imread(filename)
        out.write(img)
    out.release()

    #rmtree(imgLoc)
    print('Video Creation Complete')


def createVideo(saveLoc, imgLoc, videoName, imgShape, fps):
    print('\nCreating video...')
    import glob  # For creating videos
    import cv2  # For creating videos
    from shutil import rmtree

    # out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, imgShape)
    out = cv2.VideoWriter(saveLoc + videoName + '.mp4', cv2.VideoWriter_fourcc('M', 'P', '4', 'V'), fps, (int(imgShape[0]), int(imgShape[1])))
    for file in tqdm(glob.glob(imgLoc + '*.jpg')):
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
    