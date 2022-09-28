"""
Accompanying script for this simulation. 
Saving the video we just made
"""
from venv import create
from tqdm import tqdm

width = 1440
height = 1080

def createVideo(saveLoc, imgLoc, videoName, imgShape):
    print('\nCreating video...')
    import glob # For creating videos
    import cv2 # For creating videos
    from shutil import rmtree

    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, imgShape)
    for file in tqdm(glob.glob(imgLoc+'*.jpg')):
        img = cv2.imread(file)
        out.write(img)
    out.release
    
    rmtree(imgLoc)
    print('Video Creation Complete')

# Save the video
imgLoc = "BallsInBoxPics/"
saveLoc = ''
createVideo(saveLoc,imgLoc,'Balls In Box Next',(width,height))