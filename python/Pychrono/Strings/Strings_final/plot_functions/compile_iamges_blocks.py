# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 14:40:11 2021

@author: dmulr
"""
import cv2
import numpy as np




IM=[]
IMH=[]
entries=[119,239,359,479,639,759,863]

file='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'
name='04_04_2021_09_22_00'
n=4
m=3
for i in entries:
    frame=str("frame%04d" % i)
    im = cv2.imread('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+name+'_video/'+frame+'.jpg')

    IM.append(im)

im_h = cv2.hconcat([IM[0],IM[1],IM[2],IM[3],IM[4],IM[5],IM[6]])
cv2.imwrite('latters.jpg', im_h)
cv2.imwrite('latters.bmp', im_h)
# for i in range(m):
#     im_h = cv2.hconcat(IM[m*i:m*i+m])
#     IMH.append(im_h)


    
# im_v=cv2.vconcat([IMH[0],IMH[1],IMH[2]])
# cv2.imwrite('morph1.jpg', im_v)
# im1 = cv2.imread('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/08_03_2021_13_44_12_video/frame0000.jpg')
# im2 = cv2.imread('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/08_03_2021_13_44_12_video/frame0050.jpg')
# im3 = cv2.imread('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/08_03_2021_13_44_12_video/frame0100.jpg')
# im4 = cv2.imread('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/08_03_2021_13_44_12_video/frame0150.jpg')
# im_h = cv2.hconcat([im1, im2, im3,im4])
# cv2.imwrite('trial1.jpg', im_h)

# im5 = cv2.imread('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/08_03_2021_13_44_12_video/frame0200.jpg')
# im6 = cv2.imread('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/08_03_2021_13_44_12_video/frame0250.jpg')
# im7 = cv2.imread('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/08_03_2021_13_44_12_video/frame0300.jpg')
# im8 = cv2.imread('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/08_03_2021_13_44_12_video/frame0359.jpg')
# im_h = cv2.hconcat([im5, im6, im7,im8])
# cv2.imwrite('trial2.jpg', im_h)

# im1=cv2.imread('trial1.jpg')
# im2=cv2.imread('trial2.jpg')
# im_v = cv2.vconcat([im1, im2])

# cv2.imwrite('trial3.jpg', im_v)
