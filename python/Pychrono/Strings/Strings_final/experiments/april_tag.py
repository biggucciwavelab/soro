# -*- coding: utf-8 -*-
"""
Created on Thu Jul  8 16:51:07 2021

@author: dmulr
"""

import cv2
import numpy as np
from numpy import arange
import urllib.request


def padded_str(value, length, character='0'):
    init_string = str(value)

    return (length - len(init_string)) * character + init_string

# METHOD #1: OpenCV, NumPy, and urllib
def url_to_image(url):
    # download the image, convert it to a NumPy array, and then read
    # it into OpenCV format
    resp = urllib.request.urlopen(url)
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)

    # return the image
    return image

def addMargin(img, width):
    if width > 1:
        img[0:width, :, :] = 0
        img[-width:, :, :] = 0
        img[:, 0:width, :] = 0
        img[:, -width:, :] = 0
    else:
        img[0, :, :] = 0
        img[-1, :, :] = 0
        img[:, 0, :] = 0
        img[:, -1, :] = 0

    return img

source = 'https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/'
store_folder = 'F:/Soro_chrono/python/Pychrono/Strings/Strings_final/experiments/Apriltags2/'

# image size in mm
conv = 100 / (1.04 * 25.4)  # px / mm

margin = True
margin_width = 1


size = 54      # in mm
size = int(conv * size)
print(size)

# Which ID's to select
num_min = 1
num_max = 13

# Generate links to source images
ids = arange(num_min, num_max + 1, 1)
requested_tags = [source + 'tag36_11_' + padded_str(i, 5) + '.png' for i in ids]


for url, i  in zip(requested_tags, ids):
    img = url_to_image(url)
    im_large = cv2.resize(img, (size, size), interpolation=cv2.INTER_NEAREST)
    if margin:
        im_large = addMargin(im_large, margin_width)

    print('writting image with id: ' + str(i))
    cv2.imwrite(store_folder + 'tag36_11_' + padded_str(i, 5) + '_' + str(size) + '_mm.png', im_large)

print('Done!')