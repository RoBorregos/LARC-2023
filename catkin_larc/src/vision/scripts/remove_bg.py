#make me a code in opencv to remove the background of all the images on a path and save it again, the background is green

import cv2
import numpy as np
import os
import pathlib
import imutils

path = "/home/jabv/Desktop/dataset"
path2 = "/home/jabv/Desktop/Dataset_final_yolov8_png_no_bg"


def remove_green_background(image):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    

    # define range of cube color in HSV
    lower_green = np.array([0, 0, 0])
    upper_green = np.array([125, 90, 255])

    # define range of green color in HSV

    # lower_green = np.array([0, 0, 0])
    # upper_green = np.array([125, 90, 255])

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)

    #reverse mask to get green letters
    # mask = cv2.bitwise_not(mask)
    
    cv2.imshow('mask', mask)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(image, image, mask=mask)

    res = img.copy()
    res = cv2.cvtColor(res, cv2.COLOR_BGR2BGRA)
    res[:, :, 3] = mask

    return res

#use remove_green_background function to remove green background from all images in path
for filename in os.listdir(path):
    if filename.endswith(".png"):
        img = cv2.imread(path + "/" + filename)
        print(filename + " loaded")
        img = remove_green_background(img)
        cv2.imwrite(path2 + "/" + filename, img)
        print(filename + " done")



# lala = imutils.resize(cv2.imread("/home/jabv/Desktop/Dataset_final_yolov8_png/IMG_0600.png"), width=640)
# img= remove_green_background(lala)
# cv2.imshow('image', img)
# cv2.imshow('image2', lala)