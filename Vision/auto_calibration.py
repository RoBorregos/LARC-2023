#use opencv and a given image to find the color range of a given color
#select a square in the image and get the color range
#select the square by clicking the top left and bottom right corners of the square

import cv2
import numpy as np
import time

# A required callback method that goes into the trackbar function.
def nothing(x):
    pass

# Initializing the webcam feed.
#cap = cv2.VideoCapture(0)

# Cambiar a video capture para calibrar #
img = cv2.imread('gazebo_color_test1.png')
img_rectangle = img.copy()
#cap.set(3,1280)
#cap.set(4,720)

drawing = False # true if mouse is pressed
selected = False
ix,iy = -1,-1
ox,oy = -1,-1

lower_range = np.array([0,0,0])
upper_range = np.array([0,0,0])

def draw_rectangle(event,x,y,flags,param):
    global ix, iy, ox, oy, drawing, img_rectangle, img, selected
    if event == cv2.EVENT_LBUTTONDOWN:
        img_rectangle = img.copy()
        drawing = True
        ix,iy = x,y
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        selected = True
        ox,oy = x,y
        cv2.rectangle(img_rectangle,(ix,iy),(ox,oy),(255,0,255),2)

cv2.namedWindow('Color selection')
cv2.setMouseCallback('Color selection', draw_rectangle)

while True:
    frame = img
    # Convert the BGR image to HSV image.
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Get the values of lower an upper HSV range for the selected square
    if selected:
        lower_range = np.array([np.min(hsv[iy:oy,ix:ox,0]), np.min(hsv[iy:oy,ix:ox,1]), np.min(hsv[iy:oy,ix:ox,2])])
        upper_range = np.array([np.max(hsv[iy:oy,ix:ox,0]), np.max(hsv[iy:oy,ix:ox,1]), np.max(hsv[iy:oy,ix:ox,2])])
        selected = False
        #print arrays to console comma separated and between brackets
        print('np.array(['+str(lower_range[0])+','+str(lower_range[1])+','+str(lower_range[2])+'], np.uint8)')
        print('np.array(['+str(upper_range[0])+','+str(upper_range[1])+','+str(upper_range[2])+'], np.uint8)')


    # Filter the image and get the binary mask, where white represents
    # your target color
    mask = cv2.inRange(hsv, lower_range, upper_range)

    #Detect the square drawn by the user
    cv2.imshow('Raw',frame)
    cv2.imshow('Color selection', img_rectangle)
    cv2.imshow('Mask',mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
