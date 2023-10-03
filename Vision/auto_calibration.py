#use opencv and a given image to find the color range of a given color
#select a square in the image and get the color range
#select the square by clicking the top left and bottom right corners of the square

import cv2
import numpy as np
import time
from mss import mss
from PIL import Image

# A required callback method that goes into the trackbar function.
def nothing(x):
    pass

# Screen capture
sct = mss()
w, h = 700, 600
monitor = {'top': 0, 'left': 740, 'width': w, 'height': h}
img = Image.frombytes('RGB', (w,h), sct.grab(monitor).rgb)
img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
img_rectangle = img.copy()

color_input = input('Color: ')
color_selected = (0,0,0)
if color_input == 'red':
    color_selected = (0,0,255)
elif color_input == 'green':
    color_selected = (0,255,0)
elif color_input == 'blue':
    color_selected = (255,0,0)
elif color_input == 'yellow':
    color_selected = (0,255,255)


# Cambiar a video capture para calibrar #
#img = cv2.imread('auto_calib_img/testr1.png')
#img_rectangle = img.copy()
#cap.set(3,1280)
#cap.set(4,720)

drawing = False # true if mouse is pressed
selected = False
ix,iy = -1,-1
ox,oy = -1,-1

lower_range = np.array([255,255,255])
upper_range = np.array([0,0,0])
lower_range2 = np.array([255,255,255])
upper_range2 = np.array([0,0,0])

def draw_rectangle(event,x,y,flags,param):
    global ix, iy, ox, oy, drawing, img_rectangle, img, selected
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        selected = True
        ox,oy = x,y

cv2.namedWindow('Color selection')
cv2.setMouseCallback('Color selection', draw_rectangle)

while True:
    img = Image.frombytes('RGB', (w,h), sct.grab(monitor).rgb)
    img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    if not drawing:
        img_rectangle = img.copy()
        cv2.rectangle(img_rectangle,(ix,iy),(ox,oy),color_selected,2)

    # Convert the BGR image to HSV image.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #Get the values of lower an upper HSV range for the selected square
    if selected:
        lower_range_current = np.array([np.min(hsv[iy:oy,ix:ox,0]), np.min(hsv[iy:oy,ix:ox,1]), np.min(hsv[iy:oy,ix:ox,2])])
        upper_range_current = np.array([np.max(hsv[iy:oy,ix:ox,0]), np.max(hsv[iy:oy,ix:ox,1]), np.max(hsv[iy:oy,ix:ox,2])])

        if color_input == 'red':
            # Generate two lower and upper ranges for red, one from zero to the maximum hue not bigger than 10, and the other from the minimum hue not smaller than 170 to 179
            lower_range_current = np.array([0, np.min(hsv[iy:oy,ix:ox,1]), np.min(hsv[iy:oy,ix:ox,2])])
            upper_range_current = np.array([0, np.max(hsv[iy:oy,ix:ox,1]), np.max(hsv[iy:oy,ix:ox,2])])
            lower_range_current2 = np.array([179, np.min(hsv[iy:oy,ix:ox,1]), np.min(hsv[iy:oy,ix:ox,2])])
            upper_range_current2 = np.array([179, np.max(hsv[iy:oy,ix:ox,1]), np.max(hsv[iy:oy,ix:ox,2])])
            for i in range(iy,oy):
                for j in range(ix,ox):
                    if hsv[i,j,0] > 0 and hsv[i,j,0] < 15:
                        upper_range_current[0] = max(upper_range_current[0], hsv[i,j,0])
                    elif hsv[i,j,0] > 165 and hsv[i,j,0] < 179:
                        lower_range_current2[0] = min(lower_range_current2[0], hsv[i,j,0])
            for i in range(3):
                lower_range2[i] = min(lower_range_current2[i], lower_range2[i])
                upper_range2[i] = max(upper_range_current2[i], upper_range2[i])


        selected = False

        #make range arrays equal to the minimum and maximum values, comparing each value
        for i in range(3):
            lower_range[i] = min(lower_range_current[i], lower_range[i])
            upper_range[i] = max(upper_range_current[i], upper_range[i])

        #print arrays to console comma separated and between brackets
        print('lower'+color_input.capitalize()+' = np.array(['+str(lower_range[0])+','+str(lower_range[1])+','+str(lower_range[2])+'], np.uint8)')
        print('upper'+color_input.capitalize()+' = np.array(['+str(upper_range[0])+','+str(upper_range[1])+','+str(upper_range[2])+'], np.uint8)')
        if color_input == 'red':
            print('lower'+color_input.capitalize()+'2 = np.array(['+str(lower_range2[0])+','+str(lower_range2[1])+','+str(lower_range2[2])+'], np.uint8)')
            print('upper'+color_input.capitalize()+'2 = np.array(['+str(upper_range2[0])+','+str(upper_range2[1])+','+str(upper_range2[2])+'], np.uint8)')


    # Filter the image and get the binary mask, where white represents
    # your target color
    mask = cv2.inRange(hsv, lower_range, upper_range)
    if color_input == 'red':
        mask2 = cv2.inRange(hsv, lower_range2, upper_range2)
        mask = cv2.bitwise_or(mask, mask2)

    #Detect the square drawn by the user
    cv2.imshow('Raw',img)
    cv2.imshow('Color selection', img_rectangle)
    cv2.imshow('Mask',mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
