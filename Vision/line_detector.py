import cv2
import numpy as np
import math

lowMask = np.array([97,11,4],np.uint8)
upperMask = np.array([179,84,152],np.uint8)

img = cv2.imread("lineas.png")
#crop image to center
img = img[0:, 480:1300]
h,w = img.shape[0:2]
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv,lowMask,upperMask)
#bitwise and mask and original image
res = cv2.bitwise_and(img,img,mask=mask)
cv2.imshow("res",res)
gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
#mask = cv2.erode(mask,None,iterations=1)
gray = cv2.dilate(mask,None,iterations=1)
#gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
_, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
cv2.imshow("thresh",thresh)

kernel = np.ones((5,5), np.uint8)
thresh = cv2.dilate(thresh, kernel, iterations=2)
cv2.imshow("thresh",thresh)

# Detect lines using Hough transform
lines = cv2.HoughLines(thresh, 1, np.pi/20, 700)

existent_pt1 = []
existent_pt2 = []

# Filter out horizontal lines
horizontal_lines = []
for line in lines:
    rho, theta = line[0]
    a = math.cos(theta)
    b = math.sin(theta)
    x0 = a * rho
    y0 = b * rho
    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

    if abs(theta - np.pi/2) < 0.05:
        valid = True
        for ePt1 in existent_pt1:
            if abs(ePt1[0] - pt1[0]) < 50 and abs(ePt1[1] - pt1[1]) < 50:
                valid = False
        for ePt2 in existent_pt2:
            if abs(ePt2[0] - pt2[0]) < 50 and abs(ePt2[1] - pt2[1]) < 50:
                valid = False
        if valid:
            horizontal_lines.append(line)
            existent_pt1.append(pt1)
            existent_pt2.append(pt2)
            cv2.line(img, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

# Count the number of horizontal lines
num_lines = len(horizontal_lines)
print("Number of horizontal lines:", num_lines)
"""edges = cv2.Canny(thresh, 50, 200, None, 3)


def dist(x, y, x1, y1):
    return ((x-x1)**2+(y-y1)**2)**(0.5)


def slope(x, y, x1, y1):
    if y1 != y:
        return ((x1-x)/(y1-y))
    else:
        return 0


fld = cv2.ximgproc.createFastLineDetector()
lines = fld.detect(edges)
no_of_hlines = 0
#result_img = fld.drawSegments(img, lines)
for line in lines:
    x0 = int(round(line[0][0]))
    y0 = int(round(line[0][1]))
    x1 = int(round(line[0][2]))
    y1 = int(round(line[0][3]))
    d = dist(x0, y0, x1, y1)
    if d>150: #You can adjust the distance for precision
        m = (slope(x0, y0, x1, y1))
        if abs(m)<=0.2: #slope for horizontal lines and adjust slope for vertical lines
            no_of_hlines+=1
            cv2.line(img, (x0, y0), (x1, y1), (255, 0, 255), 1, cv2.LINE_AA)
print(no_of_hlines)
"""
cv2.imshow("lines",img)
cv2.waitKey(0)
cv2.destroyAllWindows()