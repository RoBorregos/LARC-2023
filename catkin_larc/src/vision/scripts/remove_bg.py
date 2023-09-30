#make me a code in opencv to remove the background of all the images on a path and save it again, the background is green

import cv2
import numpy as np
import os
import pathlib
import imutils

path = "/home/jabv/Desktop/dataset"
path2 = "/home/jabv/Desktop/Dataset_final_yolov8_png_no_bg"

def remove_background_biggest_contour(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection
    canny = cv2.Canny(gray, 50, 150)

    # Dilate the edges to close gaps
    canny = cv2.dilate(canny, None, iterations=1)
    
    #cv2.imshow('canny', canny) 

    # Find contours in the Canny image
    contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Find the largest contour by area
        largest_contour = max(contours, key=cv2.contourArea)

        # Create an empty mask
        mask = np.zeros_like(gray)

        # Draw the largest contour on the mask
        cv2.drawContours(mask, [largest_contour], 0, (255), thickness=cv2.FILLED)

        # Use the mask to extract the region of interest (ROI) from the original image
        result = cv2.bitwise_and(image, image, mask=mask)
        #result = cv2.cvtColor(result, cv2.COLOR_BGR2BGRA)
        #result[:, :, 3] = mask


        return result

    else:
        # If no contours are found, return the original image
        return image

def remove_green_background(image):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    

    # define range of cube color in HSV
    lower_green = np.array([0, 0, 0])
    upper_green = np.array([179, 130, 255])

    # define range of green color in HSV

    # lower_green = np.array([0, 0, 0])
    # upper_green = np.array([125, 90, 255])

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)

    #reverse mask to get green letters
    mask = cv2.bitwise_not(mask)
    
    #cv2.imshow('mask', mask)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(image, image, mask=mask)

    #res = image.copy()  
    res = np.zeros_like(image, dtype=np.uint8)
    res[mask != 255] = image[mask != 255]
    #res = cv2.cvtColor(res, cv2.COLOR_BGR2BGRA)
    #res[:, :, 3] = mask

    return res

def find_extreme_points(img):
    # Carga la imagen en escala de grises
    image = img #cv2.imread(image_path, 0)
    
    image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Encuentra los contornos en la imagen
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Asegúrate de que se encontraron contornos
    if len(contours) == 0:
        print("No se encontraron contornos en la imagen.")
        return None

    # Encuentra el contorno más grande (puede haber varios contornos)
    largest_contour = max(contours, key=cv2.contourArea)

    # Encuentra los puntos extremos del contorno más grande
    leftmost = tuple(largest_contour[largest_contour[:, :, 0].argmin()][0])
    rightmost = tuple(largest_contour[largest_contour[:, :, 0].argmax()][0])
    topmost = tuple(largest_contour[largest_contour[:, :, 1].argmin()][0])
    bottommost = tuple(largest_contour[largest_contour[:, :, 1].argmax()][0])

    return topmost, rightmost, bottommost, leftmost


#use remove_green_background function to remove green background from all images in path
# for filename in os.listdir(path):
#     if filename.endswith(".png"):
#         img = cv2.imread(path + "/" + filename)
#         print(filename + " loaded")
#         img = remove_green_background(img)
#         cv2.imwrite(path2 + "/" + filename, img)
#         print(filename + " done")



lala = imutils.resize(cv2.imread("/home/jabv/Desktop/DS_estantes_png_resize/IMG_3177.png"), width=640)
img= remove_background_biggest_contour(lala)
result = remove_green_background(img)

cv2.imshow('result', result)
cv2.imshow('image', img)
cv2.imshow('image2', lala)

#inserts extreme points in image
topmost, rightmost, bottommost, leftmost = find_extreme_points(result)
cv2.circle(result, topmost, 8, (0, 255, 0), -1)
cv2.circle(result, rightmost, 8, (0, 255, 0), -1)
cv2.circle(result, bottommost, 8, (0, 255, 0), -1)
cv2.circle(result, leftmost, 8, (0, 255, 0), -1)

cv2.imshow('result2', result)

cv2.waitKey(0)
cv2.destroyAllWindows()