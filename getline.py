import cv2
import numpy as np
import imutils

def get_countourn(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection
    edges = cv2.Canny(gray, threshold1=50, threshold2=150)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create an empty image to draw the detected lines
    result = np.zeros_like(image)

    # Define the lower and upper bounds for black color in BGR format
    lower_black = np.array([0, 0, 0], dtype=np.uint8)
    upper_black = np.array([30, 30, 30], dtype=np.uint8)

    # Iterate through the contours and filter black lines
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        roi = image[y:y+h, x:x+w]
        
        # Calculate the mean color of the region of interest (ROI)
        mean_color = np.mean(roi, axis=(0, 1))
        
        # Check if the mean color falls within the black color range
        if np.all(mean_color >= lower_black) and np.all(mean_color <= upper_black):
            cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)  # Draw the contour in green
    return result

# Load the image
image = imutils.resize(cv2.imread('/home/jabv/Desktop/LARC-2023/Vision/lineas1.jpeg'),width=640)

resp = get_countourn(image)
cv2.imshow('image', resp)


cv2.waitKey(0)
cv2.destroyAllWindows()
