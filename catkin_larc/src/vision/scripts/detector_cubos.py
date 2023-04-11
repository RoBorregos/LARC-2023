import cv2
import numpy as np

# Read camera input
cap = cv2.VideoCapture(0)

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection to detect the edges
    edges = cv2.Canny(gray, 100, 200)

    # Find all the contours in the image
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over each contour and approximate its shape
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.05*cv2.arcLength(cnt, True), True)

        # Check if the contour is a rectangle
        if len(approx) == 4:
            # Calculate the aspect ratio of the rectangle
            x,y,w,h = cv2.boundingRect(cnt)
            aspect_ratio = float(w)/h

            # Check if the aspect ratio is close to 1:1:1
            if abs(aspect_ratio - 1.0) < 0.1:
                # Draw the rectangle on the image
                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)

    # Display the camera image with the detected cube
    cv2.imshow('Cube Detector', frame)

    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
