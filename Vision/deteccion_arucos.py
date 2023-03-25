import cv2
import cv2.aruco as aruco
from vision_utils import *

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

parameters = cv2.aruco.DetectorParameters_create()

cap = cv2.VideoCapture(0)

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    print(ids)
    # Draw detected markers on frame
    frame_with_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Show output frame
    frame_with_markers = cv2.resize(frame_with_markers, (0, 0), fx = 0.4, fy = 0.4)
    cv2.imshow("Output", frame_with_markers)

    # Exit if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
