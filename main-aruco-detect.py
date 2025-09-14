import cv2
import numpy as np

# Open the default camera
cam = cv2.VideoCapture(0)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

while True:
    ret, frame = cam.read()
    image = frame 

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()

    # Create the ArUco detector
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    # Detect the markers
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        print("CENTER - X distance: ", (frame_width/2) - corners[0][0][0][0])
        #cv2.aruco.drawDetectedMarkers(image, corners, ids)

# Release the capture and writer objects
cam.release()
# cv2.destroyAllWindows()
