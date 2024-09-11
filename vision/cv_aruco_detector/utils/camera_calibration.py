import cv2
import os
import json
import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_6X6_250  # Dictionary ID
SQUARES_VERTICALLY = 7               # Number of squares vertically
SQUARES_HORIZONTALLY = 5             # Number of squares horizontally
SQUARE_LENGTH = 30                   # Square side length (in pixels)
MARKER_LENGTH = 15                   # ArUco marker side length (in pixels)
MARGIN_PX = 20                       # Margins size (in pixels)

IMG_SIZE = tuple(i * SQUARE_LENGTH + 2 * MARGIN_PX for i in (SQUARES_VERTICALLY, SQUARES_HORIZONTALLY))
OUTPUT_NAME = 'ChArUco_Marker.png'

create_board = False
calibrate_camera = False
view_images = False
view_undistorted = True


def create_and_save_new_board():
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
    size_ratio = SQUARES_HORIZONTALLY / SQUARES_VERTICALLY
    img = cv2.aruco.CharucoBoard.generateImage(board, IMG_SIZE, marginSize=MARGIN_PX)
    cv2.imwrite(OUTPUT_NAME, img)

if create_board:
    create_and_save_new_board()


def get_calibration_parameters(img_dir):
    # Define the aruco dictionary, charuco board and detector
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    
    # Load images from directory
    image_files = [os.path.join(img_dir, f) for f in os.listdir(img_dir) if f.endswith(".png")]
    all_charuco_ids = []
    all_charuco_corners = []

    # Loop over images and extraction of corners
    for image_file in image_files:
        image = cv2.imread(image_file)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if view_images:
            # view the image
            cv2.imshow('image', image)
            cv2.waitKey(0)

        imgSize = image.shape
        
        image_copy = image.copy()
        marker_corners, marker_ids, rejectedCandidates = detector.detectMarkers(image)
        
        if marker_ids is not None and len(marker_ids) > 0: # If at least one marker is detected
            print(len(marker_ids))
            # cv2.aruco.drawDetectedMarkers(image_copy, marker_corners
            # , marker_ids)
            ret, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, board)
            
            if charucoIds is not None and len(charucoCorners) > 3:
                print(len(charucoIds))
                all_charuco_corners.append(charucoCorners)
                all_charuco_ids.append(charucoIds)
    
    # Calibrate camera with extracted information
    result, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, board, imgSize, None, None)
    return mtx, dist


if calibrate_camera:
    SENSOR = 'ov13855'
    LENS = 'default'
    OUTPUT_JSON = 'calibration.json'


    home_directory = os.path.expanduser('~/cv_images')  # Path to the home directory
    mtx, dist = get_calibration_parameters(img_dir=home_directory)
    data = {"sensor": SENSOR, "lens": LENS, "mtx": mtx.tolist(), "dist": dist.tolist()}

    with open(OUTPUT_JSON, 'w') as json_file:
        json.dump(data, json_file, indent=4)

    print(f'Data has been saved to {OUTPUT_JSON}')



def unidstort(img_dir):
    # Define the aruco dictionary, charuco board and detector

    mtx = np.array([[174.88, 0, 239.27],
                          [0, 144.72, 280.49],
                          [0, 0, 1]])
                          
    dist = np.array([0.01228, -0.03170, -0.02041, 0.01481, 0.00599])
    
    # Load images from directory
    image_files = [os.path.join(img_dir, f) for f in os.listdir(img_dir) if f.endswith(".png")]
    all_charuco_ids = []
    all_charuco_corners = []

    # Loop over images and extraction of corners
    for image_file in image_files:
        image = cv2.imread(image_file)


        h,  w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        image = cv2.undistort(image, mtx, dist, None, newcameramtx)

        # view the image
        cv2.imshow('image', image)
        cv2.waitKey(0)

if view_undistorted:
    home_directory = os.path.expanduser('~/cv_images')  # Path to the home directory
    unidstort(home_directory)