#!/usr/bin/env python

import cv2 as cv
import numpy as np
import glob

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6 * 8, 3), np.float32)
objp[:, :2] = np.mgrid[0:6, 0:8].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane

images = glob.glob('images/*.jpg')
for fname in images:
    if fname is None:
        break

    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (6, 8), None)
    cv.drawChessboardCorners(gray, (6, 8), corners, ret)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (6, 8), corners2, ret)
        cv.imshow("foundCorners", gray)
        cv.imshow('img', img)
        cv.waitKey(50)
        print("Corners found.")
    else:
        print("No corners found.")

cv.destroyAllWindows()

# Calibrate camera and print calibration matrix
# https://docs.opencv.org/4.5.3/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Undistortion

img = cv.imread(images[0])
h, w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# Crop the image
x, y, w, h = roi
dst = dst[y:y + h, x:x + w]

# Write camera parameters to files
np.savetxt("calib_matrix.txt", mtx)
np.savetxt("dist_params.txt", dist)
