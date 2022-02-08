import cv2 as cv
import numpy as np
import glob

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
checkerboard_size = (6, 9)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane

images = glob.glob('./pairs/*.png')
for fname in images:
    if fname is None:
        print("No files to process.")
        break

    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, checkerboard_size, None)

    # If found, add object points, image points (after refining them)
    if ret:
        print("Corners found.")
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(gray, checkerboard_size, corners2, ret)
        cv.imshow('img', gray)
        if cv.waitKey(9999999) == 110:
            continue
    else:
        print("No corners found.")
        cv.imshow("img",img)
        if cv.waitKey(999999) == 110:
            continue

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
np.savetxt("camera_matrix.txt", mtx)
np.savetxt("distance_params.txt", dist)
