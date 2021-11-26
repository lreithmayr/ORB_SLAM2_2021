import cv2 as cv
import os

print os.getcwd()


map_name = "gridmap_py_kitti_stereo07.pgm"
gmap = cv.imread("./maps/{:s}".format(map_name), cv.IMREAD_UNCHANGED)

# gmap = cv.imread("/home/lorenz/Projects/BA/01_Algorithms/ORB_SLAM2_MOD/gridmapper/maps/monoKitti_gridMap_CPP_red.pgm")

while True:
    cv.imshow("Grid Map", gmap)
    if cv.waitKey(0) == 27:
        break

cv.destroyAllWindows()
