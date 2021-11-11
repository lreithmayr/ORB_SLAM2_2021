import cv2 as cv

# Reduced: 500 images of KITTI 00
# Full: Full Sequence of KITTI 00
reduced = 1

if reduced == 1:
    map_name = "grid_map_red.pgm"
else:
    map_name = "grid_map_full.pgm"

gmap = cv.imread("./maps/{:s}".format(map_name), cv.IMREAD_UNCHANGED)

window = cv.namedWindow("GridMap", cv.WINDOW_NORMAL)
cv.resizeWindow(window, 800, 600)

cv.imshow(window, gmap)
if cv.waitKey(0) == 27:
    cv.destroyAllWindows()
