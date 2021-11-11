import cv2

reduced = 0

if reduced == 1:
    map_name = "grid_map_red.pgm"
else:
    map_name = "grid_map_full.pgm"

gmap = cv2.imread("./maps/{:s}".format(map_name), cv2.IMREAD_UNCHANGED)
cv2.imshow("Grid Map", gmap)
cv2.waitKey(0)
