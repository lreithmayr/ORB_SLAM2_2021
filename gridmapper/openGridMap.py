import cv2 as cv
import Tkinter as tk
import Tix
from PIL import Image, ImageTk
from tkscrolledframe import ScrolledFrame


# seq_name = 'stKi'
seq_name = 'monoKi'

reduced = 0
if reduced == 1:
    map_name = '{:s}_grid_map_red.pgm'.format(seq_name)
else:
    map_name = '{:s}_grid_map_full.pgm'.format(seq_name)

gmap = cv.imread("./maps/{:s}".format(map_name), cv.IMREAD_UNCHANGED)

while True:
    cv.imshow("test", gmap)
    if cv.waitKey(0) == 27:
        break

cv.destroyAllWindows()
