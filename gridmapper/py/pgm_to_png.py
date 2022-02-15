import cv2 as cv
import os
import glob

files = glob.glob("./maps/*.pgm")
for fpath in files:
    gmap = cv.imread(fpath, cv.IMREAD_UNCHANGED)
    noext_fpath = os.path.splitext(fpath)
    fname = os.path.normpath(noext_fpath[0]).split(os.path.sep)[1]
    cv.imwrite("./maps/gridmap_images/" + fname + ".png", gmap)
