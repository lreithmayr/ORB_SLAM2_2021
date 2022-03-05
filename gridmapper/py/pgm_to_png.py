import cv2 as cv
import os
import glob

print(os.getcwd())

files = glob.glob("maps/gridmaps/*.pgm")
for fpath in files:
    gmap = cv.imread(fpath, cv.IMREAD_UNCHANGED)
    noext_fpath = os.path.splitext(fpath)
    print(noext_fpath)
    fname = os.path.normpath(noext_fpath[0]).split(os.path.sep)[-1]
    print(fname)
    cv.imwrite("maps/gridmap_images/" + fname + ".png", gmap)
