import numpy as np
import glob
import os

os.makedirs("./calib_results_txt", exist_ok=True)

files = glob.glob("*.npy")
for fname in files:
    file = np.load(fname)
    noext_fname = os.path.splitext(fname)
    np.savetxt("./calib_results_txt/" + noext_fname[0] + ".txt", file)
