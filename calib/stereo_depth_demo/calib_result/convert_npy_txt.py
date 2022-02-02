import numpy as np
import glob

files = glob.glob("./*.npy")

for fname in files:
    file = np.load(fname)
    np.savetxt(fname + ".txt", file)
