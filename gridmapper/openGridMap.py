import cv2 as cv
import Tkinter as tk
import PIL.Image, PIL.ImageTk

# Reduced: 500 images of KITTI 00
# Full: Full Sequence of KITTI 00
reduced = 1

if reduced == 1:
    map_name = "grid_map_red.pgm"
else:
    map_name = "grid_map_full.pgm"

gmap = cv.imread("./maps/{:s}".format(map_name), cv.IMREAD_UNCHANGED)

window = tk.Tk()
window.title("Grid Map")

height, width = gmap.shape
print height, width
canvas = tk.Canvas(window, width=width, height=height)
canvas.pack()

photo = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(gmap))

# Add a PhotoImage to the Canvas
canvas.create_image(0, 0, image=photo, anchor=tk.NW)

# Run the window loop
window.mainloop()
