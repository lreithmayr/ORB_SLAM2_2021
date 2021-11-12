import cv2 as cv
import Tkinter as tk
import Tix
from PIL import Image, ImageTk
from tkscrolledframe import ScrolledFrame


# Reduced: 500 images of KITTI 00
# Full: Full Sequence of KITTI 00
reduced = 1

if reduced == 1:
    map_name = "grid_map_red.pgm"
else:
    map_name = "grid_map_full.pgm"

gmap = cv.imread("./maps/{:s}".format(map_name), cv.IMREAD_UNCHANGED)

window = tk.Tk()

height, width = gmap.shape
print height, width

sf = ScrolledFrame(window, width=800, height=600)
sf.pack(side="top", expand=1, fill="both")

# Bind the arrow keys and scroll wheel
sf.bind_arrow_keys(window)
sf.bind_scroll_wheel(window)

# Create a frame within the ScrolledFrame
inner_frame = sf.display_widget(tk.Frame)

canvas = tk.Canvas(inner_frame, width=width, height=height)
canvas.pack()

photo = ImageTk.PhotoImage(image=Image.fromarray(gmap))

# Add a PhotoImage to the Canvas
canvas.create_image(0, 0, image=photo, anchor=tk.NW)

# Run the window loop
window.mainloop()

