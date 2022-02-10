"""

This script receives an input feed from the Arducam stereo camera and saves a frame at a rate of 10 frames per second to a folder.

The name of the folder can be chosen by altering the variable "sequence_name" at the beginning of the program.

"""

import os
import time
from datetime import datetime
import arducam_mipicamera as arducam
import json
import cv2
import numpy as np

sequence_name = "fps_test"
os.makedirs("./videos/" + sequence_name, exist_ok=True)

def align_down(size, align):
    return (size & ~((align)-1))

def align_up(size, align):
    return align_down(size + align - 1, align)

def get_frame(camera):
    frame = camera.capture(encoding = 'i420')
    fmt = camera.get_format()
    height = int(align_up(fmt['height'], 16))
    width = int(align_up(fmt['width'], 32))
    image = frame.as_array.reshape(int(height * 1.5), width)
    image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
    image = image[:fmt['height'], :fmt['width']]
    return image

# Initialize the camera
camera = arducam.mipi_camera()
print("Open camera...")
camera.init_camera()
camera.set_mode(10)
fmt = camera.get_format()
print("Current mode: {},resolution: {}x{}".format(fmt['mode'], fmt['width'], fmt['height']))
# camera.set_control(0x00980911, 1000)

# Camera settings
cam_width = fmt['width']              # Cam sensor width settings
cam_height = fmt['height']              # Cam sensor height settings
print ("Used camera resolution: "+str(cam_width)+" x "+str(cam_height))

# Buffer for captured image settings
img_width = cam_width 
img_height = cam_height 
print ("Scaled image resolution: "+str(img_width)+" x "+str(img_height))

framecount = 0
counter = 10000
ts = []
timestamp = 0
print ("Starting video.")
while True:
    t1 = datetime.now()
    framecount = framecount + 1
    ts.append(timestamp)
    print("Frame Number | Timestamp: %s" % framecount, timestamp)

    frame = get_frame(camera)
    filename = './videos/' + sequence_name + '/' + str(counter) + '.jpg'
    cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])

    counter = counter + 1

    # fps = framecount / timestamp
    # print("FPS: %s" % int(fps))


    # key = (cv2.waitKey(1) & 0xFF)
    # if key == 27:
    #     break

    # delta_t = (t2 - t1).total_seconds()

    # if framecount > 6:
    #     add_time = 0.1 - delta_t 
    #     time.sleep(add_time)


    cv2.imshow("Window", frame)
    if cv2.waitKey(60) == 27:
        break;

    t2 = datetime.now()
    delta_t = (t2 - t1).total_seconds()
    timestamp = timestamp + delta_t 

print ("Video finished")

f = open('./videos/' + sequence_name + '/' + 'times.txt', "w")
for time in ts:
    f.write(str(time) + "\n")
f.close()
