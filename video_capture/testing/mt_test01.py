import os
import time
from datetime import datetime
import arducam_mipicamera as arducam
import cv2
import numpy as np
from collections import deque
from multiprocessing.pool import ThreadPool


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


def save_frame(frame, counter):
    filename = './' + sequence_name + '/' + str(counter) + '.jpg'
    cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return frame

sequence_name = "multithread_test01"
os.makedirs(sequence_name, exist_ok=True)

if __name__ == '__main__':
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
    # Setup.
    thread_num = cv2.getNumberOfCPUs()
    pool = ThreadPool(processes=thread_num)
    pending_task = deque()
    counter = 10000
    ts = []
    timestamp = 0

    while True:
        ts.append(timestamp)
        print(counter, timestamp)
        t1 = datetime.now()

        # Consume the queue.
        while len(pending_task) > 0 and pending_task[0].ready():
            res = pending_task.popleft().get()
            res = cv2.imshow('Video Input', res)

        # Populate the queue.
        if len(pending_task) < thread_num:
            frame = get_frame(camera) 
            task = pool.apply_async(save_frame, (frame.copy(), counter,))
            pending_task.append(task)

        # Show preview.
        if cv2.waitKey(1) == 27:
            break

        t2 = datetime.now()
        delta_t = (t2 - t1).total_seconds()
        counter = counter + 1
        timestamp = timestamp + delta_t

cv2.destroyAllWindows()

# # Initialize the camera
# 
# counter = 10000
# ts = []
# timestamp = 0
# print ("Starting video.")
# while True:
#     ts.append(timestamp)
#     print(counter, timestamp)
#     t1 = datetime.now()
#     frame = get_frame(camera)
#     filename = './' + sequence_name + '/' + str(counter) + '.jpg'
#     cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
#     t2 = datetime.now()
#     delta_t = (t2 - t1).total_seconds()
#     counter = counter + 1
#     timestamp = timestamp + delta_t
#     cv2.imshow("Window", frame)
#     key = (cv2.waitKey(1) & 0xFF)
#     if key == 27:
#         break
# print ("Video finished")
# 
# f = open('./' + sequence_name + '/' + 'times.txt', "w")
# for time in ts:
#     f.write(str(time) + "\n")
# f.close()
