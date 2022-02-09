import cv2
from collections import deque
from multiprocessing.pool import ThreadPool

video = "/home/lorenz/Downloads/ride.MOV"
sequence_name = "test"
counter = 0
cap = cv2.VideoCapture(video)
print(cv2.getNumberOfCPUs())

print ("Starting loading.")
while True:
    ret, frame = cap.read()
    cv2.imshow("s", frame)
    if cv2.waitKey(1) == 27:
        break
    # filename = './' + sequence_name + '/' + str(counter) + '.jpg'
    # cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
print ("Video finished")


def save_frame(frame):
    filename = './' + sequence_name + '/' + str(counter) + '.jpg'
    cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    counter++

if __name__ == '__main__':
    # Setup.
    cap = cv.VideoCapture(VIDEO_SOURCE)
    thread_num = cv.getNumberOfCPUs()
    pool = ThreadPool(processes=thread_num)
    pending_task = deque()

    while True:
        # Consume the queue.
        while len(pending_task) > 0 and pending_task[0].ready():
            res = pending_task.popleft().get()
            cv.imshow('threaded video', res)

        # Populate the queue.
        if len(pending_task) < thread_num:
            frame_got, frame = cap.read()
            if frame_got:
                task = pool.apply_async(process_frame, (frame.copy(),))
                pending_task.append(task)

        # Show preview.
        if cv.waitKey(1) == 27 or not frame_got:
            break

cv.destroyAllWindows()
