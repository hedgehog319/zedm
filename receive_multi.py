import sys
import pyzed.sl as sl
import cv2
import threading
import time
import numpy as np

zed_list = []
img_list = []
threads = []

is_run = True


def grab_run(index):
    global zed_list
    global img_list
    global is_run

    zed = zed_list[index]
    img = img_list[index]

    while is_run:
        err = zed.grab()
        if err != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(img, sl.VIEW.LEFT)


def main():
    global is_run
    
    if (len(sys.argv) > 2):
        ip = sys.argv[1]
        ports = list(map(int, sys.argv[2:]))
    else:
        print('Usage : python3 streaming_receiver.py ip ports[list]')
        exit(1)

    for i in range(len(ports)):
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init.set_from_stream(ip, ports[i])
        print("Opening camera")

        cam = sl.Camera()
        status = cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            cam.close()
            continue

        zed_list.append(cam)
        img_list.append(sl.Mat())

    print("Launch threads")
    for i in range(len(zed_list)):
        if zed_list[i].is_opened():
            threads.append(
                threading.Thread(
                    target=grab_run,
                    args=(i,)
                )
            )
            threads[i].start()

    # Display camera images
    key = ''
    while key != 113:  # for 'q' key
        # for i in range(len(zed_list)):
        #     if zed_list[i].is_opened():
        #         cv2.imshow(str(i), img_list[i].get_data())
        #         # x = round(depth_list[i].get_width() / 2)
        #         # y = round(depth_list[i].get_height() / 2)
        #         # err, depth_value = depth_list[i].get_value(x, y)
        #         # if np.isfinite(depth_value):
        #         # print("{} depth at center: {}MM".format(
        #         # cam_names[i], round(depth_value)))
        #         # last_ts_list[i] = timestamp_list[i]
        img = np.concatenate(
            (
                img_list[0].get_data(),
                img_list[1].get_data()
            ),
            axis=1)
        cv2.imshow("Cameras", img)
        key = cv2.waitKey(10)
    cv2.destroyAllWindows()

    is_run = False
    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main()
