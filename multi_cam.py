import pyzed.sl as sl
import cv2
import numpy as np
import threading
import time
import signal

zed_list = []
left_list = []
depth_list = []
timestamp_list = []
thread_list = []
stop_signal = False

pose_list = []
sensors_data_list = []

img_capture = False
depth_capture = False
pos_tracking = False


def signal_handler(signal, frame):
    global stop_signal
    stop_signal = True
    time.sleep(0.5)
    exit()


def grab_run(index):
    global stop_signal
    global zed_list
    global timestamp_list
    global left_list
    global depth_list
    global pose_list
    global sensors_data_list

    zed_cam = zed_list[index]
    zed_pose = pose_list[index]
    sensors_data = sensors_data_list[index]
    depth_img = depth_list[index]
    img = left_list[index]
    # timestamp = timestamp_list[index]

    runtime = sl.RuntimeParameters()
    while not stop_signal:
        err = zed_cam.grab(runtime)
        if err != sl.ERROR_CODE.SUCCESS:
            continue

        if img_capture:
            zed_cam.retrieve_image(img, sl.VIEW.LEFT)
            timestamp_list[index] = zed_cam.get_timestamp(
                sl.TIME_REFERENCE.CURRENT
            ).data_ns

        if depth_capture:
            zed_cam.retrieve_measure(
                depth_img,
                sl.MEASURE.DEPTH
            )

        if pos_tracking:
            zed_cam.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            zed_cam.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
            zed_imu = sensors_data.get_imu_data()

            # Смещение относительно точки отчёта
            py_translation = sl.Translation()
            tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
            ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
            tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
            print(f"Translation: {tx}, {ty}, {tz}\n")

            # Угловая скорость
            # a_velocity = np.zeros(3)
            # zed_imu.get_angular_velocity(a_velocity)
            a_velocity = zed_imu.get_angular_velocity()
            vx, vy, vz = np.round(a_velocity, 3)
            print(f"IMU ang velocity: {vx}, {vy}, {vz}\n")

            # Угловое ускорение
            # a_accel = np.zeros(3)
            # zed_imu.get_linear_acceleration(a_accel)
            a_accel = zed_imu.get_linear_acceleration()
            ax, ay, az = np.round(a_accel, 3)
            print(f"IMU ang acceleration: {ax}, {ay}, {az}\n")

        time.sleep(0.001)  # 1ms
    zed_cam.close()


def main():
    global stop_signal
    global zed_list
    global left_list
    global depth_list
    global timestamp_list
    global pose_list
    global sensors_data_list
    global thread_list
    signal.signal(signal.SIGINT, signal_handler)

    print('Configure init parameters')
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    init_params.camera_fps = 15

    stream_params = sl.StreamingParameters()
    stream_params.codec = sl.STREAMING_CODEC.H264
    stream_params.bitrate = 4000

    py_transform = sl.Transform()
    tracking_parameters = sl.PositionalTrackingParameters(
        _init_pos=py_transform
    )

    # List and open cameras
    cam_names = []
    last_ts_list = []
    cameras = sl.Camera.get_device_list()
    index = 0
    for cam in cameras:
        zed_serial = cam.serial_number
        init_params.set_from_serial_number(zed_serial)
        cam_names.append("ZED {}".format(zed_serial))

        print("Opening {}".format(zed_serial))
        zed = sl.Camera()
        timestamp_list.append(0)  # ??
        last_ts_list.append(0)  # ??

        status = zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            zed.close()
            continue

        print('Configuring stream parameters')
        stream_params.port = 30000 + index + index % 2
        status = zed.enable_streaming(stream_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            zed.close()
            continue

        # print('Init position tracking')
        # status = zed.enable_positional_tracking(tracking_parameters)
        # if status != sl.ERROR_CODE.SUCCESS:
        #     print(repr(status))
        #     zed.close()
        #     continue

        index = index + 1

        zed_list.append(zed)
        left_list.append(sl.Mat())
        depth_list.append(sl.Mat())
        pose_list.append(sl.Pose())
        sensors_data_list.append(sl.SensorsData())

    # Start camera threads
    for index in range(len(zed_list)):
        if zed_list[index].is_opened():
            thread_list.append(
                threading.Thread(
                    target=grab_run,
                    args=(index,)
                )
            )
            thread_list[index].start()
    # for zed in zed_list:
    #     if not zed.is_opened():
    #         continue
    #     thread_list.append(
    #         threading.Thread(
    #             target=zed_grab,
    #             args=(zed,)
    #         )
    #     )
    #     thread_list[-1].start()

    key = ''
    while key != 113:
        key = cv2.waitKey(10)

    # Stop the threads
    stop_signal = True
    for index in range(0, len(thread_list)):
        thread_list[index].join()

    print("\nFINISH")


if __name__ == "__main__":
    main()
