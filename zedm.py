import pyzed.sl as sl
import numpy as np
from time import sleep

# img_capture = False
img_capture = True

pos_tracking = False
# pos_tracking = True


COUNTER_MAX_VALUE = 200


def main():
    print('Configure init parameters')

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    init_params.camera_fps = 15

    print('Open the camera')

    zed = sl.Camera()
    zed_status = zed.open(init_params)
    if zed_status != sl.ERROR_CODE.SUCCESS:
        print(repr(zed_status))
        exit(1)

    print('Configure runtime parameters')

    runtime_params = sl.RuntimeParameters()

    print('Configuring stream parameters')

    stream_params = sl.StreamingParameters()
    stream_params.codec = sl.STREAMING_CODEC.H264
    stream_params.bitrate = 4000
    stream_params.port = 30000

    zed_status = zed.enable_streaming(stream_params)
    if zed_status != sl.ERROR_CODE.SUCCESS:
        print(repr(zed_status))
        exit(1)

    print('Init position tracking')
    py_transform = sl.Transform()
    tracking_parameters = sl.PositionalTrackingParameters(
        _init_pos=py_transform)
    # zed_status = zed.enable_positional_tracking(tracking_parameters)
    if zed_status != sl.ERROR_CODE.SUCCESS:
        print(repr(zed_status))
        exit(1)

    print('Init pose data')
    zed_pose = sl.Pose()

    print('Init sensors data')
    sensors_data = sl.SensorsData()

    image = sl.Mat()
    counter = 0

    while True:
        zed_status = zed.grab(runtime_params)
        if zed_status != sl.ERROR_CODE.SUCCESS:
            print(zed_status)
            continue

        # Получение изображения
        if img_capture:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
            # print(f"Image resolution: {image.get_width()}x{image.get_height()} || \
                # Image timestamp: {timestamp.get_milliseconds()}\n")

        # Трекинг позиции
        if pos_tracking:
            zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
            zed_imu = sensors_data.get_imu_data()

            # Смещение относительно точки отчёта
            py_translation = sl.Translation()
            tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
            ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
            tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
            print(f"Translation: {tx=}, {ty=}, {tz=}\n")

            # Угловая скорость
            # a_velocity = np.zeros(3)
            # zed_imu.get_angular_velocity(a_velocity)
            a_velocity = zed_imu.get_angular_velocity()
            vx, vy, vz = np.round(a_velocity, 3)
            print(f"IMU ang velocity: {vx=}, {vy=}, {vz=}\n")

            # Угловое ускорение
            # a_accel = np.zeros(3)
            # zed_imu.get_linear_acceleration(a_accel)
            a_accel = zed_imu.get_linear_acceleration()
            ax, ay, az = np.round(a_accel, 3)
            print(f"IMU ang acceleration: {ax=}, {ay=}, {az=}\n")

    print('Disable streaming')
    zed.disable_streaming()

    print('Close the camera')
    zed.close()


if __name__ == "__main__":
    main()
