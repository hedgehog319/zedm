import pyzed.sl as sl
from time import sleep


def main():
    zed = sl.Camera()

    print('configuring')
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.coordinate_units = sl.UNIT.METER
    init_params.camera_fps = 15

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        exit(1)

    print('init tracking')
    # Включение обработки позиции
    py_transform = sl.Transform()
    tracking_parameters = sl.PositionalTrackingParameters(
        _init_pos=py_transform)
    if zed.enable_positional_tracking(tracking_parameters) != sl.ERROR_CODE.SUCCESS:
        exit(1)

    print('init pose and sensor data')
    # Данные о позиции и данные с датчиков
    zed_pose = sl.Pose()
    sensors_data = sl.SensorsData()

    image = sl.Mat()
    while True:
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            continue

        # Получение изображения
        # zed.retrieve_image(image, sl.VIEW.LEFT)
        # timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
        # print(f"Image resolution: {image.get_width()} x {image.get_height()} || \
        #     Image timestamp: {timestamp.get_milliseconds()}\n")

        # Трекинг позиции
        zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
        zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE)
        zed_imu = sensors_data.get_imu_data()

        py_translation = sl.Translation()
        tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
        ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
        tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
        print(f"Translation: {tx=}, {ty=}, {tz=}, \
              Timestamp: {zed_pose.timestamp.get_milliseconds()}\n")

        # Угловая скорость
        a_velocity = [0, 0, 0]
        zed_imu.get_angular_velocity(a_velocity)
        vx = round(a_velocity[0], 3)
        vy = round(a_velocity[1], 3)
        vz = round(a_velocity[2], 3)
        print(f"IMU Angular Velocity: {vx=}, {vy=}, {vz=}\n")
        sleep(0.5)

    # Close the camera
    zed.close()


if __name__ == "__main__":
    main()
