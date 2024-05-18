class RobotDevices:
    def __init__(self, time_step, robot):
        """
        Initialize the RobotDevices class.

        Parameters:
        - time_step: Time step for sensor and actuator updates.
        - robot: Instance of the robot.
        """
        self.time_step = time_step
        self.robot = robot
        self.left_wheel, self.right_wheel = self.init_wheels()
        self.ir_sensor_list = self.enable_distance_sensors()
        self.camera = self.init_camera()

    def init_wheels(self):
        """
        Initialize and set the wheels of the robot.

        Returns:
        - tuple: left and right wheel motor devices.
        """
        left_wheel = self.robot.getDevice("left wheel motor")
        right_wheel = self.robot.getDevice("right wheel motor")
        left_wheel.setPosition(float("inf"))
        right_wheel.setPosition(float("inf"))
        left_wheel.setVelocity(0)
        right_wheel.setVelocity(0)
        return left_wheel, right_wheel

    def enable_distance_sensors(self):
        """
        Enable and retrieve distance sensors.

        Returns:
        - dict: Dictionary with activated distance sensors.
        """
        sensor_names = [
            "rear left infrared sensor",
            "left infrared sensor",
            "front left infrared sensor",
            "front infrared sensor",
            "front right infrared sensor",
            "right infrared sensor",
            "rear right infrared sensor",
            "rear infrared sensor",
        ]
        sensor_dict = {}
        for name in sensor_names:
            sensor_dict[name] = self.robot.getDevice(name)
            sensor_dict[name].enable(self.time_step)
        return sensor_dict

    def init_camera(self):
        """
        Initialize and enable the camera.

        Returns:
        - camera: The enabled camera device.
        """
        camera = self.robot.getDevice("camera")
        camera.enable(self.time_step * 10)
        return camera
