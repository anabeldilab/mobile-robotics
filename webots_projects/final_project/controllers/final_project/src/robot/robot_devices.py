import numpy as np
import cv2

class RobotDevices:
    WHEEL_BASE = 0.10829  # meters (105.4 mm)
    WHEEL_RADIUS = 0.021  # meters (21 mm)

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
        self.pos_l, self.pos_r = self.init_position_sensors()
        self.gyro = self.init_gyro()
        self.robot.step(self.time_step)


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
    
    def detect_yellow_block(self, threshold=0.19): # 0.19
        """
        Detect a yellow block in front of the robot using the camera.

        Parameters:
        - threshold: Proportion of pixels to consider the object detected (0..1).

        Returns:
        - bool: True if the object is detected, False otherwise.
        """
        # Constants for yellow color in HSV
        COLOR_RANGE_1 = (25, 50, 50)  # Lower bound of yellow in HSV
        COLOR_RANGE_2 = (35, 255, 255)  # Upper bound of yellow in HSV

        # Get camera image dimensions
        W = self.camera.getWidth()
        H = self.camera.getHeight()

        # Get the image from the camera
        image = np.frombuffer(self.camera.getImage(), np.uint8).reshape((H, W, 4))

        # Convert the image from BGRA to BGR
        image_bgr = image[:, :, :3]

        # Convert BGR image to HSV
        image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        # Create a mask for yellow color
        mask = cv2.inRange(image_hsv, COLOR_RANGE_1, COLOR_RANGE_2)

        # Count non-zero pixels in the mask
        non_zero_count = cv2.countNonZero(mask)

        # Calculate the ratio of yellow pixels to the total number of pixels
        yellow_ratio = non_zero_count / (W * H)

        print(f"Yellow ratio: {yellow_ratio}")
        print(f"Threshold: {threshold}")

        # Return True if the yellow ratio exceeds the threshold
        return yellow_ratio > threshold

    def init_position_sensors(self):
        """
        Initialize and enable the position sensors.

        Returns:
        - tuple: left and right position sensors.
        """
        pos_l = self.robot.getDevice("left wheel sensor")
        pos_r = self.robot.getDevice("right wheel sensor")
        pos_l.enable(self.time_step)
        pos_r.enable(self.time_step)
        return pos_l, pos_r
    
    def init_gyro(self):
        """
        Initialize and enable the gyro sensor.

        Returns:
        - gyro: The enabled gyro sensor.
        """
        gyro = self.robot.getDevice("gyro")
        gyro.enable(self.time_step)
        return gyro
