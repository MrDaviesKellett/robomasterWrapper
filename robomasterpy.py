import robomaster
import sys
from robomaster import robot
from robomaster import led
from typing import overload
from helperFuncs import clamp
from typing import Union
from time import sleep
from threading import Thread
from math import pi

RADTODEG = 180 / pi

if not (sys.version_info[0:2] == (3, 7) or sys.version_info[0:2] == (3, 8)):
    raise Exception("Robomaster lib Requires python 3.7 or 3.8")


class RoboMaster:

    # Initialisation and settings

    def __init__(
        self, conn_type: str = "device", protocol: str = "tcp", serial: str = None
    ):
        """
        Initialize the RoboMaster SDK.
        Args:
        conn_type (str, optional): Connection type. "device", "station" or "usb". Defaults to "device".
        protocol (str, optional): Protocol. "tcp" or "udp". Defaults to "tcp".
        serial (str, optional): Serial number of the RoboMaster device. Defaults to None.
        """
        self.robot = robot.Robot()
        if conn_type.lower() == "sta":
            connection = "sta"
        elif conn_type.lower() == "station":
            connection = "sta"
        elif conn_type.lower() == "ap":
            connection = "ap"
        elif conn_type.lower() == "device":
            connection = "ap"
        elif conn_type.lower() == "rndis":
            connection = "rndis"
        elif conn_type.lower() == "usb":
            connection = "rndis"
        else:
            connection = "ap"

        self.robot.initialize(conn_type=connection, proto_type=protocol, sn=serial)
        self.battery = self.robot.battery
        self.blaster = self.robot.blaster
        self.camera = self.robot.camera
        self.chassis = self.robot.chassis
        self.gimbal = self.robot.gimbal
        self.led = self.robot.led
        self.robotic_arm = self.robot.robotic_arm
        self.vision = self.robot.vision
        self.reset()
        self.setRobotMode("chassis")

    def __repr__(self) -> str:
        """
        Returns the RoboMaster's serial number.
        Returns:
        str: Serial number of the RoboMaster device.
        """
        return f"RoboMaster {self.robot.get_sn()}"

    def getModule(self, module: str) -> object:
        """
        Get a module from the RoboMaster SDK.
        Args:
        module (str): Module name.
        Returns:
        object: The module.
        """
        if module.lower() == "battery":
            return self.battery
        elif module.lower() == "blaster":
            return self.blaster
        elif module.lower() == "camera":
            return self.camera
        elif module.lower() == "chassis":
            return self.chassis
        elif module.lower() == "gimbal":
            return self.gimbal
        elif module.lower() == "led":
            return self.led
        elif module.lower() == "robotic_arm":
            return self.robotic_arm
        elif module.lower() == "vision":
            return self.vision
        else:
            raise ValueError(f"Invalid module name {module}")

    def setRobotMode(self, mode: str = "free"):
        """
        Set the robot mode.
        Args:
        mode (str, optional): Robot mode. free, chassis, gimbal. Defaults to "free".
        """
        if mode.lower() == "free":
            self.mode = "free"
            md = robot.FREE
        elif mode.lower() == "chassis":
            self.mode = "chassis"
            md = robot.CHASSIS_LEAD
        elif mode.lower() == "gimbal":
            self.mode = "gimbal"
            md = robot.GIMBAL_LEAD
        else:
            self.mode = "chassis"
            md = robot.CHASSIS_LEAD
        self.robot.set_robot_mode(md)

    def getRobotMode(self) -> str:
        """
        Get the robot mode.
        Returns:
        str: Robot mode. free, chassis, gimbal.
        """
        return self.mode

    def getSN(self) -> str:
        """
        Get the serial number of the RoboMaster device.
        Returns:
        str: The serial number.
        """
        return self.robot.get_sn()

    def getVersion(self) -> str:
        """
        Get the version of the RoboMaster device.
        Returns:
        str: The version.
        """
        return self.robot.get_version()

    def reset(self) -> None:
        """
        Reset the RoboMaster device.
        """
        self.robot.reset()
        sleep(1)

    # Audio

    def playAudio(
        self, path: str, blocking: bool = False, timeout=None
    ) -> robomaster.action.Action:
        """
        Play an audio file.
        Args:
        path (str): Path to the audio file.
        """
        if not blocking:
            return self.robot.play_audio(path)
        else:
            return self.robot.play_audio(path).wait_for_completed(timeout)
        print(f"Playing audio: {path}")

    def playSound(
        self, soundID: int, blocking: bool = False, timeout=None
    ) -> robomaster.action.Action:
        """
        Play a sound.
        Args:
        sound (str): Sound name.
        """
        if not blocking:
            return self.robot.play_sound(soundID)
        else:
            return self.robot.play_sound(soundID).wait_for_completed(timeout)
        print(f"Playing sound: {sound}")

    # LEDs

    @overload
    def setLEDs(self, r: int, g: int, b: int):
        """
        Set the robot LEDs to a specific colour.
        Args:
        r(int): Red value.
        g(int): Green value.
        b(int): Blue value.
        """
        ...

    @overload
    def setLEDs(self, r: int, g: int, b: int, leds: str):
        """
        Set the robot LEDs to a specific colour.
        Args:
        r(int): Red value.
        g(int): Green value.
        b(int): Blue value.
        leds (str): LED component. front, back, left, right, gimbal, gimbalLeft, gimbalRight, all. Defaults to "all".
        """
        ...

    @overload
    def setLEDs(self, r: int, g: int, b: int, effect: str):
        """
        Set the robot LEDs to a specific colour.
        Args:
        r(int): Red value.
        g(int): Green value.
        b(int): Blue value.
        effect (str): LED effect. on, off, pulse, flash, breath, scrolling. Defaults to "on".
        """
        ...

    @overload
    def setLEDs(self, r: int, g: int, b: int, leds: str, effect: str):
        """
        Set the robot LEDs to a specific colour.
        Args:
        r(int): Red value.
        g(int): Green value.
        b(int): Blue value.
        leds (str): LED component. front, back, left, right, gimbal, gimbalLeft, gimbalRight, all. Defaults to "all".
        effect (str): LED effect. on, off, pulse, flash, breath, scrolling. Defaults to "on".
        """
        ...

    def setLEDs(
        self,
        r: Union[int, None] = None,
        g: Union[int, None] = None,
        b: Union[int, None] = None,
        leds: Union[str, None] = None,
        effect: Union[str, None] = None,
    ):

        if r is None or g is None or b is None:
            raise ValueError("Please specify a colour or RGB values.")

        comp = led.COMP_ALL
        effect = led.EFFECT_ON

        if leds is not None:
            if leds == "front":
                comp = led.COMP_BOTTOM_FRONT
            elif leds == "back":
                comp = led.COMP_BOTTOM_BACK
            elif leds == "left":
                comp = led.COMP_BOTTOM_LEFT
            elif leds == "right":
                comp = led.COMP_BOTTOM_RIGHT
            elif leds == "gimbal":
                comp = led.COMP_TOP_ALL
            elif leds == "gimbalLeft":
                comp = led.COMP_TOP_LEFT
            elif leds == "gimbalRight":
                comp = led.COMP_TOP_RIGHT
            elif leds == "all":
                comp = led.COMP_ALL
            else:
                comp = led.COMP_ALL
                effect = leds

        if effect is not None:
            if effect == "on":
                effect = led.EFFECT_ON
            elif effect == "off":
                effect = led.EFFECT_OFF
            elif effect == "pulse":
                effect = led.EFFECT_PULSE
            elif effect == "flash":
                effect = led.EFFECT_FLASH
            elif effect == "breath":
                effect = led.EFFECT_BREATH
            elif effect == "scrolling":
                effect = led.EFFECT_SCROLLING

        self.led.set_led(comp=comp, r=r, g=g, b=b, effect=effect)

    # Chassis

    def stop(self) -> None:
        """
        Stop the chassis from moving
        """
        self.robot.chassis.stop()

    def stopAfter(self, duration: float = 1.0, blocking: bool = False):
        """
        wait for a certain period of time and then stop the robot.
        Args:
        duration (float): Duration to wait in seconds. Defaults to 1.0.
        blocking (bool): Whether to block until the action is completed. Defaults to False.
        """

        def stopAfterThread(duration: float = 1.0):
            """
            sub method to create a thread to wait for a certain period of time and then stop the robot.
            """
            sleep(duration)
            self.stop()

        if not blocking:
            stop_thread = Thread(target=stopAfterThread, args=(duration))
            stop_thread.start()
        else:
            sleep(duration)
            self.stop()

    def setSpeed(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        timeout: Union[int, None] = None,
    ) -> None:
        """
        Set the chassis speed in m/s - max speed 3.5m/s.
        Rotate the chassis about the Z axis in °/s - max speed 600°/s (1.6 rotations/s).
        Positive values move forward, negative values move backward.
        Positive degrees turn left, negative degrees turn right.
        Args:
        x (float): Speed in x axis (m/s). Defaults to 0.0.
        y (float): Speed in y axis (m/s). Defaults to 0.0.
        z (float): Rotation Speed about the z axis (°/s). Defaults to 0.0.
        timeout (int): Timeout for the action. Defaults to None.
        """
        if -3.5 > x > 3.5:
            print("X speed is out of range.")
            print("X requires a value between -3.5 and 3.5")
            print("Limiting X to +-3.5")
            x = clamp(x, -3.5, 3.5)
        if -3.5 > y > 3.5:
            print("Y speed is out of range.")
            print("Y requires a value between -3.5 and 3.5")
            print("Limiting Y to +-3.5")
            y = clamp(y, -3.5, 3.5)
        if -600 > z > 600:
            print("Z speed is out of range.")
            print("Z requires a value between -600 and 600")
            print("Limiting Z to +-600")
            z = clamp(z, -600, 600)

        self.robot.chassis.drive_speed(x=x, y=y, z=z, timeout=timeout)

    def rotate(
        self, angularVelocity: float = 0.0, timeout: Union[int, None] = None
    ) -> None:
        """
        Rotate the chassis about the Z axis in °/s - max speed 600°/s  (1.6 rotations/s).
        Positive degrees rotate left, negative degrees rotate right.
        Args:
        angularVelocity (float): Rotation Speed about the z axis (°/s). Defaults to 0.0.
        timeout (int): Timeout for the action. Defaults to None.
        """
        self.setSpeed(z=angularVelocity, timeout=timeout)

    def rotateLeft(
        self, angularVelocity: float = 0.0, timeout: Union[int, None] = None
    ) -> None:
        """
        Rotate the chassis about the Z axis in °/s - max speed 600°/s  (1.6 rotations/s).
        Positive degrees rotate left, negative degrees rotate right.
        Args:
        angularVelocity (float): Rotation Speed about the z axis (°/s). Defaults to 0.0.
        timeout (int): Timeout for the action. Defaults to None.
        """
        self.rotate(angularVelocity=angularVelocity, timeout=timeout)

    def rotateRight(
        self, angularVelocity: float = 0.0, timeout: Union[int, None] = None
    ) -> None:
        """
        Rotate the chassis about the Z axis in °/s - max speed 600°/s  (1.6 rotations/s).
        Positive degrees rotate right, negative degrees rotate left.
        Args:
        angularVelocity (float): Rotation Speed about the z axis (°/s). Defaults to 0.0.
        timeout (int): Timeout for the action. Defaults to None.
        """
        self.rotate(angularVelocity=-angularVelocity, timeout=timeout)

    def turn(
        self, angle: float = 0.0, speed: float = 90.0, blocking: bool = True
    ) -> None:
        """
        Rotate the chassis about the Z axis in °/s - max speed 600°/s (1.6 rotations/s).
        Positive degrees turn left, negative degrees turn right.
        Args:
        angle (float): Rotation amount about the z axis (°). Defaults to 0.0.
        speed (float): Speed of rotation in °/s. Defaults to 90.0.
        blocking (bool): Whether the action should block until complete or not. Defaults to True.
        """
        if not blocking:
            self.robot.chassis.move(z=angle, z_speed=speed)
        else:
            self.robot.chassis.move(z=angle, z_speed=speed).wait_for_completed()

    def turnLeft(
        self, angle: float = 0.0, speed: float = 90.0, blocking: bool = True
    ) -> None:
        """
        Rotate the chassis about the Z axis in °/s - max speed 600°/s  (1.6 rotations/s).
        Positive degrees turn left, negative degrees turn right.
        Args:
        angle (float): Rotation Speed about the z axis (°/s). Defaults to 0.0.
        speed (float): Speed of rotation in °/s. Defaults to 90.0.
        blocking (bool): Whether the action should block until complete or not. Defaults to True.
        """
        self.turn(angle=angle, speed=speed, blocking=blocking)

    def turnRight(
        self, angle: float = 0.0, speed: float = 90.0, blocking: bool = True
    ) -> None:
        """
        Rotate the chassis about the Z axis in °/s - max speed 600°/s  (1.6 rotations/s).
        Positive degrees turn right, negative degrees turn left.
        Args:
        angle (float): Rotation Speed about the z axis (°/s). Defaults to 0.0.
        speed (float): Speed of rotation in °/s. Defaults to 90.0.
        blocking (bool): Whether the action should block until complete or not. Defaults to True.
        """
        self.turn(angle=-angle, speed=speed, blocking=blocking)

    def setWheelRPMs(
        self,
        frontRight: int = 0,
        frontLeft: int = 0,
        backRight: int = 0,
        backLeft: int = 0,
        timeout: Union[int, None] = None,
    ) -> None:
        """
        Set the chassis wheels in Revolutions Per Minute (RPMs).
        Maximum speed is 1000 RPMs (16.6 rotations/s) (approx 3.5m/s).
        Positive values rotate the wheels clockwise, negative values counterclockwise.
        Therefore positive values move forward, negative values move backward.
        Combinations of positive and negative values are possible, this will result in the chassis turning.
        Args:
        frontRight (int): Speed of the front right wheel (RPMs). Defaults to 0.
        frontLeft (int): Speed of the front right wheel (RPMs). Defaults to 0.
        backRight (int): Speed of the back right wheel (RPMs). Defaults to 0.
        backLeft (int): Speed of the back left wheel (RPMs). Defaults to 0.
        timeout (int): Timeout for the action (seconds). Defaults to None.
        """
        if -1000 > frontRight > 1000:
            print("Front right wheel speed is out of range.")
            print("Front right wheel requires a value between -1000 and 1000")
            print("Limiting Front right wheel to +-1000")
            frontRight = clamp(frontRight, -1000, 1000)
        if -1000 > frontLeft > 1000:
            print("Front left wheel speed is out of range.")
            print("Front left wheel requires a value between -1000 and 1000")
            print("Limiting Front left wheel to +-1000")
            frontLeft = clamp(frontLeft, -1000, 1000)
        if -1000 > backRight > 1000:
            print("Back right wheel speed is out of range.")
            print("Back right wheel requires a value between -1000 and 1000")
            print("Limiting Back right wheel to +-1000")
            backRight = clamp(backRight, -1000, 1000)
        if -1000 > backLeft > 1000:
            print("Back left wheel speed is out of range.")
            print("Back left wheel requires a value between -1000 and 1000")
            print("Limiting Back left wheel to +-1000")
            backLeft = clamp(backLeft, -1000, 1000)

        self.robot.chassis.drive_wheels(
            w1=frontRight, w2=frontLeft, w3=backRight, w4=backLeft, timeout=timeout
        )

    def move(
        self,
        xDistance: float = 0.0,
        yDistance: float = 0.0,
        zRotation: float = 0.0,
        speed: float = 1,
        turnSpeed: float = 90,
        blocking: bool = True,
    ) -> robomaster.action.Action:
        """
        Move the chassis a set distance in meters from its current position.
        Positive values in X move forward, negative values move backward.
        Positive values in Y move left, negative values move right.
        Positive values in Z rotate left, negative values rotate right.
        Speed is in meters/second (m/s). default speed is 1 m/s.
        Turning speed is in degrees/second (°/s). Default turning speed is 90°/s.
        Maximum move distance is 5 meters.
        Speed must be within the range of 0.5 and 2.0 m/s.
        Turning speed must be within the range of 10 and 540°/s.
        Args:
        xDistance (float): Distance to move in the x direction (meters). Defaults to 0.0.
        yDistance (float): Distance to move in the y direction (meters). Defaults to 0.0.
        zRotation (float): Angle to rotate around the z axis (°). Defaults to 0.0.
        speed (float): Speed of the chassis (m/s). Defaults to 1 m/s.
        turnSpeed (float): Turning speed of the chassis (°/s). Defaults to 90°/s.
        blocking (bool): Block until action is complete. Defaults to False.
        Returns:
        robomaster.action.Action: Action object that can be used to wait for completion or get feedback
        """
        if not blocking:
            return self.robot.chassis.move(
                x=xDistance, y=yDistance, z=zRotation, xy_speed=speed, z_speed=turnSpeed
            )
        else:
            return self.robot.chassis.move(
                x=xDistance, y=yDistance, z=zRotation, xy_speed=speed, z_speed=turnSpeed
            ).wait_for_completed()

    def forward(
        self,
        distance: float = 0.5,
        speed: float = 1,
        blocking: bool = True,
    ) -> robomaster.action.Action:
        """
        Move the chassis a set distance in meters from its current position.
        Positive values move forward, negative values move backward.
        Speed is in meters/second (m/s). default speed is 0.5 m/s.
        Maximum move distance is 5 meters.
        Speed must be within the range of 0.5 and 2.0 m/s.
        Args:
        distance (float): Distance to move in the x direction (meters). Defaults to 0.5.
        speed (float): Speed of the chassis (m/s). Defaults to 0.5 m/s.
        blocking (bool): Block until action is complete. Defaults to False.
        Returns:
        robomaster.action.Action: Action object that can be used to wait for completion or get feedback
        """
        if not blocking:
            return self.robot.chassis.move(x=distance, xy_speed=speed)
        else:
            return self.robot.chassis.move(
                x=distance, xy_speed=speed
            ).wait_for_completed()

    def back(
        self,
        distance: float = 0.5,
        speed: float = 1,
        blocking: bool = True,
    ) -> robomaster.action.Action:
        """
        Move the chassis a set distance in meters from its current position.
        Positive values move backward, negative values move forward.
        Speed is in meters/second (m/s). default speed is 0.5 m/s.
        Maximum move distance is 5 meters.
        Speed must be within the range of 0.5 and 2.0 m/s.
        Args:
        distance (float): Distance to move in the x direction (meters). Defaults to 0.5.
        speed (float): Speed of the chassis (m/s). Defaults to 0.5 m/s.
        blocking (bool): Block until action is complete. Defaults to False.
        Returns:
        robomaster.action.Action: Action object that can be used to wait for completion or get feedback
        """
        self.forward(distance=-distance, speed=speed, blocking=blocking)

    def backward(
        self,
        distance: float = 0.5,
        speed: float = 1,
        turnSpeed: float = 30,
        blocking: bool = True,
    ) -> robomaster.action.Action:
        """
        Move the chassis a set distance in meters from its current position.
        Positive values move backward, negative values move forward.
        Speed is in meters/second (m/s). default speed is 0.5 m/s.
        Maximum move distance is 5 meters.
        Speed must be within the range of 0.5 and 2.0 m/s.
        Args:
        distance (float): Distance to move in the x direction (meters). Defaults to 0.5.
        speed (float): Speed of the chassis (m/s). Defaults to 0.5 m/s.
        blocking (bool): Block until action is complete. Defaults to False.
        Returns:
        robomaster.action.Action: Action object that can be used to wait for completion or get feedback
        """
        self.back(distance=distance, speed=speed, blocking=blocking)

    def left(
        self,
        distance: float = 0.5,
        speed: float = 1,
        blocking: bool = True,
    ) -> robomaster.action.Action:
        """
        Move the chassis a set distance in meters from its current position.
        Positive values move left, negative values move right.
        Speed is in meters/second (m/s). default speed is 0.5 m/s.
        Maximum move distance is 5 meters.
        Speed must be within the range of 0.5 and 2.0 m/s.
        Args:
        distance (float): Distance to move in the x direction (meters). Defaults to 0.5.
        speed (float): Speed of the chassis (m/s). Defaults to 0.5 m/s.
        blocking (bool): Block until action is complete. Defaults to False.
        Returns:
        robomaster.action.Action: Action object that can be used to wait for completion or get feedback
        """
        if not blocking:
            return self.robot.chassis.move(y=-distance, xy_speed=speed)
        else:
            return self.robot.chassis.move(
                y=-distance, xy_speed=speed
            ).wait_for_completed()

    def right(
        self,
        distance: float = 0.5,
        speed: float = 1,
        blocking: bool = True,
    ) -> robomaster.action.Action:
        """
        Move the chassis a set distance in meters from its current position.
        Positive values move right, negative values move left.
        Speed is in meters/second (m/s). default speed is 0.5 m/s.
        Maximum move distance is 5 meters.
        Speed must be within the range of 0.5 and 2.0 m/s.
        Args:
        distance (float): Distance to move in the x direction (meters). Defaults to 0.5.
        speed (float): Speed of the chassis (m/s). Defaults to 0.5 m/s.
        turnspeed (float): Turning speed of the chassis (°/s). Defaults to 30°/s.
        blocking (bool): Block until action is complete. Defaults to False.
        Returns:
        robomaster.action.Action: Action object that can be used to wait for completion or get feedback
        """
        self.left(distance=-distance, speed=speed, blocking=blocking)

    def circle(
        self,
        radius: float = 0.25,
        speed: float = 1.0,
        numCircles: int = 1,
        blocking: bool = True,
    ) -> None:
        """
        Circle the chassis a set distance in meters from its current position.
        Speed is in meters/second (m/s). default speed is 0.5 m/s.
        Maximum move distance is 5 meters.
        Speed must be within the range of 0.5 and 2.0 m/s.
        Args:
        radius (float): Distance to circle in the x direction (meters). Defaults to 0.25.
        speed (float): Speed of the chassis (m/s). Defaults to 1.0 m/s.
        numCircles (int): Number of circles to make. Defaults to 1.
        blocking (bool): Block until action is complete. Defaults to False.
        """
        angle = speed / radius * RADTODEG
        circumference = 2 * pi * radius
        duration = circumference / speed
        self.setSpeed(x=speed, z=angle)
        self.stopAfter(duration=duration * numCircles, blocking=blocking)
