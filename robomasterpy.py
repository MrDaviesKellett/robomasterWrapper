import robomaster
from robomaster import robot
from robomaster import led
from random import randint
from typing import overload

import robomaster.action


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
        match conn_type.lower():
            case "sta":
                connection = "sta"
            case "station":
                connection = "sta"
            case "ap":
                connection = "ap"
            case "device":
                connection = "ap"
            case "rndis":
                connection = "rndis"
            case "usb":
                connection = "rndis"
            case _:
                connection = "sta"
        self.robot.initialize(conn_type=connection, proto_type=protocol, sn=serial)
        self.battery = self.robot.battery
        self.blaster = self.robot.blaster
        self.camera = self.robot.camera
        self.chassis = self.robot.chassis
        self.gimbal = self.robot.gimbal
        self.led = self.robot.led
        self.robotic_arm = self.robot.robotic_arm
        self.vision = self.robot.vision
        self.mode = "free"

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
        match module.lower():
            case "battery":
                return self.battery
            case "blaster":
                return self.blaster
            case "camera":
                return self.camera
            case "chassis":
                return self.chassis
            case "gimbal":
                return self.gimbal
            case "led":
                return self.led
            case "robotic_arm":
                return self.robotic_arm
            case "vision":
                return self.vision
            case _:
                raise ValueError(f"Invalid module name {module}")

    def setRobotMode(self, mode: str = "free"):
        """
        Set the robot mode.
        Args:
        mode (str, optional): Robot mode. free, chassis, gimbal. Defaults to "free".
        """
        match mode.lower():
            case "free":
                self.mode = "free"
                md = "sta"
            case "chassis":
                self.mode = "chassis"
                md = robot.CHASSIS_LEAD
            case "gimbal":
                self.mode = "gimbal"
                md = robot.GIMBAL_LEAD
            case _:
                self.mode = "gimbal"
                md = robot.GIMBAL_LEAD
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
        print("Resetting...")

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
        r: int | None = None,
        g: int | None = None,
        b: int | None = None,
        leds: str | None = None,
        effect: str | None = None,
    ):

        if r is None or g is None or b is None:
            raise ValueError("Please specify a colour or RGB values.")

        if leds is not None:
            match leds:
                case "front":
                    comp = led.COMP_BOTTOM_FRONT
                case "back":
                    comp = led.COMP_BOTTOM_BACK
                case "left":
                    comp = led.COMP_BOTTOM_LEFT
                case "right":
                    comp = led.COMP_BOTTOM_RIGHT
                case "gimbal":
                    comp = led.COMP_TOP_ALL
                case "gimbalLeft":
                    comp = led.COMP_TOP_LEFT
                case "gimbalRight":
                    comp = led.COMP_TOP_RIGHT
                case "all":
                    comp = led.COMP_ALL
                case _:
                    effect = leds

        if effect is not None:
            match effect:
                case "on":
                    effect = led.EFFECT_ON
                case "off":
                    effect = led.EFFECT_OFF
                case "pulse":
                    effect = led.EFFECT_PULSE
                case "flash":
                    effect = led.EFFECT_FLASH
                case "breath":
                    effect = led.EFFECT_BREATH
                case "scrolling":
                    effect = led.EFFECT_SCROLLING

        self.led.set_led(comp=comp, r=r, g=g, b=b, effect=effect)

    def move(self, speed=100, radius=50, direction="forward"):
        if direction == "forward":
            self.robot.drive_direct(speed, 0)
        elif direction == "backward":
            self.robot.drive_direct(-speed, 0)
        elif direction == "left":
            self.robot.drive_direct(0, speed)
        elif direction == "right":
            self.robot.drive_direct(0, -speed)
        else:
            print("Invalid direction")

    def turn(self, speed=100, radius=50, direction="left"):
        if direction == "left":
            self.robot.drive_direct(-speed, 0)
        elif direction == "right":
            self.robot.drive_direct(speed, 0)
        else:
            print("Invalid direction")

    def stop(self):
        self.robot.stop()

    def shoot(self):
        self.robot.shoot()
