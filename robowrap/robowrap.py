from __future__ import annotations

from math import pi
from threading import Timer
from time import sleep
from typing import Any, Optional

from ._sdk import LED_COMPONENTS, LED_EFFECTS, create_sdk_robot, normalize_robot_mode, resolve_robot_mode
from .arm import Arm, Gripper
from .camera import Camera
from .gun import Gun
from .helperFuncs import (
    deprecated_alias,
    ensure_choice,
    ensure_int_range,
    ensure_range,
    wait_for_action,
)
from .modules import AiModule, Armor, Battery, Chassis, DistanceSensor, SensorAdaptor, Servo, Uart

RAD_TO_DEG = 180.0 / pi


class RoboMaster:
    def __init__(
        self,
        conn_type: str = "device",
        protocol: str = "tcp",
        serial: Optional[str] = None,
        *,
        auto_reset: bool = True,
        default_mode: str = "chassis",
        connect_retries: int = 2,
        connect_retry_delay: float = 1.0,
        _sdk_robot: Optional[Any] = None,
        _sleep=sleep,
    ) -> None:
        self.sleep = _sleep
        self.robot = _sdk_robot if _sdk_robot is not None else create_sdk_robot()
        self._mode = "free"

        connection = self._normalize_connection_type(conn_type)
        ensure_choice("protocol", protocol.lower(), ("tcp", "udp"))
        ensure_int_range("connect_retries", connect_retries, 0, 20)
        ensure_range("connect_retry_delay", connect_retry_delay, 0.0, 30.0, unit="seconds")

        initialize_result: Any = False
        initialize_error: Optional[Exception] = None
        for attempt in range(connect_retries + 1):
            try:
                initialize_result = self.robot.initialize(conn_type=connection, proto_type=protocol.lower(), sn=serial)
            except Exception as exc:
                initialize_result = False
                initialize_error = exc
            if initialize_result is not False:
                break
            if attempt < connect_retries:
                self.sleep(connect_retry_delay)
        if initialize_result is False:
            message = "Robot could not be connected. Check power, network mode, and serial number."
            if initialize_error is not None:
                message = f"{message} Last error: {initialize_error}"
            raise RuntimeError(message)

        self.chassis = Chassis(self)
        self.gun = Gun(self)
        self.gripper = Gripper(self)
        self.arm = Arm(self)
        self.arm.gripper = self.gripper
        self.cam = Camera(self)
        self.battery = Battery(self)
        self.servo = Servo(self)
        self.sensor = DistanceSensor(self)
        self.sensor_adaptor = SensorAdaptor(self)
        self.armor = Armor(self)
        self.uart = Uart(self)
        self.ai_module = AiModule(self)

        if auto_reset:
            self.reset()
        self.set_robot_mode(default_mode)

    def __repr__(self) -> str:
        return f"RoboMaster {self.get_sn()}"

    def _normalize_connection_type(self, conn_type: str) -> str:
        mapping = {
            "sta": "sta",
            "station": "sta",
            "ap": "ap",
            "device": "ap",
            "rndis": "rndis",
            "usb": "rndis",
        }
        return mapping.get(conn_type.lower(), "ap")

    def _complete_action(self, action: Any, *, blocking: bool = True, timeout: Optional[float] = None) -> Any:
        return wait_for_action(action, blocking=blocking, timeout=timeout)

    def close(self) -> Any:
        self.stop()
        self.cam.stop()
        return self.robot.close()

    def get_module(self, module: str) -> Any:
        normalized = module.lower()
        modules = {
            "arm": self.arm,
            "gripper": self.gripper,
            "gun": self.gun,
            "gimbal": self.gun,
            "camera": self.cam,
            "cam": self.cam,
            "chassis": self.chassis,
            "battery": self.battery,
            "servo": self.servo,
            "sensor": self.sensor,
            "sensor_adaptor": self.sensor_adaptor,
            "armor": self.armor,
            "uart": self.uart,
            "ai_module": self.ai_module,
            "led": self.robot.led,
            "vision": self.robot.vision,
            "blaster": self.robot.blaster,
            "dds": self.robot.dds,
        }
        if normalized not in modules:
            raise ValueError(f"Unknown module '{module}'.")
        return modules[normalized]

    def set_robot_mode(self, mode: str = "free") -> Any:
        ensure_choice("mode", mode.lower(), ("free", "chassis", "gun"))
        self._mode = mode.lower()
        return self.robot.set_robot_mode(resolve_robot_mode(mode))

    def get_robot_mode(self) -> str:
        getter = getattr(self.robot, "get_robot_mode", None)
        if callable(getter):
            mode = getter()
            self._mode = normalize_robot_mode(mode)
        return self._mode

    def get_sn(self) -> str:
        return self.robot.get_sn()

    def get_version(self) -> str:
        return self.robot.get_version()

    def reset(self) -> Any:
        return self.robot.reset()

    def play_audio(self, path: str, *, blocking: bool = False, timeout: Optional[float] = None) -> Any:
        action = self.robot.play_audio(path)
        return self._complete_action(action, blocking=blocking, timeout=timeout)

    def play_sound(
        self,
        sound_id: int,
        *,
        times: int = 1,
        blocking: bool = False,
        timeout: Optional[float] = None,
    ) -> Any:
        ensure_int_range("times", times, 1, 10)
        action = self.robot.play_sound(sound_id, times=times)
        return self._complete_action(action, blocking=blocking, timeout=timeout)

    def set_leds(
        self,
        r: int = 0,
        g: int = 0,
        b: int = 0,
        *,
        leds: str = "all",
        effect: str = "on",
    ) -> Any:
        ensure_int_range("r", r, 0, 255)
        ensure_int_range("g", g, 0, 255)
        ensure_int_range("b", b, 0, 255)
        ensure_choice("leds", leds, tuple(LED_COMPONENTS))
        ensure_choice("effect", effect, tuple(LED_EFFECTS))
        return self.robot.led.set_led(
            comp=LED_COMPONENTS[leds],
            r=r,
            g=g,
            b=b,
            effect=LED_EFFECTS[effect],
        )

    def stop(self) -> Any:
        self.robot.chassis.drive_speed(0.0, 0.0, 0.0)
        return self.robot.chassis.drive_wheels(0, 0, 0, 0)

    def stop_after(self, duration: float = 1.0, *, blocking: bool = False) -> Optional[Timer]:
        ensure_range("duration", duration, 0.0, 600.0, unit="seconds")
        if blocking:
            self.sleep(duration)
            self.stop()
            return None
        timer = Timer(duration, self.stop)
        timer.start()
        return timer

    def set_speed(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        *,
        timeout: Optional[float] = None,
    ) -> Any:
        ensure_range("x", x, -3.5, 3.5, unit="m/s")
        ensure_range("y", y, -3.5, 3.5, unit="m/s")
        ensure_range("z", z, -600.0, 600.0, unit="degrees/second")
        if timeout is not None:
            ensure_range("timeout", timeout, 0.0, 600.0, unit="seconds")
        return self.robot.chassis.drive_speed(x=x, y=y, z=z, timeout=timeout)

    def rotate(self, angular_velocity: float = 0.0, *, timeout: Optional[float] = None) -> Any:
        return self.set_speed(z=angular_velocity, timeout=timeout)

    def rotate_left(self, angular_velocity: float = 0.0, *, timeout: Optional[float] = None) -> Any:
        return self.rotate(angular_velocity=abs(angular_velocity), timeout=timeout)

    def rotate_right(self, angular_velocity: float = 0.0, *, timeout: Optional[float] = None) -> Any:
        return self.rotate(angular_velocity=-abs(angular_velocity), timeout=timeout)

    def turn(
        self,
        angle: float = 0.0,
        *,
        speed: float = 90.0,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        ensure_range("angle", angle, -1800.0, 1800.0, unit="degrees")
        ensure_range("speed", speed, 10.0, 540.0, unit="degrees/second")
        action = self.robot.chassis.move(z=angle, z_speed=speed)
        return self._complete_action(action, blocking=blocking, timeout=timeout)

    def turn_left(
        self,
        angle: float = 0.0,
        *,
        speed: float = 90.0,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        return self.turn(angle=abs(angle), speed=speed, blocking=blocking, timeout=timeout)

    def turn_right(
        self,
        angle: float = 0.0,
        *,
        speed: float = 90.0,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        return self.turn(angle=-abs(angle), speed=speed, blocking=blocking, timeout=timeout)

    def set_wheel_rpms(
        self,
        front_right: int = 0,
        front_left: int = 0,
        back_right: int = 0,
        back_left: int = 0,
        *,
        timeout: Optional[float] = None,
    ) -> Any:
        ensure_int_range("front_right", front_right, -1000, 1000, unit="rpm")
        ensure_int_range("front_left", front_left, -1000, 1000, unit="rpm")
        ensure_int_range("back_right", back_right, -1000, 1000, unit="rpm")
        ensure_int_range("back_left", back_left, -1000, 1000, unit="rpm")
        if timeout is not None:
            ensure_range("timeout", timeout, 0.0, 600.0, unit="seconds")
        return self.robot.chassis.drive_wheels(
            w1=front_right,
            w2=front_left,
            w3=back_right,
            w4=back_left,
            timeout=timeout,
        )

    def move(
        self,
        x: float = 0.0,
        y: float = 0.0,
        angle: float = 0.0,
        *,
        speed: float = 1.0,
        turn_speed: float = 90.0,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        ensure_range("x", x, -5.0, 5.0, unit="m")
        ensure_range("y", y, -5.0, 5.0, unit="m")
        ensure_range("angle", angle, -1800.0, 1800.0, unit="degrees")
        ensure_range("speed", speed, 0.5, 2.0, unit="m/s")
        ensure_range("turn_speed", turn_speed, 10.0, 540.0, unit="degrees/second")
        action = self.robot.chassis.move(x=x, y=y, z=angle, xy_speed=speed, z_speed=turn_speed)
        return self._complete_action(action, blocking=blocking, timeout=timeout)

    def forward(self, distance: float = 0.5, *, speed: float = 1.0, blocking: bool = True) -> Any:
        return self.move(x=distance, speed=speed, blocking=blocking)

    def back(self, distance: float = 0.5, *, speed: float = 1.0, blocking: bool = True) -> Any:
        return self.move(x=-distance, speed=speed, blocking=blocking)

    def backward(self, distance: float = 0.5, *, speed: float = 1.0, blocking: bool = True) -> Any:
        return self.back(distance=distance, speed=speed, blocking=blocking)

    def left(self, distance: float = 0.5, *, speed: float = 1.0, blocking: bool = True) -> Any:
        return self.move(y=-distance, speed=speed, blocking=blocking)

    def right(self, distance: float = 0.5, *, speed: float = 1.0, blocking: bool = True) -> Any:
        return self.move(y=distance, speed=speed, blocking=blocking)

    def circle(
        self,
        radius: float = 0.25,
        speed: float = 1.0,
        *,
        num_circles: int = 1,
        blocking: bool = True,
    ) -> Optional[Timer]:
        ensure_range("radius", radius, 0.05, 5.0, unit="m")
        ensure_range("speed", speed, 0.1, 2.0, unit="m/s")
        ensure_int_range("num_circles", num_circles, 1, 100)
        angular_velocity = speed / radius * RAD_TO_DEG
        duration = (2 * pi * radius / speed) * 1.5 * num_circles
        self.set_speed(x=speed, z=angular_velocity)
        return self.stop_after(duration, blocking=blocking)

    @deprecated_alias("get_module")
    def getModule(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_robot_mode")
    def setRobotMode(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("get_robot_mode")
    def getRobotMode(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("get_sn")
    def getSN(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("get_version")
    def getVersion(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("play_audio")
    def playAudio(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("play_sound")
    def playSound(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_leds")
    def setLEDs(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("stop_after")
    def stopAfter(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_speed")
    def setSpeed(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("rotate_left")
    def rotateLeft(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("rotate_right")
    def rotateRight(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("turn_left")
    def turnLeft(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("turn_right")
    def turnRight(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_wheel_rpms")
    def setWheelRPMs(self, *args: Any, **kwargs: Any) -> Any:
        return None
