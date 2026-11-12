from __future__ import annotations

from math import isfinite, pi
from threading import Event, Lock, Thread, Timer
from time import monotonic, sleep
from typing import Any, Optional

from ._sdk import LED_COMPONENTS, LED_EFFECTS, QuietSdkLogs, create_sdk_robot, normalize_robot_mode, resolve_robot_mode
from .arm import Arm, Gripper
from .camera import Camera
from .gun import Gun
from .helperFuncs import (
    ensure_choice,
    ensure_int_range,
    ensure_range,
    wait_for_action,
)
from .modules import AiModule, Armor, Battery, Chassis, DistanceSensor, SensorAdaptor, Servo, Uart

RAD_TO_DEG = 180.0 / pi
_UNSET = object()


class RoboMaster:
    def __init__(
        self,
        conn_type: str = "ap",
        protocol: str = "udp",
        serial: Optional[str] = None,
        *,
        auto_reset: bool = False,
        default_mode: str = "chassis",
        connect_retries: int = 2,
        connect_retry_delay: float = 1.0,
        connect_timeout: Optional[float] = None,
        _sdk_robot: Optional[Any] = None,
        _sleep=sleep,
    ) -> None:
        self.sleep = _sleep
        self.robot = _sdk_robot if _sdk_robot is not None else create_sdk_robot()
        self._mode = "free"
        self.default_move_speed = 1.0
        self.default_turn_speed = 90.0

        connection = self._normalize_connection_type(conn_type)
        ensure_choice("protocol", protocol.lower(), ("tcp", "udp"))
        ensure_int_range("connect_retries", connect_retries, 0, 20)
        ensure_range("connect_retry_delay", connect_retry_delay, 0.0, 30.0, unit="seconds")

        if connect_timeout is not None:
            ensure_range("connect_timeout", connect_timeout, 0.001, float("inf"), unit="seconds")
            if not isfinite(connect_timeout):
                raise ValueError("connect_timeout must be finite.")

        self._connect(connection, protocol.lower(), serial, connect_retries,
                      connect_retry_delay, connect_timeout)

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

        # initialize() already resets the SDK modules. auto_reset is retained
        # for constructor compatibility; explicit reset() remains available.
        self.set_robot_mode(default_mode)

    def _connect(self, connection, protocol, serial, retries, retry_delay, timeout):
        # SDK initialize() has no timeout hook. Keep its blocking discovery off
        # the caller thread, and close late connections after a timeout.
        from ._sdk import connection_target

        target = connection_target(connection, serial)
        print(f"Connecting to robot via {connection} on {target} ...", flush=True)
        done = Event()
        cancelled = Event()
        lock = Lock()
        result = False
        error = None
        # The SDK prints ~8 raw ERROR lines per failed attempt. Hide them while
        # connecting and fold the most useful one into our RuntimeError.
        quiet = QuietSdkLogs()
        quiet.start()

        def initialize():
            nonlocal result, error
            try:
                for attempt in range(retries + 1):
                    if cancelled.is_set():
                        break
                    try:
                        result = self.robot.initialize(conn_type=connection, proto_type=protocol, sn=serial)
                    except Exception as exc:
                        result = False
                        error = exc
                    if result is not False or cancelled.is_set():
                        break
                    if attempt < retries:
                        if self.sleep is sleep:
                            cancelled.wait(retry_delay)
                        else:
                            self.sleep(retry_delay)
            finally:
                with lock:
                    abandoned = cancelled.is_set()
                    if not abandoned:
                        quiet.stop()
                    done.set()
                if abandoned:
                    try:
                        self.robot.close()
                    except Exception:
                        pass
                    finally:
                        quiet.stop()

        started = monotonic()
        deadline = None if timeout is None else started + timeout
        heartbeat = started + 10.0
        Thread(target=initialize, name="robowrap-connect", daemon=True).start()
        while True:
            now = monotonic()
            wake = heartbeat if deadline is None else min(heartbeat, deadline)
            if done.wait(max(0.0, wake - now)):
                break
            now = monotonic()
            if deadline is not None and now >= deadline:
                with lock:
                    if done.is_set():
                        break
                    cancelled.set()
                break
            print(f"Still connecting to robot via {connection} on {target} ... "
                  f"({now - started:.0f} seconds elapsed)", flush=True)
            heartbeat += 10.0
        if cancelled.is_set() or result is False:
            if cancelled.is_set():
                error = TimeoutError(f"connect_timeout of {timeout:g} seconds expired.")
            message = "Robot could not be connected. Check power, network mode, and serial number."
            if error is not None:
                message = f"{message} Last error: {error}"
            detail = quiet.detail()
            if detail is not None:
                message = f"{message} {detail}"
            raise RuntimeError(message)

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
        ensure_choice("conn_type", conn_type.lower(), tuple(mapping))
        return mapping[conn_type.lower()]

    def _complete_action(self, action: Any, *, blocking: bool = True, timeout: Optional[float] = None) -> Any:
        return wait_for_action(action, blocking=blocking, timeout=timeout)

    def __enter__(self) -> "RoboMaster":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        # The SDK keeps non-daemon threads alive until close(), so a script
        # that skips it hangs at exit. Keep the body's error if close fails too.
        try:
            self.close()
        except Exception:
            if exc_type is None:
                raise
        return False

    def close(self) -> Any:
        # Always reach robot.close(), even if stopping motion or the camera fails.
        errors = []
        for step in (self.stop, self.cam.stop):
            try:
                step()
            except Exception as exc:
                errors.append(exc)
        result = self.robot.close()
        if errors:
            raise errors[0]
        return result

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

    def set_default_speed(
        self,
        speed: Optional[float] = None,
        turn_speed: Optional[float] = None,
    ) -> None:
        if speed is not None:
            self.default_move_speed = ensure_range("speed", speed, 0.5, 2.0, unit="m/s")
        if turn_speed is not None:
            self.default_turn_speed = ensure_range("turn_speed", turn_speed, 10.0, 540.0, unit="degrees/second")

    def get_default_speed(self) -> tuple[float, float]:
        return self.default_move_speed, self.default_turn_speed

    def _move_speed(self, speed: Optional[float]) -> float:
        return self.default_move_speed if speed is None else speed

    def _turn_speed(self, speed: Optional[float]) -> float:
        return self.default_turn_speed if speed is None else speed

    def set_speed(self, *args: Any, **kwargs: Any) -> Any:
        raise ValueError(
            "set_speed is ambiguous: use set_default_speed(speed=..., turn_speed=...) "
            "to change the default speed for future move commands, or use "
            "continuous_drive(x=..., y=..., z=..., timeout=...) to start driving "
            "continuously now."
        )

    def continuous_drive(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        *,
        timeout: Optional[float] = None,
    ) -> Any:
        # Hardware-capture pending: DJI examples use x forward, y right, z clockwise.
        ensure_range("x", x, -3.5, 3.5, unit="m/s")
        ensure_range("y", y, -3.5, 3.5, unit="m/s")
        ensure_range("z", z, -600.0, 600.0, unit="degrees/second")
        if timeout is not None:
            ensure_range("timeout", timeout, 0.0, 600.0, unit="seconds")
        self.robot.chassis.drive_speed(x=x, y=y, z=z, timeout=timeout)
        return None

    def rotate(self, angular_velocity: float = 0.0, *, timeout: Optional[float] = None) -> Any:
        """Rotate continuously; positive velocity is clockwise (DJI drive_speed)."""
        return self.continuous_drive(z=angular_velocity, timeout=timeout)

    def rotate_left(self, angular_velocity: float = 0.0, *, timeout: Optional[float] = None) -> Any:
        return self.rotate(angular_velocity=-abs(angular_velocity), timeout=timeout)

    def rotate_right(self, angular_velocity: float = 0.0, *, timeout: Optional[float] = None) -> Any:
        return self.rotate(angular_velocity=abs(angular_velocity), timeout=timeout)

    def turn(
        self,
        angle: float = 0.0,
        *,
        speed: Optional[float] = None,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        speed = self._turn_speed(speed)
        ensure_range("angle", angle, -1800.0, 1800.0, unit="degrees")
        ensure_range("speed", speed, 10.0, 540.0, unit="degrees/second")
        action = self.robot.chassis.move(z=angle, z_speed=speed)
        return self._complete_action(action, blocking=blocking, timeout=timeout)

    def turn_left(
        self,
        angle: float = 0.0,
        *,
        speed: Optional[float] = None,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        return self.turn(angle=abs(angle), speed=speed, blocking=blocking, timeout=timeout)

    def turn_right(
        self,
        angle: float = 0.0,
        *,
        speed: Optional[float] = None,
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
            w3=back_left,
            w4=back_right,
            timeout=timeout,
        )

    def move(
        self,
        x: float = _UNSET,
        y: float = _UNSET,
        angle: float = _UNSET,
        *,
        speed: Optional[float] = None,
        turn_speed: Optional[float] = None,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        if x is _UNSET and y is _UNSET and angle is _UNSET:
            raise ValueError(
                "move() requires x, y, or angle. Accepted arguments: x, y, angle, "
                "speed, turn_speed, blocking, timeout. Use move(x=0, y=0) for an intentional stop."
            )
        x = 0.0 if x is _UNSET else x
        y = 0.0 if y is _UNSET else y
        angle = 0.0 if angle is _UNSET else angle
        speed = self._move_speed(speed)
        turn_speed = self._turn_speed(turn_speed)
        ensure_range("x", x, -5.0, 5.0, unit="m")
        ensure_range("y", y, -5.0, 5.0, unit="m")
        ensure_range("angle", angle, -1800.0, 1800.0, unit="degrees")
        ensure_range("speed", speed, 0.5, 2.0, unit="m/s")
        ensure_range("turn_speed", turn_speed, 10.0, 540.0, unit="degrees/second")
        action = self.robot.chassis.move(x=x, y=y, z=angle, xy_speed=speed, z_speed=turn_speed)
        return self._complete_action(action, blocking=blocking, timeout=timeout)

    def forward(self, distance: float = 0.5, *, speed: Optional[float] = None, blocking: bool = True) -> Any:
        return self.move(x=distance, speed=speed, blocking=blocking)

    def back(self, distance: float = 0.5, *, speed: Optional[float] = None, blocking: bool = True) -> Any:
        return self.move(x=-distance, speed=speed, blocking=blocking)

    def backward(self, distance: float = 0.5, *, speed: Optional[float] = None, blocking: bool = True) -> Any:
        return self.back(distance=distance, speed=speed, blocking=blocking)

    def left(self, distance: float = 0.5, *, speed: Optional[float] = None, blocking: bool = True) -> Any:
        return self.move(y=-distance, speed=speed, blocking=blocking)

    def right(self, distance: float = 0.5, *, speed: Optional[float] = None, blocking: bool = True) -> Any:
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
        angular_velocity = -speed / radius * RAD_TO_DEG
        duration = (2 * pi * radius / speed) * num_circles
        self.continuous_drive(x=speed, z=angular_velocity)
        return self.stop_after(duration, blocking=blocking)
