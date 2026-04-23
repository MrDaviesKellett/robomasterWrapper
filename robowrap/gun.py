from __future__ import annotations

from typing import Any, Callable, Optional

from ._sdk import LED_COMPONENTS, LED_EFFECTS
from .helperFuncs import deprecated_alias, ensure_choice, ensure_int_range, ensure_range


class Gun:
    def __init__(self, robomaster: Any) -> None:
        self.robomaster = robomaster
        self.robot = self.robomaster.robot

    def set_leds(
        self,
        r: int = 0,
        g: int = 0,
        b: int = 0,
        *,
        leds: str = "gun",
        effect: str = "on",
    ) -> Any:
        ensure_int_range("r", r, 0, 255)
        ensure_int_range("g", g, 0, 255)
        ensure_int_range("b", b, 0, 255)
        ensure_choice("leds", leds, ("gun", "gun_left", "gun_right"))
        ensure_choice("effect", effect, tuple(LED_EFFECTS))
        return self.robot.led.set_led(
            comp=LED_COMPONENTS[leds],
            r=r,
            g=g,
            b=b,
            effect=LED_EFFECTS[effect],
        )

    def rotate(self, pitch_speed: float = 0.0, yaw_speed: float = 0.0) -> Any:
        ensure_range("pitch_speed", pitch_speed, -360.0, 360.0, unit="degrees/second")
        ensure_range("yaw_speed", yaw_speed, -360.0, 360.0, unit="degrees/second")
        return self.robot.gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)

    def move(
        self,
        pitch: float = 0.0,
        yaw: float = 0.0,
        *,
        pitch_speed: float = 50.0,
        yaw_speed: float = 50.0,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        ensure_range("pitch", pitch, -55.0, 55.0, unit="degrees")
        ensure_range("yaw", yaw, -55.0, 55.0, unit="degrees")
        ensure_range("pitch_speed", pitch_speed, 0.0, 540.0, unit="degrees/second")
        ensure_range("yaw_speed", yaw_speed, 0.0, 540.0, unit="degrees/second")
        action = self.robot.gimbal.move(
            pitch=pitch,
            yaw=yaw,
            pitch_speed=pitch_speed,
            yaw_speed=yaw_speed,
        )
        return self.robomaster._complete_action(action, blocking=blocking, timeout=timeout)

    def move_to(
        self,
        pitch: float = 0.0,
        yaw: float = 0.0,
        *,
        pitch_speed: float = 50.0,
        yaw_speed: float = 50.0,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        ensure_range("pitch", pitch, -25.0, 30.0, unit="degrees")
        ensure_range("yaw", yaw, -250.0, 250.0, unit="degrees")
        ensure_range("pitch_speed", pitch_speed, 0.0, 540.0, unit="degrees/second")
        ensure_range("yaw_speed", yaw_speed, 0.0, 540.0, unit="degrees/second")
        action = self.robot.gimbal.moveto(
            pitch=pitch,
            yaw=yaw,
            pitch_speed=pitch_speed,
            yaw_speed=yaw_speed,
        )
        return self.robomaster._complete_action(action, blocking=blocking, timeout=timeout)

    def recenter(
        self,
        *,
        pitch_speed: float = 100.0,
        yaw_speed: float = 100.0,
        blocking: bool = True,
        timeout: Optional[float] = 8.0,
    ) -> Any:
        ensure_range("pitch_speed", pitch_speed, 0.0, 540.0, unit="degrees/second")
        ensure_range("yaw_speed", yaw_speed, 0.0, 540.0, unit="degrees/second")
        action = self.robot.gimbal.recenter(pitch_speed=pitch_speed, yaw_speed=yaw_speed)
        result = self.robomaster._complete_action(action, blocking=blocking, timeout=timeout)
        if not blocking or result is not False:
            return result

        # Some firmware/Wi-Fi combinations never report completion for gimbal.recenter().
        fallback_action = self.robot.gimbal.moveto(
            pitch=0.0,
            yaw=0.0,
            pitch_speed=pitch_speed,
            yaw_speed=yaw_speed,
        )
        return self.robomaster._complete_action(fallback_action, blocking=blocking, timeout=timeout)

    def resume(self) -> Any:
        return self.robot.gimbal.resume()

    def suspend(self) -> Any:
        return self.robot.gimbal.suspend()

    def fire(self, fire_type: str = "ir", times: int = 1) -> Any:
        ensure_choice("fire_type", fire_type, ("ir", "water"))
        ensure_int_range("times", times, 1, 8)
        return self.robot.blaster.fire(fire_type=fire_type, times=times)

    def set_led(self, brightness: int = 100, effect: str = "on") -> Any:
        ensure_int_range("brightness", brightness, 0, 255)
        ensure_choice("effect", effect, ("on", "off"))
        return self.robot.blaster.set_led(brightness=brightness, effect=effect)

    def subscribe_angle(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.robot.gimbal.sub_angle(freq=freq, callback=callback)

    def unsubscribe_angle(self) -> Any:
        return self.robot.gimbal.unsub_angle()

    @deprecated_alias("set_leds")
    def setLEDs(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("move_to")
    def moveto(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_led")
    def setLED(self, *args: Any, **kwargs: Any) -> Any:
        return None
