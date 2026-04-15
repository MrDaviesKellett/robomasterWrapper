from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Optional


@dataclass
class FakeAction:
    name: str
    args: tuple[Any, ...] = field(default_factory=tuple)
    kwargs: dict[str, Any] = field(default_factory=dict)
    waited: bool = False
    timeout: Optional[float] = None

    def wait_for_completed(self, timeout: Optional[float] = None) -> "FakeAction":
        self.waited = True
        self.timeout = timeout
        return self


class _Recorder:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple[Any, ...], dict[str, Any]]] = []

    def record(self, name: str, *args: Any, **kwargs: Any) -> None:
        self.calls.append((name, args, kwargs))

    def last_call(self) -> tuple[str, tuple[Any, ...], dict[str, Any]]:
        return self.calls[-1]


class FakeChassis(_Recorder):
    def drive_speed(self, *args: Any, **kwargs: Any) -> bool:
        self.record("drive_speed", *args, **kwargs)
        return True

    def drive_wheels(self, *args: Any, **kwargs: Any) -> bool:
        self.record("drive_wheels", *args, **kwargs)
        return True

    def move(self, *args: Any, **kwargs: Any) -> FakeAction:
        self.record("move", *args, **kwargs)
        return FakeAction("chassis.move", args, kwargs)

    def set_pwm_value(self, **kwargs: Any) -> bool:
        self.record("set_pwm_value", **kwargs)
        return True

    def set_pwm_freq(self, **kwargs: Any) -> bool:
        self.record("set_pwm_freq", **kwargs)
        return True

    def sub_position(self, **kwargs: Any) -> bool:
        self.record("sub_position", **kwargs)
        return True

    def unsub_position(self) -> bool:
        self.record("unsub_position")
        return True

    def sub_attitude(self, **kwargs: Any) -> bool:
        self.record("sub_attitude", **kwargs)
        return True

    def unsub_attitude(self) -> bool:
        self.record("unsub_attitude")
        return True

    def sub_status(self, **kwargs: Any) -> bool:
        self.record("sub_status", **kwargs)
        return True

    def unsub_status(self) -> bool:
        self.record("unsub_status")
        return True

    def sub_imu(self, **kwargs: Any) -> bool:
        self.record("sub_imu", **kwargs)
        return True

    def unsub_imu(self) -> bool:
        self.record("unsub_imu")
        return True

    def sub_mode(self, **kwargs: Any) -> bool:
        self.record("sub_mode", **kwargs)
        return True

    def unsub_mode(self) -> bool:
        self.record("unsub_mode")
        return True

    def sub_esc(self, **kwargs: Any) -> bool:
        self.record("sub_esc", **kwargs)
        return True

    def unsub_esc(self) -> bool:
        self.record("unsub_esc")
        return True

    def sub_velocity(self, **kwargs: Any) -> bool:
        self.record("sub_velocity", **kwargs)
        return True

    def unsub_velocity(self) -> bool:
        self.record("unsub_velocity")
        return True


class FakeGimbal(_Recorder):
    def drive_speed(self, **kwargs: Any) -> bool:
        self.record("drive_speed", **kwargs)
        return True

    def move(self, **kwargs: Any) -> FakeAction:
        self.record("move", **kwargs)
        return FakeAction("gimbal.move", (), kwargs)

    def moveto(self, **kwargs: Any) -> FakeAction:
        self.record("moveto", **kwargs)
        return FakeAction("gimbal.moveto", (), kwargs)

    def recenter(self, **kwargs: Any) -> FakeAction:
        self.record("recenter", **kwargs)
        return FakeAction("gimbal.recenter", (), kwargs)

    def resume(self) -> bool:
        self.record("resume")
        return True

    def suspend(self) -> bool:
        self.record("suspend")
        return True

    def sub_angle(self, **kwargs: Any) -> bool:
        self.record("sub_angle", **kwargs)
        return True

    def unsub_angle(self) -> bool:
        self.record("unsub_angle")
        return True


class FakeBlaster(_Recorder):
    def fire(self, **kwargs: Any) -> bool:
        self.record("fire", **kwargs)
        return True

    def set_led(self, **kwargs: Any) -> bool:
        self.record("set_led", **kwargs)
        return True


class FakeLed(_Recorder):
    def set_led(self, **kwargs: Any) -> bool:
        self.record("set_led", **kwargs)
        return True


class FakeCameraModule(_Recorder):
    def __init__(self) -> None:
        super().__init__()
        self.next_image: Any = None

    def start_video_stream(self, **kwargs: Any) -> bool:
        self.record("start_video_stream", **kwargs)
        return True

    def stop_video_stream(self) -> bool:
        self.record("stop_video_stream")
        return True

    def read_cv2_image(self, **kwargs: Any) -> Any:
        self.record("read_cv2_image", **kwargs)
        return self.next_image

    def take_photo(self) -> bool:
        self.record("take_photo")
        return True

    def start_audio_stream(self) -> bool:
        self.record("start_audio_stream")
        return True

    def stop_audio_stream(self) -> bool:
        self.record("stop_audio_stream")
        return True

    def read_audio_frame(self, **kwargs: Any) -> bytes:
        self.record("read_audio_frame", **kwargs)
        return b"frame"

    def record_audio(self, **kwargs: Any) -> bool:
        self.record("record_audio", **kwargs)
        return True


class FakeVision(_Recorder):
    def sub_detect_info(self, mode: str, color: Optional[str], callback: Callable[..., Any]) -> bool:
        self.record("sub_detect_info", mode, color, callback=callback)
        self.mode = mode
        self.color = color
        self.callback = callback
        return True

    def unsub_detect_info(self, mode: str) -> bool:
        self.record("unsub_detect_info", mode)
        return True

    def reset(self) -> bool:
        self.record("reset")
        return True


class FakeRoboticArm(_Recorder):
    def move(self, **kwargs: Any) -> FakeAction:
        self.record("move", **kwargs)
        return FakeAction("arm.move", (), kwargs)

    def moveto(self, **kwargs: Any) -> FakeAction:
        self.record("moveto", **kwargs)
        return FakeAction("arm.moveto", (), kwargs)

    def recenter(self) -> FakeAction:
        self.record("recenter")
        return FakeAction("arm.recenter")

    def sub_position(self, **kwargs: Any) -> bool:
        self.record("sub_position", **kwargs)
        return True

    def unsub_position(self) -> bool:
        self.record("unsub_position")
        return True


class FakeGripperModule(_Recorder):
    def open(self, **kwargs: Any) -> bool:
        self.record("open", **kwargs)
        return True

    def close(self, **kwargs: Any) -> bool:
        self.record("close", **kwargs)
        return True

    def pause(self) -> bool:
        self.record("pause")
        return True

    def sub_status(self, **kwargs: Any) -> bool:
        self.record("sub_status", **kwargs)
        return True

    def unsub_status(self) -> bool:
        self.record("unsub_status")
        return True


class FakeBattery(_Recorder):
    def sub_battery_info(self, **kwargs: Any) -> bool:
        self.record("sub_battery_info", **kwargs)
        return True

    def unsub_battery_info(self) -> bool:
        self.record("unsub_battery_info")
        return True


class FakeServo(_Recorder):
    def moveto(self, **kwargs: Any) -> FakeAction:
        self.record("moveto", **kwargs)
        return FakeAction("servo.moveto", (), kwargs)

    def drive_speed(self, **kwargs: Any) -> bool:
        self.record("drive_speed", **kwargs)
        return True

    def pause(self, **kwargs: Any) -> bool:
        self.record("pause", **kwargs)
        return True

    def get_angle(self, **kwargs: Any) -> int:
        self.record("get_angle", **kwargs)
        return 42

    def sub_servo_info(self, **kwargs: Any) -> bool:
        self.record("sub_servo_info", **kwargs)
        return True

    def unsub_servo_info(self) -> bool:
        self.record("unsub_servo_info")
        return True


class FakeSensor(_Recorder):
    def sub_distance(self, **kwargs: Any) -> bool:
        self.record("sub_distance", **kwargs)
        return True

    def unsub_distance(self) -> bool:
        self.record("unsub_distance")
        return True


class FakeSensorAdaptor(_Recorder):
    def get_adc(self, **kwargs: Any) -> int:
        self.record("get_adc", **kwargs)
        return 123

    def get_io(self, **kwargs: Any) -> int:
        self.record("get_io", **kwargs)
        return 1

    def get_pulse_period(self, **kwargs: Any) -> int:
        self.record("get_pulse_period", **kwargs)
        return 10

    def sub_adapter(self, **kwargs: Any) -> bool:
        self.record("sub_adapter", **kwargs)
        return True

    def unsub_adapter(self) -> bool:
        self.record("unsub_adapter")
        return True


class FakeArmor(_Recorder):
    def sub_hit_event(self, **kwargs: Any) -> bool:
        self.record("sub_hit_event", **kwargs)
        return True

    def unsub_hit_event(self) -> bool:
        self.record("unsub_hit_event")
        return True

    def sub_ir_event(self, **kwargs: Any) -> bool:
        self.record("sub_ir_event", **kwargs)
        return True

    def unsub_ir_event(self) -> bool:
        self.record("unsub_ir_event")
        return True

    def set_hit_sensitivity(self, **kwargs: Any) -> bool:
        self.record("set_hit_sensitivity", **kwargs)
        return True


class FakeUart(_Recorder):
    def serial_param_set(self, **kwargs: Any) -> bool:
        self.record("serial_param_set", **kwargs)
        return True

    def serial_send_msg(self, payload: Any) -> bool:
        self.record("serial_send_msg", payload)
        return True

    def sub_serial_msg(self, callback: Callable[..., Any], args: tuple[Any, ...], kwargs: dict[str, Any]) -> bool:
        self.record("sub_serial_msg", callback, args, kwargs)
        return True

    def unsub_serial_msg(self) -> bool:
        self.record("unsub_serial_msg")
        return True


class FakeAiModule(_Recorder):
    def init_ai_module(self) -> bool:
        self.record("init_ai_module")
        return True

    def sub_ai_event(self, **kwargs: Any) -> bool:
        self.record("sub_ai_event", **kwargs)
        return True

    def unsub_ai_event(self) -> bool:
        self.record("unsub_ai_event")
        return True


class FakeRobot:
    def __init__(self) -> None:
        self.chassis = FakeChassis()
        self.gimbal = FakeGimbal()
        self.blaster = FakeBlaster()
        self.led = FakeLed()
        self.camera = FakeCameraModule()
        self.vision = FakeVision()
        self.robotic_arm = FakeRoboticArm()
        self.gripper = FakeGripperModule()
        self.battery = FakeBattery()
        self.servo = FakeServo()
        self.sensor = FakeSensor()
        self.sensor_adaptor = FakeSensorAdaptor()
        self.armor = FakeArmor()
        self.uart = FakeUart()
        self.ai_module = FakeAiModule()
        self.dds = object()
        self._mode = "free"
        self.calls: list[tuple[str, tuple[Any, ...], dict[str, Any]]] = []

    def initialize(self, **kwargs: Any) -> bool:
        self.calls.append(("initialize", (), kwargs))
        return True

    def close(self) -> bool:
        self.calls.append(("close", (), {}))
        return True

    def set_robot_mode(self, mode: str) -> bool:
        self._mode = mode
        self.calls.append(("set_robot_mode", (mode,), {}))
        return True

    def get_robot_mode(self) -> str:
        return self._mode

    def get_sn(self) -> str:
        return "RM-FAKE-001"

    def get_version(self) -> str:
        return "0.0.0"

    def reset(self) -> bool:
        self.calls.append(("reset", (), {}))
        return True

    def play_audio(self, path: str) -> FakeAction:
        self.calls.append(("play_audio", (path,), {}))
        return FakeAction("play_audio", (path,), {})

    def play_sound(self, sound_id: int, times: int = 1) -> FakeAction:
        self.calls.append(("play_sound", (sound_id,), {"times": times}))
        return FakeAction("play_sound", (sound_id,), {"times": times})
