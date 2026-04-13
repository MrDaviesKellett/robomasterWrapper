from __future__ import annotations

from typing import Any, Callable, Optional

try:  # pragma: no cover - optional dependency in local tests
    import cv2
except ImportError:  # pragma: no cover - optional dependency in local tests
    cv2 = None

try:  # pragma: no cover - optional dependency in local tests
    from simple_pid import PID
except ImportError:  # pragma: no cover - exercised in tests without the dependency
    class PID:  # type: ignore[override]
        def __init__(self, p: float, i: float, d: float, *, setpoint: float = 0.0, sample_time: float = 0.05) -> None:
            self.Kp = p
            self.Ki = i
            self.Kd = d
            self.setpoint = setpoint
            self.sample_time = sample_time
            self.output_limits = (None, None)

        def __call__(self, measurement: float) -> float:
            error = self.setpoint - measurement
            output = self.Kp * error
            lower, upper = self.output_limits
            if lower is not None:
                output = max(lower, output)
            if upper is not None:
                output = min(upper, output)
            return output

from .helperFuncs import (
    deprecated_alias,
    ensure_choice,
    ensure_color,
    ensure_range,
)


class Camera:
    def __init__(self, robomaster: Any) -> None:
        self.robomaster = robomaster
        self.robot = self.robomaster.robot
        self.camera = self.robot.camera
        self.vision = self.robot.vision
        self.streaming = False
        self.audio_streaming = False
        self.detecting = False
        self.detect_mode = "line"
        self.vision_debug = False
        self.debug_color = (0, 255, 0)
        self.resolution = "360p"
        self.width = 640
        self.height = 360
        self.debug_mode: Any = False
        self.frequency = 30
        self.follow_speed = 0.5
        self.follow_distance = 0.5
        self.at_marker = False
        self._debug_items: list[dict[str, Any]] = []
        self.pid = PID(-330, 0, -28, setpoint=0.0, sample_time=1.0 / self.frequency)
        self.pid.output_limits = (-self.follow_speed / 3.5 * 600, self.follow_speed / 3.5 * 600)

    def _require_cv2(self) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV is required for Camera.view(). Install 'opencv-python'.")

    def _update_resolution_dimensions(self) -> None:
        if self.resolution == "360p":
            self.width, self.height = 640, 360
        elif self.resolution == "540p":
            self.width, self.height = 960, 540
        else:
            self.width, self.height = 1280, 720

    def set_resolution(self, resolution: str = "360p") -> None:
        resolution = resolution.lower()
        ensure_choice("resolution", resolution, ("360p", "540p", "720p"))
        self.resolution = resolution
        self._update_resolution_dimensions()

    def start(self) -> Any:
        if self.streaming:
            return True
        result = self.camera.start_video_stream(display=False, resolution=self.resolution)
        self.streaming = bool(result is not False)
        return result

    def stop_detect(self) -> None:
        if self.detecting:
            self.vision.unsub_detect_info(self.detect_mode)
            self.detecting = False

    def stop(self) -> Any:
        self.stop_detect()
        if self.audio_streaming:
            self.stop_audio_stream()
        if not self.streaming:
            return True
        result = self.camera.stop_video_stream()
        self.streaming = False
        if cv2 is not None:
            cv2.destroyAllWindows()
        return result

    def view(self) -> Any:
        self._require_cv2()
        if not self.streaming:
            self.start()
        image = self.camera.read_cv2_image(strategy="newest")
        if image is None:
            return None
        for item in self._debug_items:
            if item["type"] == "box":
                cv2.rectangle(image, item["start"], item["end"], item["color"], 2)
            elif item["type"] == "point":
                cv2.circle(image, item["point"], 2, item["color"], -1)
        cv2.imshow("RoboMaster", image)
        cv2.waitKey(1)
        return image

    def set_pid(self, p: float = 330, i: float = 0, d: float = 28) -> None:
        self.pid.Kp = -p
        self.pid.Ki = -i
        self.pid.Kd = -d

    def set_detect_mode(self, mode: str = "line") -> None:
        ensure_choice("mode", mode.lower(), ("person", "gesture", "line", "marker", "robot"))
        self.detect_mode = mode.lower()

    def set_vision_debug(self, value: bool = True) -> None:
        self.vision_debug = bool(value)

    def set_debug_color(self, color: tuple[int, int, int] = (255, 0, 0)) -> None:
        self.debug_color = ensure_color("color", color)

    def reset_vision(self) -> Any:
        self.stop_detect()
        self._debug_items = []
        return self.vision.reset()

    def _store_boxes(self, items: list[list[Any]]) -> None:
        boxes: list[dict[str, Any]] = []
        for item in items:
            x = int(item[0] * self.width)
            y = int(item[1] * self.height)
            w = int(item[2] * self.width)
            h = int(item[3] * self.height)
            boxes.append(
                {
                    "type": "box",
                    "start": (x - w // 2, y - h // 2),
                    "end": (x + w // 2, y + h // 2),
                    "color": self.debug_color,
                }
            )
        self._debug_items = boxes

    def _store_line(self, info: list[Any]) -> None:
        points: list[dict[str, Any]] = []
        for point in info[1:]:
            x = int(point[0] * self.width)
            y = int(point[1] * self.height)
            points.append({"type": "point", "point": (x, y), "color": self.debug_color})
        self._debug_items = points

    def _detect_callback(self, info: Any) -> bool:
        if not info:
            self._debug_items = []
            return False
        if self.detect_mode in {"person", "gesture", "marker", "robot"}:
            self._store_boxes(info)
        elif self.detect_mode == "line":
            self._store_line(info)
        return True

    def _restart_detection(self, mode: str, callback: Callable[..., Any], color: Optional[str] = None) -> Any:
        ensure_choice("mode", mode, ("person", "gesture", "line", "marker", "robot"))
        if color is not None:
            ensure_choice("color", color, ("red", "green", "blue"))
        if not self.streaming:
            self.start()
        if self.detecting:
            self.stop_detect()
        self.detect_mode = mode
        result = self.vision.sub_detect_info(mode, color, callback)
        self.detecting = bool(result is not False)
        return result

    def detect(self, name: Optional[str] = None, color: Optional[str] = None) -> Any:
        mode = self.detect_mode if name is None else name.lower()
        return self._restart_detection(mode, self._detect_callback, color=color)

    def detect_person(self) -> Any:
        return self.detect("person")

    def detect_gesture(self) -> Any:
        return self.detect("gesture")

    def detect_line(self, color: str = "red") -> Any:
        return self.detect("line", color=color)

    def detect_marker(self, color: str = "red") -> Any:
        return self.detect("marker", color=color)

    def detect_robot(self) -> Any:
        return self.detect("robot")

    def set_follow_speed(self, speed: float) -> None:
        ensure_range("speed", speed, 0.05, 3.5, unit="m/s")
        self.follow_speed = speed
        self.pid.output_limits = (-self.follow_speed / 3.5 * 600, self.follow_speed / 3.5 * 600)

    def set_follow_distance(self, distance: float) -> None:
        ensure_range("distance", distance, 0.0, 1.0)
        self.follow_distance = distance

    def _follow_callback(self, info: Any) -> bool:
        self._detect_callback(info)
        if self.detect_mode != "line":
            return False
        if info == [0] or not info:
            self.robomaster.stop()
            return False
        follow_point = min(5, len(info) - 2)
        tangent_angle = info[follow_point + 1][2]
        turn_speed = self.pid(tangent_angle)
        self.robomaster.set_speed(x=self.follow_speed, z=turn_speed)
        return True

    def follow(self, name: Optional[str] = None, color: str = "red") -> Any:
        mode = self.detect_mode if name is None else name.lower()
        if mode != "line":
            raise NotImplementedError("Only line following is implemented in the student wrapper.")
        return self._restart_detection(mode, self._follow_callback, color=color)

    def follow_line(self, color: str = "red") -> Any:
        return self.follow("line", color=color)

    def move_to_marker(
        self,
        marker_type: str = "1",
        *,
        color: str = "red",
        error: float = 0.06,
        speed: float = 1.0,
        min_speed: float = 0.02,
        target_x: float = 0.0,
        target_y: float = 0.5,
    ) -> Any:
        ensure_choice("color", color, ("red", "green", "blue"))
        ensure_range("error", error, 0.0, 1.0)
        ensure_range("speed", speed, 0.05, 3.5, unit="m/s")
        ensure_range("min_speed", min_speed, 0.0, 1.0, unit="m/s")
        ensure_range("target_x", target_x, -1.0, 1.0)
        ensure_range("target_y", target_y, -1.0, 1.0)
        marker_type = str(marker_type)
        self.at_marker = False

        def _callback(info: Any) -> bool:
            self._detect_callback(info)
            if not info:
                return False
            for marker in info:
                if str(marker[4]) != marker_type:
                    continue
                x = marker[0] * 2 - 1
                y = marker[1] * 2 - 1
                if abs(x - target_x) > error:
                    lateral = (x - target_x) * speed
                    if 0 < abs(lateral) < min_speed:
                        lateral = min_speed if lateral > 0 else -min_speed
                    self.robomaster.set_speed(y=lateral)
                    return False
                if abs(y - target_y) > error:
                    forward = (target_y - y) * speed
                    if 0 < abs(forward) < min_speed:
                        forward = min_speed if forward > 0 else -min_speed
                    self.robomaster.set_speed(x=forward)
                    return False
                self.robomaster.stop()
                self.stop_detect()
                self.at_marker = True
                return True
            return False

        return self._restart_detection("marker", _callback, color=color)

    def take_photo(self) -> Any:
        return self.camera.take_photo()

    def start_audio_stream(self) -> Any:
        result = self.camera.start_audio_stream()
        self.audio_streaming = bool(result is not False)
        return result

    def stop_audio_stream(self) -> Any:
        result = self.camera.stop_audio_stream()
        self.audio_streaming = False
        return result

    def read_audio_frame(self, timeout: float = 1.0) -> Any:
        ensure_range("timeout", timeout, 0.01, 60.0, unit="seconds")
        return self.camera.read_audio_frame(timeout=timeout)

    def record_audio(self, save_file: str = "output.wav", seconds: float = 5.0, sample_rate: int = 48000) -> Any:
        ensure_range("seconds", seconds, 0.1, 600.0, unit="seconds")
        ensure_choice("sample_rate", sample_rate, (8000, 16000, 32000, 44100, 48000))
        return self.camera.record_audio(save_file=save_file, seconds=seconds, sample_rate=sample_rate)

    @deprecated_alias("set_resolution")
    def setResolution(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("stop_detect")
    def stopDetect(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_pid")
    def setPID(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_detect_mode")
    def setDetectMode(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_vision_debug")
    def setVisionDebug(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_debug_color")
    def setDebugColor(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("detect_person")
    def detectPerson(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("detect_gesture")
    def detectGesture(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("detect_line")
    def detectLine(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("detect_marker")
    def detectMarker(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("detect_robot")
    def detectRobot(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_follow_speed")
    def setFollowSpeed(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("set_follow_distance")
    def setFollowDistance(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("follow_line")
    def followLine(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("move_to_marker")
    def moveToMarker(self, *args: Any, **kwargs: Any) -> Any:
        return None

    @deprecated_alias("view")
    def display(self, *args: Any, **kwargs: Any) -> Any:
        return None
