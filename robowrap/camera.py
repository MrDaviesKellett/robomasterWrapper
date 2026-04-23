from __future__ import annotations

import importlib.util
import multiprocessing
import queue
import threading
import time
from typing import Any, Callable, Optional

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
from .viewer import run_qt_viewer


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
        self._line_follow_profiles: dict[str, dict[str, float]] = {
            "classroom": {"kp": 230.0, "ki": 0.0, "kd": 35.0, "heading_weight": 0.35, "base_speed": 0.28},
            "balanced": {"kp": 260.0, "ki": 0.0, "kd": 40.0, "heading_weight": 0.45, "base_speed": 0.4},
            "sport": {"kp": 300.0, "ki": 0.0, "kd": 45.0, "heading_weight": 0.55, "base_speed": 0.58},
        }
        self._line_follow_speed_presets: dict[str, float] = {"slow": 0.25, "medium": 0.4, "fast": 0.6}
        self._line_follow_target_presets: dict[str, float] = {"left": 0.4, "center": 0.5, "right": 0.6}
        self._line_follow_config: dict[str, Any] = {
            "profile": "classroom",
            "target_x": 0.5,
            "lookahead": 3,
            "stop_on_lost": True,
            "base_speed": 0.28,
            "min_speed": 0.08,
            "max_turn_speed": 220.0,
            "heading_weight": 0.35,
            "speed_slowdown_gain": 0.7,
            "curvature_slowdown_gain": 0.45,
            "lost_hold_seconds": 0.35,
            "stop_after_lost_seconds": 1.2,
            "search_turn_speed": 65.0,
            "kp": 230.0,
            "ki": 0.0,
            "kd": 35.0,
            "invert_turn": False,
        }
        self._line_follow_state: dict[str, Any] = {
            "active": False,
            "line_visible": False,
            "line_type": None,
            "last_error": 0.0,
            "last_turn_speed": 0.0,
            "last_speed": 0.0,
            "lost_seconds": 0.0,
            "last_line_time": None,
            "last_update_time": None,
        }
        self._line_integral = 0.0
        self._debug_items: list[dict[str, Any]] = []
        self._viewer_queue: Optional[Any] = None
        self._viewer_process: Optional[multiprocessing.Process] = None
        self._viewer_thread: Optional[threading.Thread] = None
        self._viewer_stop = threading.Event()
        self._consecutive_frame_failures = 0
        self.pid = PID(-330, 0, -28, setpoint=0.0, sample_time=1.0 / self.frequency)
        self.pid.output_limits = (-self.follow_speed / 3.5 * 600, self.follow_speed / 3.5 * 600)

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
        self._consecutive_frame_failures = 0
        return result

    def stop_detect(self) -> None:
        if self.detecting:
            self.vision.unsub_detect_info(self.detect_mode)
            self.detecting = False
        self._reset_line_follow_runtime()

    def _reset_line_follow_runtime(self) -> None:
        self._line_integral = 0.0
        self._line_follow_state["active"] = False
        self._line_follow_state["line_visible"] = False
        self._line_follow_state["line_type"] = None
        self._line_follow_state["last_error"] = 0.0
        self._line_follow_state["last_turn_speed"] = 0.0
        self._line_follow_state["last_speed"] = 0.0
        self._line_follow_state["lost_seconds"] = 0.0
        self._line_follow_state["last_line_time"] = None
        self._line_follow_state["last_update_time"] = None

    def stop(self) -> Any:
        self.stop_detect()
        self.stop_view()
        if self.audio_streaming:
            self.stop_audio_stream()
        if not self.streaming:
            return True
        result = self.camera.stop_video_stream()
        self.streaming = False
        self._consecutive_frame_failures = 0
        return result

    def _recover_video_stream(self) -> bool:
        if self.streaming:
            try:
                self.camera.stop_video_stream()
            except Exception:
                pass
            self.streaming = False
        result = self.start()
        return bool(result is not False)

    def _draw_box(self, image: Any, start: tuple[int, int], end: tuple[int, int], color: tuple[int, int, int]) -> None:
        try:
            height, width = image.shape[:2]
            x1 = max(0, min(width - 1, start[0]))
            y1 = max(0, min(height - 1, start[1]))
            x2 = max(0, min(width - 1, end[0]))
            y2 = max(0, min(height - 1, end[1]))
            if x2 <= x1 or y2 <= y1:
                return
            image[y1 : min(y1 + 2, height), x1:x2] = color
            image[max(y2 - 2, 0) : y2, x1:x2] = color
            image[y1:y2, x1 : min(x1 + 2, width)] = color
            image[y1:y2, max(x2 - 2, 0) : x2] = color
        except Exception:
            return

    def _draw_point(self, image: Any, point: tuple[int, int], color: tuple[int, int, int]) -> None:
        try:
            height, width = image.shape[:2]
            x = max(0, min(width - 1, point[0]))
            y = max(0, min(height - 1, point[1]))
            image[max(y - 2, 0) : min(y + 3, height), max(x - 2, 0) : min(x + 3, width)] = color
        except Exception:
            return

    def _draw_debug_items(self, image: Any) -> None:
        for item in self._debug_items:
            if item["type"] == "box":
                self._draw_box(image, item["start"], item["end"], item["color"])
            elif item["type"] == "point":
                self._draw_point(image, item["point"], item["color"])

    def frame(self, strategy: str = "newest", *, draw_debug: bool = True) -> Any:
        if not self.streaming:
            self.start()
        try:
            image = self.camera.read_cv2_image(strategy=strategy)
        except Exception:
            image = None
        if image is None:
            self._consecutive_frame_failures += 1
            if self._consecutive_frame_failures >= 3 and self._recover_video_stream():
                try:
                    image = self.camera.read_cv2_image(strategy=strategy)
                except Exception:
                    image = None
                self._consecutive_frame_failures = 0 if image is not None else self._consecutive_frame_failures
            if image is None:
                return None
        self._consecutive_frame_failures = 0
        if draw_debug:
            self._draw_debug_items(image)
        return image

    def _viewer_worker(self, fps: int) -> None:
        interval = 1.0 / fps
        while not self._viewer_stop.is_set():
            if self._viewer_process is not None and not self._viewer_process.is_alive():
                break
            image = self.frame(draw_debug=True)
            if image is not None and self._viewer_queue is not None:
                while True:
                    try:
                        self._viewer_queue.get_nowait()
                    except queue.Empty:
                        break
                try:
                    self._viewer_queue.put_nowait(image)
                except queue.Full:
                    pass
            time.sleep(interval)

    def view(self, *, fps: int = 30, title: str = "RoboMaster Camera", color_format: str = "bgr") -> bool:
        ensure_choice("color_format", color_format, ("bgr", "rgb"))
        ensure_range("fps", fps, 1, 60, unit="frames/second")
        if importlib.util.find_spec("PySide6") is None:
            raise RuntimeError("Camera.view() requires PySide6. Install it with 'python -m pip install PySide6'.")
        if self._viewer_process is not None and self._viewer_process.is_alive():
            return True
        if not self.streaming:
            self.start()
        context = multiprocessing.get_context("spawn")
        self._viewer_queue = context.Queue(maxsize=2)
        self._viewer_stop.clear()
        self._viewer_process = context.Process(
            target=run_qt_viewer,
            args=(self._viewer_queue, title, color_format),
            daemon=True,
        )
        self._viewer_process.start()
        self._viewer_thread = threading.Thread(target=self._viewer_worker, args=(fps,), daemon=True)
        self._viewer_thread.start()
        return True

    def show(self, *args: Any, **kwargs: Any) -> Any:
        return self.view(*args, **kwargs)

    def stop_view(self) -> None:
        self._viewer_stop.set()
        if self._viewer_queue is not None:
            try:
                self._viewer_queue.put_nowait(None)
            except Exception:
                pass
        if self._viewer_thread is not None and self._viewer_thread.is_alive():
            self._viewer_thread.join(timeout=1.0)
        if self._viewer_process is not None and self._viewer_process.is_alive():
            self._viewer_process.join(timeout=1.0)
            if self._viewer_process.is_alive():
                self._viewer_process.terminate()
                self._viewer_process.join(timeout=1.0)
        self._viewer_thread = None
        self._viewer_process = None
        self._viewer_queue = None

    def set_pid(self, p: float = 330, i: float = 0, d: float = 28) -> None:
        ensure_range("p", p, 0.0, 2000.0)
        ensure_range("i", i, 0.0, 1000.0)
        ensure_range("d", d, 0.0, 1000.0)
        self.pid.Kp = -p
        self.pid.Ki = -i
        self.pid.Kd = -d
        self._line_follow_config["kp"] = float(p)
        self._line_follow_config["ki"] = float(i)
        self._line_follow_config["kd"] = float(d)

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

    def _extract_line_points(self, info: Any) -> tuple[list[dict[str, float]], Optional[int]]:
        if info is None or info == [0]:
            return [], 0
        points: list[dict[str, float]] = []
        line_type: Optional[int] = None

        if isinstance(info, list) and info and all(isinstance(value, (int, float)) for value in info):
            if len(info) >= 6:
                count = int(info[0])
                line_type = int(info[1])
                flat = info[2:]
                available = min(count, len(flat) // 4)
                for idx in range(available):
                    x, y, theta, curvature = flat[idx * 4 : idx * 4 + 4]
                    points.append({"x": float(x), "y": float(y), "theta": float(theta), "curvature": float(curvature)})
            return points, line_type

        values = info if isinstance(info, list) else []
        if values and isinstance(values[0], (int, float)) and len(values) > 1 and isinstance(values[1], (list, tuple)):
            line_type = int(values[0])
            values = values[1:]

        for item in values:
            if not isinstance(item, (list, tuple)) or len(item) < 4:
                continue
            x, y, theta, curvature = item[:4]
            if not all(isinstance(value, (int, float)) for value in (x, y, theta, curvature)):
                continue
            points.append({"x": float(x), "y": float(y), "theta": float(theta), "curvature": float(curvature)})
        return points, line_type

    def _store_line(self, info: Any) -> None:
        points: list[dict[str, Any]] = []
        line_points, _ = self._extract_line_points(info)
        for point in line_points:
            x = int(point["x"] * self.width)
            y = int(point["y"] * self.height)
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

    def _normalize_heading_error(self, theta: float) -> float:
        value = float(theta)
        if abs(value) <= 3.2:
            normalized = value / 1.2
        elif abs(value) <= 180.0:
            normalized = value / 90.0
        else:
            normalized = value / 180.0
        return max(-1.0, min(1.0, normalized))

    def _clamp(self, value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def set_line_follow_profile(self, profile: str = "classroom") -> None:
        profile_name = profile.lower()
        ensure_choice("profile", profile_name, tuple(self._line_follow_profiles))
        selected = self._line_follow_profiles[profile_name]
        self._line_follow_config["profile"] = profile_name
        self._line_follow_config["kp"] = selected["kp"]
        self._line_follow_config["ki"] = selected["ki"]
        self._line_follow_config["kd"] = selected["kd"]
        self._line_follow_config["heading_weight"] = selected["heading_weight"]
        self._line_follow_config["base_speed"] = selected["base_speed"]

    def set_line_follow_speed(self, speed: float) -> None:
        ensure_range("speed", speed, 0.05, 3.5, unit="m/s")
        self._line_follow_config["base_speed"] = speed
        self.follow_speed = speed

    def stop_line_follow(self, *, stop_robot: bool = True) -> None:
        self.stop_detect()
        if stop_robot:
            self.robomaster.stop()

    def get_line_follow_status(self) -> dict[str, Any]:
        return dict(self._line_follow_state)

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
        self.set_line_follow_speed(speed)
        self.pid.output_limits = (-self.follow_speed / 3.5 * 600, self.follow_speed / 3.5 * 600)

    def set_follow_distance(self, distance: float) -> None:
        ensure_range("distance", distance, 0.0, 1.0)
        self.follow_distance = distance
        self._line_follow_config["target_x"] = self._clamp(distance, 0.0, 1.0)

    def _follow_callback(self, info: Any) -> bool:
        self._detect_callback(info)
        if self.detect_mode != "line":
            return False

        points, line_type = self._extract_line_points(info)
        now = time.monotonic()
        last_update = self._line_follow_state["last_update_time"] or now
        dt = max(0.001, now - last_update)
        self._line_follow_state["last_update_time"] = now
        self._line_follow_state["line_type"] = line_type

        if points:
            self._line_follow_state["active"] = True
            self._line_follow_state["line_visible"] = True
            self._line_follow_state["last_line_time"] = now
            lookahead = int(self._line_follow_config["lookahead"])
            point = points[min(max(0, lookahead), len(points) - 1)]
            lateral_error = point["x"] - float(self._line_follow_config["target_x"])
            heading_error = self._normalize_heading_error(point["theta"])
            weighted_error = lateral_error + float(self._line_follow_config["heading_weight"]) * heading_error

            self._line_integral += weighted_error * dt
            derivative = (weighted_error - float(self._line_follow_state["last_error"])) / dt
            self._line_follow_state["last_error"] = weighted_error

            turn_speed = (
                float(self._line_follow_config["kp"]) * weighted_error
                + float(self._line_follow_config["ki"]) * self._line_integral
                + float(self._line_follow_config["kd"]) * derivative
            )
            if bool(self._line_follow_config["invert_turn"]):
                turn_speed = -turn_speed
            max_turn = float(self._line_follow_config["max_turn_speed"])
            turn_speed = self._clamp(turn_speed, -max_turn, max_turn)

            curvature = self._clamp(abs(point["curvature"]) / 10.0, 0.0, 1.0)
            speed_scale = 1.0 - (
                float(self._line_follow_config["speed_slowdown_gain"]) * abs(turn_speed) / max_turn
                + float(self._line_follow_config["curvature_slowdown_gain"]) * curvature
            )
            speed_scale = self._clamp(speed_scale, 0.15, 1.0)
            speed = max(float(self._line_follow_config["min_speed"]), float(self._line_follow_config["base_speed"]) * speed_scale)

            self._line_follow_state["last_turn_speed"] = turn_speed
            self._line_follow_state["last_speed"] = speed
            self._line_follow_state["lost_seconds"] = 0.0
            self.robomaster.set_speed(x=speed, z=turn_speed)
            return True

        self._line_follow_state["line_visible"] = False
        last_line = self._line_follow_state["last_line_time"]
        lost_seconds = 99.0 if last_line is None else max(0.0, now - last_line)
        self._line_follow_state["lost_seconds"] = lost_seconds
        if lost_seconds <= float(self._line_follow_config["lost_hold_seconds"]):
            last_turn = float(self._line_follow_state["last_turn_speed"])
            crawl = max(0.05, float(self._line_follow_config["min_speed"]))
            self.robomaster.set_speed(x=crawl, z=last_turn * 0.6)
            return False

        if bool(self._line_follow_config["stop_on_lost"]) and lost_seconds >= float(self._line_follow_config["stop_after_lost_seconds"]):
            self.robomaster.stop()
            return False

        search_turn = float(self._line_follow_config["search_turn_speed"])
        direction = -1.0 if float(self._line_follow_state["last_turn_speed"]) < 0 else 1.0
        self.robomaster.set_speed(x=max(0.05, float(self._line_follow_config["min_speed"])), z=direction * search_turn)
        return True

    def follow(
        self,
        name: Optional[str] = None,
        color: str = "red",
        *,
        profile: str = "classroom",
        speed: Optional[float | str] = None,
        target: str = "center",
        stop_on_lost: bool = True,
        lookahead: int = 3,
    ) -> Any:
        mode = self.detect_mode if name is None else name.lower()
        if mode != "line":
            raise NotImplementedError("Only line following is implemented in the student wrapper.")
        self.set_line_follow_profile(profile)
        ensure_choice("target", target.lower(), tuple(self._line_follow_target_presets))
        self._line_follow_config["target_x"] = self._line_follow_target_presets[target.lower()]
        ensure_range("lookahead", lookahead, 0, 9)
        self._line_follow_config["lookahead"] = int(lookahead)
        self._line_follow_config["stop_on_lost"] = bool(stop_on_lost)

        if speed is not None:
            if isinstance(speed, str):
                preset = speed.lower()
                ensure_choice("speed", preset, tuple(self._line_follow_speed_presets))
                self.set_line_follow_speed(self._line_follow_speed_presets[preset])
            else:
                self.set_line_follow_speed(float(speed))

        self._line_follow_state["active"] = True
        self._line_follow_state["last_update_time"] = None
        self._line_follow_state["last_line_time"] = None
        self._line_integral = 0.0
        return self._restart_detection(mode, self._follow_callback, color=color)

    def follow_line(
        self,
        color: str = "red",
        *,
        speed: Optional[float | str] = None,
        profile: str = "classroom",
        target: str = "center",
        stop_on_lost: bool = True,
        lookahead: int = 3,
    ) -> Any:
        return self.follow(
            "line",
            color=color,
            speed=speed,
            profile=profile,
            target=target,
            stop_on_lost=stop_on_lost,
            lookahead=lookahead,
        )

    def follow_line_easy(self, color: str = "red", speed: str = "slow") -> Any:
        ensure_choice("speed", speed.lower(), tuple(self._line_follow_speed_presets))
        return self.follow_line(color=color, speed=speed.lower(), profile="classroom")

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
