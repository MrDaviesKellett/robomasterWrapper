from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import time
import traceback
from importlib import metadata
from pathlib import Path
from typing import Any


def _jsonable(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    return repr(value)


def _record(results: list[dict[str, Any]], name: str, status: str, **extra: Any) -> None:
    item = {"name": name, "status": status}
    item.update({key: _jsonable(value) for key, value in extra.items()})
    results.append(item)
    marker = {"pass": "PASS", "fail": "FAIL", "skip": "SKIP", "info": "INFO"}.get(status, status.upper())
    print(f"[{marker}] {name}")
    if extra:
        print(json.dumps(item, indent=2))


def _save_frame(image: Any, path: Path) -> dict[str, Any]:
    try:
        from PIL import Image

        if hasattr(image, "shape"):
            shape = tuple(int(item) for item in image.shape)
            if len(shape) == 2:
                pil_image = Image.fromarray(image)
            elif len(shape) == 3 and shape[2] >= 3:
                pil_image = Image.fromarray(image[:, :, :3][:, :, ::-1])
            else:
                raise ValueError(f"Unsupported image shape {shape}")
        else:
            raise TypeError(f"Unsupported frame type {type(image)!r}")
        pil_image.save(path)
        return {"path": str(path), "shape": shape, "mode": pil_image.mode}
    except ImportError:
        try:
            import cv2

            ok = cv2.imwrite(str(path), image)
            if not ok:
                raise RuntimeError("cv2.imwrite returned False")
            shape = tuple(int(item) for item in image.shape) if hasattr(image, "shape") else None
            return {"path": str(path), "shape": shape, "mode": "cv2"}
        except Exception as exc:
            raise RuntimeError(f"Could not save frame: {exc}") from exc


def _frame_stats(image: Any) -> dict[str, Any]:
    stats: dict[str, Any] = {"type": type(image).__name__}
    if hasattr(image, "shape"):
        stats["shape"] = tuple(int(item) for item in image.shape)
    if hasattr(image, "dtype"):
        stats["dtype"] = str(image.dtype)
    try:
        stats["min"] = int(image.min())
        stats["max"] = int(image.max())
        stats["mean"] = float(image.mean())
    except Exception:
        pass
    return stats


def _wait_for_frame(
    robot: Any,
    *,
    attempts: int,
    delay: float,
    strategy: str,
    expected_shape: tuple[int, int] | None = None,
) -> Any:
    last_image = None
    for _ in range(attempts):
        image = robot.cam.frame(strategy=strategy)
        if image is not None:
            last_image = image
            if expected_shape is None or not hasattr(image, "shape"):
                return image
            if tuple(int(item) for item in image.shape[:2]) == expected_shape:
                return image
        time.sleep(delay)
    return last_image


def main() -> int:
    parser = argparse.ArgumentParser(description="No-motion RoboMaster camera hardware check.")
    parser.add_argument("--conn-type", default="station", choices=["device", "ap", "station", "sta", "usb", "rndis"])
    parser.add_argument("--protocol", default="tcp", choices=["tcp", "udp"])
    parser.add_argument("--serial", default=None)
    parser.add_argument("--ip", default=None, help="Recorded in the report for human/network validation; RoboMaster SDK init does not accept an IP directly.")
    parser.add_argument("--output-dir", default=r"C:\tmp\robowrap-camera-check")
    parser.add_argument("--skip-view", action="store_true", help="Skip PySide6 viewer launch.")
    parser.add_argument("--audio", action="store_true", help="Also test audio stream helpers.")
    parser.add_argument("--vision", action="store_true", help="Also test detection subscribe/unsubscribe helpers without moving.")
    parser.add_argument("--frame-attempts", type=int, default=12)
    parser.add_argument("--frame-delay", type=float, default=0.5)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    robot = None

    try:
        from robowrap import RoboMaster
        import robowrap

        _record(
            results,
            "environment",
            "info",
            python=sys.version,
            executable=sys.executable,
            platform=platform.platform(),
            cwd=os.getcwd(),
            requested_ip=args.ip,
            output_dir=str(output_dir),
            robowrap_version=metadata.version("robowrap"),
            robowrap_module=str(Path(robowrap.__file__).resolve()),
            robomaster_sdk_version=metadata.version("robomaster-sdk-modern"),
        )

        robot = RoboMaster(
            conn_type=args.conn_type,
            protocol=args.protocol,
            serial=args.serial,
            auto_reset=False,
            connect_retries=1,
            connect_retry_delay=1.0,
        )
        _record(results, "connect", "pass", conn_type=args.conn_type, protocol=args.protocol, serial=args.serial)

        try:
            robot.stop()
            _record(results, "defensive_stop_no_motion", "pass")
        except Exception as exc:
            _record(results, "defensive_stop_no_motion", "fail", error=str(exc))

        for resolution in ("360p", "540p", "720p"):
            try:
                robot.cam.set_resolution(resolution)
                start_result = robot.cam.start()
                expected_shape = (robot.cam.height, robot.cam.width)
                image = _wait_for_frame(
                    robot,
                    attempts=args.frame_attempts,
                    delay=args.frame_delay,
                    strategy="newest",
                    expected_shape=expected_shape,
                )
                if image is None:
                    _record(results, f"frame_{resolution}", "fail", start_result=start_result, reason="No frame returned")
                else:
                    screenshot = output_dir / f"camera_{resolution}.png"
                    saved = _save_frame(image, screenshot)
                    actual_shape = tuple(int(item) for item in image.shape[:2]) if hasattr(image, "shape") else None
                    status = "pass" if actual_shape == expected_shape else "fail"
                    _record(
                        results,
                        f"frame_{resolution}",
                        status,
                        start_result=start_result,
                        expected_shape=expected_shape,
                        frame=_frame_stats(image),
                        screenshot=saved,
                    )
            except Exception as exc:
                _record(results, f"frame_{resolution}", "fail", error=str(exc), traceback=traceback.format_exc())
            finally:
                try:
                    robot.cam.stop()
                except Exception:
                    pass

        try:
            robot.cam.set_resolution("360p")
            robot.cam.start()
            first = robot.cam.frame()
            robot.cam.stop()
            second = robot.cam.frame()
            if first is not None and second is not None:
                _record(results, "auto_restart_after_stop", "pass", first=_frame_stats(first), second=_frame_stats(second))
            else:
                _record(results, "auto_restart_after_stop", "fail", first_present=first is not None, second_present=second is not None)
        except Exception as exc:
            _record(results, "auto_restart_after_stop", "fail", error=str(exc), traceback=traceback.format_exc())
        finally:
            try:
                robot.cam.stop()
            except Exception:
                pass

        if args.skip_view:
            _record(results, "view", "skip", reason="--skip-view")
        else:
            try:
                robot.cam.set_resolution("360p")
                view_result = robot.cam.view(fps=10, title="robowrap camera check")
                time.sleep(4)
                robot.cam.stop_view()
                _record(results, "view", "pass", result=view_result, human_validation="Please confirm whether the viewer window showed live camera video.")
            except Exception as exc:
                _record(results, "view", "fail", error=str(exc), traceback=traceback.format_exc())
            finally:
                try:
                    robot.cam.stop()
                except Exception:
                    pass

        if args.audio:
            try:
                start_audio = robot.cam.start_audio_stream()
                audio_frame = robot.cam.read_audio_frame(timeout=2.0)
                stop_audio = robot.cam.stop_audio_stream()
                _record(
                    results,
                    "audio_stream",
                    "pass" if audio_frame is not None else "fail",
                    start_result=start_audio,
                    stop_result=stop_audio,
                    frame_type=type(audio_frame).__name__,
                    frame_len=len(audio_frame) if hasattr(audio_frame, "__len__") else None,
                )
            except Exception as exc:
                _record(results, "audio_stream", "fail", error=str(exc), traceback=traceback.format_exc())

        if args.vision:
            for name, call in (
                ("detect_person", lambda: robot.cam.detect_person()),
                ("detect_line_red", lambda: robot.cam.detect_line("red")),
                ("detect_marker_red", lambda: robot.cam.detect_marker("red")),
                ("detect_robot", lambda: robot.cam.detect_robot()),
            ):
                try:
                    result = call()
                    time.sleep(1.0)
                    robot.cam.stop_detect()
                    _record(results, name, "pass", result=result)
                except Exception as exc:
                    _record(results, name, "fail", error=str(exc), traceback=traceback.format_exc())

        return_code = 0 if all(item["status"] in {"pass", "skip", "info"} for item in results) else 1
        return return_code
    finally:
        if robot is not None:
            try:
                robot.stop()
            except Exception:
                pass
            try:
                robot.close()
            except Exception:
                pass
        report_path = output_dir / "camera_check_report.json"
        report_path.write_text(json.dumps(results, indent=2), encoding="utf-8")
        print(f"\nReport written to {report_path}")


if __name__ == "__main__":
    raise SystemExit(main())
