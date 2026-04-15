from __future__ import annotations

import queue
from typing import Any


def _to_rgb_bytes(frame: Any, color_format: str) -> tuple[bytes, int, int, int]:
    """Convert a numpy-like frame to RGB bytes for Qt display."""
    height, width = frame.shape[:2]
    if len(frame.shape) == 2:
        rgb = frame
        channels = 1
    else:
        channels = frame.shape[2]
        if channels >= 3 and color_format == "bgr":
            rgb = frame[:, :, [2, 1, 0]]
        else:
            rgb = frame[:, :, :3]
            channels = min(channels, 3)
    contiguous = rgb.copy()
    return contiguous.tobytes(), width, height, channels


def run_qt_viewer(frame_queue: Any, title: str = "RoboMaster Camera", color_format: str = "bgr") -> None:
    """Run a PySide6 viewer process that displays frames from a queue."""
    try:
        from PySide6.QtCore import QTimer
        from PySide6.QtGui import QImage, QPixmap
        from PySide6.QtWidgets import QApplication, QLabel
    except ImportError as exc:
        raise RuntimeError("Install 'PySide6' to use robot.cam.view().") from exc

    app = QApplication.instance() or QApplication([])
    label = QLabel()
    label.setWindowTitle(title)
    label.setMinimumSize(640, 360)
    label.setScaledContents(True)
    label.show()

    state: dict[str, Any] = {"buffer": None}

    def update_frame() -> None:
        latest = None
        while True:
            try:
                item = frame_queue.get_nowait()
            except queue.Empty:
                break
            if item is None:
                app.quit()
                return
            latest = item
        if latest is None:
            return
        try:
            data, width, height, channels = _to_rgb_bytes(latest, color_format)
        except Exception:
            return
        state["buffer"] = data
        if channels == 1:
            image_format = QImage.Format.Format_Grayscale8
            bytes_per_line = width
        else:
            image_format = QImage.Format.Format_RGB888
            bytes_per_line = width * 3
        image = QImage(data, width, height, bytes_per_line, image_format)
        label.setPixmap(QPixmap.fromImage(image))

    timer = QTimer()
    timer.timeout.connect(update_frame)
    timer.start(15)
    app.exec()
