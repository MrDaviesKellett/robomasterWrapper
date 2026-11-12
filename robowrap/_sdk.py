from __future__ import annotations

import logging
from typing import Any, Optional

try:
    from robomaster import led as sdk_led
    from robomaster import robot as sdk_robot
    from robomaster.action import Action as SdkAction
except ImportError:  # pragma: no cover - exercised in local tests without the SDK
    sdk_led = None
    sdk_robot = None
    SdkAction = object


def create_sdk_robot() -> Any:
    """Create a RoboMaster EP SDK robot instance."""
    if sdk_robot is None:
        raise ImportError(
            "The RoboMaster SDK is not installed. Install 'robomaster-sdk-modern' "
            "(recommended) or the legacy 'robomaster' package, or inject a fake robot "
            "object when testing."
        )
    return sdk_robot.Robot()


def resolve_robot_mode(mode: str) -> Any:
    """Map student-friendly robot modes onto SDK constants."""
    normalized = mode.lower()
    if normalized == "free":
        return getattr(sdk_robot, "FREE", "free") if sdk_robot else "free"
    if normalized == "chassis":
        return getattr(sdk_robot, "CHASSIS_LEAD", "chassis_lead") if sdk_robot else "chassis_lead"
    if normalized == "gun":
        return getattr(sdk_robot, "GIMBAL_LEAD", "gimbal_lead") if sdk_robot else "gimbal_lead"
    raise ValueError("mode must be 'free', 'chassis', or 'gun'.")


def normalize_robot_mode(mode: Any) -> str:
    """Convert SDK robot mode responses back to the student-facing vocabulary."""
    if mode in ("free", getattr(sdk_robot, "FREE", object())):
        return "free"
    if mode in ("chassis_lead", getattr(sdk_robot, "CHASSIS_LEAD", object())):
        return "chassis"
    if mode in ("gimbal_lead", getattr(sdk_robot, "GIMBAL_LEAD", object())):
        return "gun"
    return str(mode)


LED_COMPONENTS = {
    "front": getattr(sdk_led, "COMP_BOTTOM_FRONT", "front") if sdk_led else "front",
    "back": getattr(sdk_led, "COMP_BOTTOM_BACK", "back") if sdk_led else "back",
    "left": getattr(sdk_led, "COMP_BOTTOM_LEFT", "left") if sdk_led else "left",
    "right": getattr(sdk_led, "COMP_BOTTOM_RIGHT", "right") if sdk_led else "right",
    "gun": getattr(sdk_led, "COMP_TOP_ALL", "gun") if sdk_led else "gun",
    "gun_left": getattr(sdk_led, "COMP_TOP_LEFT", "gun_left") if sdk_led else "gun_left",
    "gun_right": getattr(sdk_led, "COMP_TOP_RIGHT", "gun_right") if sdk_led else "gun_right",
    "all": getattr(sdk_led, "COMP_ALL", "all") if sdk_led else "all",
}


LED_EFFECTS = {
    "on": getattr(sdk_led, "EFFECT_ON", "on") if sdk_led else "on",
    "off": getattr(sdk_led, "EFFECT_OFF", "off") if sdk_led else "off",
    "pulse": getattr(sdk_led, "EFFECT_BREATH", "breath") if sdk_led else "breath",
    "flash": getattr(sdk_led, "EFFECT_FLASH", "flash") if sdk_led else "flash",
    "breath": getattr(sdk_led, "EFFECT_BREATH", "breath") if sdk_led else "breath",
    "scrolling": getattr(sdk_led, "EFFECT_SCROLLING", "scrolling") if sdk_led else "scrolling",
}


def connection_target(connection: str, serial: Any = None) -> str:
    """Describe the SDK's configured route without triggering discovery."""
    config = getattr(sdk_robot, "config", None)
    override = getattr(config, "ROBOT_IP_STR", None)
    if connection == "sta":
        if override:
            return override
        return f"network discovery (serial {serial})" if serial else "network discovery"
    name = "ROBOT_DEFAULT_RNDIS_ADDR" if connection == "rndis" else "ROBOT_DEFAULT_WIFI_ADDR"
    fallback = "192.168.42.2" if connection == "rndis" else "192.168.2.1"
    return getattr(config, name, (fallback,))[0]


def sdk_loggers() -> list[logging.Logger]:
    """Loggers the SDK writes to: ``sdk`` today, ``robomaster.*`` if renamed."""
    names = ["sdk"] + sorted(
        name for name in logging.root.manager.loggerDict
        if name == "robomaster" or name.startswith("robomaster.")
    )
    return [logging.getLogger(name) for name in names]


class QuietSdkLogs(logging.Handler):
    """Hide SDK log output while connecting, keeping records for our own error."""

    def __init__(self) -> None:
        super().__init__(logging.WARNING)
        self.records: list[logging.LogRecord] = []
        self._saved: list[tuple[logging.Logger, bool, list[tuple[logging.Handler, int]]]] = []

    def emit(self, record: logging.LogRecord) -> None:
        self.records.append(record)

    def start(self) -> None:
        for logger in sdk_loggers():
            self._saved.append((logger, logger.propagate, [(h, h.level) for h in logger.handlers]))
            for handler in logger.handlers:
                handler.setLevel(logging.CRITICAL)
            logger.propagate = False
            logger.addHandler(self)

    def stop(self) -> None:
        while self._saved:
            logger, propagate, handlers = self._saved.pop()
            logger.removeHandler(self)
            logger.propagate = propagate
            for handler, level in handlers:
                handler.setLevel(level)

    def detail(self) -> Optional[str]:
        """Most useful hidden SDK message plus a count of the rest, or None."""
        records = list(self.records)
        if not records:
            return None
        best = next((r for r in records if "connection failed" in r.getMessage().lower()), records[0])
        detail = f"SDK detail: {best.getMessage()}"
        if len(records) > 1:
            detail = f"{detail} ({len(records) - 1} more SDK log messages hidden)"
        return detail
