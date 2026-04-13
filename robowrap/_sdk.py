from __future__ import annotations

from typing import Any

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
    "pulse": getattr(sdk_led, "EFFECT_PULSE", "pulse") if sdk_led else "pulse",
    "flash": getattr(sdk_led, "EFFECT_FLASH", "flash") if sdk_led else "flash",
    "breath": getattr(sdk_led, "EFFECT_BREATH", "breath") if sdk_led else "breath",
    "scrolling": getattr(sdk_led, "EFFECT_SCROLLING", "scrolling") if sdk_led else "scrolling",
}
