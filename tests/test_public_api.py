from __future__ import annotations

import inspect
import re
import unittest
import warnings

from robowrap import (
    AiModule,
    Arm,
    Armor,
    Battery,
    Camera,
    Chassis,
    DistanceSensor,
    Gripper,
    Gun,
    RoboMaster,
    SensorAdaptor,
    Servo,
    Uart,
)

try:
    from .fakes import FakeRobot
except ImportError:  # pragma: no cover - fallback for plain unittest discovery
    from fakes import FakeRobot


SNAKE_CASE = re.compile(r"^[a-z_][a-z0-9_]*$")
ALLOWED_LEGACY_NAMES = {
    "display",
    "getModule",
    "getRobotMode",
    "getSN",
    "getVersion",
    "moveTo",
    "moveToMarker",
    "moveto",
    "playAudio",
    "playSound",
    "rotateLeft",
    "rotateRight",
    "setDebugColor",
    "setDetectMode",
    "setFollowDistance",
    "setFollowSpeed",
    "setLED",
    "setLEDs",
    "setPID",
    "setResolution",
    "setRobotMode",
    "setSpeed",
    "setVisionDebug",
    "setWheelRPMs",
    "stopAfter",
    "stopDetect",
    "turnLeft",
    "turnRight",
    "detectGesture",
    "detectLine",
    "detectMarker",
    "detectPerson",
    "detectRobot",
    "followLine",
}


class PublicApiTests(unittest.TestCase):
    def test_public_callables_are_snake_case_or_allowlisted_aliases(self) -> None:
        classes = [
            RoboMaster,
            Gun,
            Arm,
            Gripper,
            Camera,
            Chassis,
            Battery,
            Servo,
            DistanceSensor,
            SensorAdaptor,
            Armor,
            Uart,
            AiModule,
        ]
        offenders: list[str] = []
        for cls in classes:
            for name, member in cls.__dict__.items():
                if name.startswith("_") or not inspect.isfunction(member):
                    continue
                if SNAKE_CASE.match(name):
                    continue
                if name in ALLOWED_LEGACY_NAMES:
                    continue
                offenders.append(f"{cls.__name__}.{name}")
        self.assertEqual(offenders, [])

    def test_student_smoke_flow_uses_snake_case_api(self) -> None:
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake, _sleep=lambda _: None)

        robot.move(x=0.5)
        robot.cam.detect_person()
        robot.arm.move_to(x=0, z=50)
        robot.gripper.close()
        robot.close()

        self.assertEqual(fake.chassis.last_call()[0], "drive_wheels")
        self.assertTrue(any(call[0] == "sub_detect_info" for call in fake.vision.calls))
        self.assertEqual(fake.vision.last_call()[0], "unsub_detect_info")
        self.assertEqual(fake.robotic_arm.last_call()[0], "moveto")
        self.assertEqual(fake.gripper.last_call()[0], "close")
        self.assertEqual(fake.calls[-1][0], "close")

    def test_deprecated_aliases_warn_and_delegate(self) -> None:
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake, _sleep=lambda _: None)

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always", DeprecationWarning)
            robot.setRobotMode("gun")
            robot.arm.moveTo(x=10, z=20)
            robot.cam.setVisionDebug(True)
            robot.gun.setLED(64)

        self.assertEqual(len(caught), 4)
        self.assertTrue(all(item.category is DeprecationWarning for item in caught))
        self.assertEqual(fake.calls[-1][0], "set_robot_mode")
        self.assertEqual(fake.robotic_arm.last_call()[0], "moveto")
        self.assertEqual(fake.blaster.last_call()[0], "set_led")
