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

    def test_camera_frame_reads_latest_image(self) -> None:
        fake = FakeRobot()
        fake.camera.next_image = object()
        robot = RoboMaster(_sdk_robot=fake, _sleep=lambda _: None)

        frame = robot.cam.frame()

        self.assertIs(frame, fake.camera.next_image)
        self.assertEqual(fake.camera.last_call()[0], "read_cv2_image")

    def test_camera_view_requires_pyside6_when_viewer_dependency_is_missing(self) -> None:
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake, _sleep=lambda _: None)
        import robowrap.camera as camera_module

        original_find_spec = camera_module.importlib.util.find_spec

        def fake_find_spec(name: str) -> object:
            if name == "PySide6":
                return None
            return original_find_spec(name)

        camera_module.importlib.util.find_spec = fake_find_spec
        try:
            with self.assertRaisesRegex(RuntimeError, "requires PySide6"):
                robot.cam.view()
        finally:
            camera_module.importlib.util.find_spec = original_find_spec

    def test_left_right_map_to_expected_chassis_y_direction(self) -> None:
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake, _sleep=lambda _: None)

        robot.left(0.4)
        left_call = fake.chassis.last_call()
        robot.right(0.6)
        right_call = fake.chassis.last_call()

        self.assertEqual(left_call[0], "move")
        self.assertEqual(left_call[2]["y"], -0.4)
        self.assertEqual(right_call[0], "move")
        self.assertEqual(right_call[2]["y"], 0.6)

    def test_gun_recenter_uses_timeout_and_fallback_when_action_does_not_complete(self) -> None:
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake, _sleep=lambda _: None)

        class TimeoutAction:
            def __init__(self) -> None:
                self.timeout = None

            def wait_for_completed(self, timeout=None):  # noqa: ANN001
                self.timeout = timeout
                return False

        timeout_action = TimeoutAction()
        fake.gimbal.recenter = lambda **kwargs: timeout_action  # type: ignore[method-assign]

        robot.gun.recenter()

        self.assertEqual(timeout_action.timeout, 8.0)
        self.assertTrue(any(call[0] == "moveto" for call in fake.gimbal.calls))

    def test_camera_show_alias_matches_view_dependency_behavior(self) -> None:
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake, _sleep=lambda _: None)
        import robowrap.camera as camera_module

        original_find_spec = camera_module.importlib.util.find_spec

        def fake_find_spec(name: str) -> object:
            if name == "PySide6":
                return None
            return original_find_spec(name)

        camera_module.importlib.util.find_spec = fake_find_spec
        try:
            with self.assertRaisesRegex(RuntimeError, "requires PySide6"):
                robot.cam.show()
        finally:
            camera_module.importlib.util.find_spec = original_find_spec

    def test_camera_frame_restarts_stream_after_repeated_read_failures(self) -> None:
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake, _sleep=lambda _: None)
        expected_frame = object()
        read_count = {"n": 0}

        def flaky_read(**kwargs):  # noqa: ANN003
            read_count["n"] += 1
            if read_count["n"] < 4:
                raise RuntimeError("decoder error")
            return expected_frame

        fake.camera.read_cv2_image = flaky_read  # type: ignore[method-assign]

        self.assertIsNone(robot.cam.frame())
        self.assertIsNone(robot.cam.frame())
        recovered = robot.cam.frame()

        self.assertIs(recovered, expected_frame)
        start_calls = [call for call in fake.camera.calls if call[0] == "start_video_stream"]
        stop_calls = [call for call in fake.camera.calls if call[0] == "stop_video_stream"]
        self.assertEqual(len(start_calls), 2)
        self.assertEqual(len(stop_calls), 1)

    def test_initialize_retries_then_succeeds(self) -> None:
        fake = FakeRobot()
        attempts = {"n": 0}

        def flaky_initialize(**kwargs):  # noqa: ANN003
            attempts["n"] += 1
            return attempts["n"] >= 3

        fake.initialize = flaky_initialize  # type: ignore[method-assign]
        sleep_calls: list[float] = []

        RoboMaster(_sdk_robot=fake, _sleep=lambda seconds: sleep_calls.append(seconds), connect_retries=3, connect_retry_delay=0.25)

        self.assertEqual(attempts["n"], 3)
        self.assertEqual(sleep_calls, [0.25, 0.25])

    def test_initialize_retries_then_raises_runtime_error(self) -> None:
        fake = FakeRobot()
        attempts = {"n": 0}

        def failing_initialize(**kwargs):  # noqa: ANN003
            attempts["n"] += 1
            return False

        fake.initialize = failing_initialize  # type: ignore[method-assign]
        sleep_calls: list[float] = []

        with self.assertRaisesRegex(RuntimeError, "Robot could not be connected"):
            RoboMaster(
                _sdk_robot=fake,
                _sleep=lambda seconds: sleep_calls.append(seconds),
                connect_retries=2,
                connect_retry_delay=0.1,
            )

        self.assertEqual(attempts["n"], 3)
        self.assertEqual(sleep_calls, [0.1, 0.1])
