import contextlib
import io
import logging
import traceback
import unittest

from robowrap import RoboMaster
from tests.fakes import FakeRobot


def make_robot(fake=None):
    with contextlib.redirect_stdout(io.StringIO()):
        return RoboMaster(_sdk_robot=fake or FakeRobot(), _sleep=lambda _: None)


class ContextManagerTests(unittest.TestCase):
    def test_enter_returns_robot_and_exit_closes(self):
        fake = FakeRobot()
        robot = make_robot(fake)
        with robot as entered:
            self.assertIs(entered, robot)
            self.assertNotIn(("close", (), {}), fake.calls)
        self.assertIn(("close", (), {}), fake.calls)
        self.assertEqual(fake.chassis.last_call()[0], "drive_wheels")

    def test_exit_closes_after_body_exception_and_keeps_it(self):
        fake = FakeRobot()
        with self.assertRaisesRegex(ZeroDivisionError, "division"):
            with make_robot(fake):
                1 / 0
        self.assertIn(("close", (), {}), fake.calls)

    def test_secondary_close_error_does_not_mask_body_exception(self):
        fake = FakeRobot()
        def broken_close():
            raise OSError("socket already gone")
        fake.close = broken_close
        with self.assertRaisesRegex(ValueError, "student bug"):
            with make_robot(fake):
                raise ValueError("student bug")

    def test_close_error_surfaces_when_body_succeeds(self):
        fake = FakeRobot()
        def broken_close():
            raise OSError("socket already gone")
        fake.close = broken_close
        with self.assertRaisesRegex(OSError, "socket already gone"):
            with make_robot(fake):
                pass

    def test_close_reaches_sdk_close_when_stop_fails(self):
        fake = FakeRobot()
        robot = make_robot(fake)
        def broken_drive(*args, **kwargs):
            raise OSError("chassis offline")
        fake.chassis.drive_speed = broken_drive
        with self.assertRaisesRegex(OSError, "chassis offline"):
            robot.close()
        self.assertIn(("close", (), {}), fake.calls)


class SetSpeedRemovalTests(unittest.TestCase):
    def test_camel_case_set_speed_is_gone(self):
        robot = make_robot()
        self.assertFalse(hasattr(RoboMaster, "setSpeed"))
        with self.assertRaises(AttributeError) as caught:
            robot.setSpeed(x=0.5)
        # Python's own suggestion leads to set_speed, whose error names the real APIs.
        hint = "".join(traceback.format_exception_only(caught.exception))
        self.assertIn("Did you mean", hint)
        self.assertIn("set_speed", hint)


class ConnectLogNoiseTests(unittest.TestCase):
    def setUp(self):
        self.sdk_logger = logging.getLogger("sdk")
        self.stream = io.StringIO()
        self.handler = logging.StreamHandler(self.stream)
        self.sdk_logger.addHandler(self.handler)
        self.old_level = self.sdk_logger.level
        self.sdk_logger.setLevel(logging.ERROR)

    def tearDown(self):
        self.sdk_logger.removeHandler(self.handler)
        self.sdk_logger.setLevel(self.old_level)

    def test_failed_connect_hides_sdk_errors_and_folds_detail(self):
        fake = FakeRobot()
        def initialize(**kwargs):
            self.sdk_logger.error("SdkConnection: RECV TimeOut!")
            self.sdk_logger.error("Robot: Connection Failed, target ('192.0.2.1', 40923).")
            for _ in range(5):
                self.sdk_logger.error("Client: send_sync_msg, client recv_task is not running.")
            return False
        fake.initialize = initialize
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout), self.assertRaises(RuntimeError) as caught:
            RoboMaster(_sdk_robot=fake, connect_retries=0)
        self.assertEqual(self.stream.getvalue(), "")
        self.assertEqual(stdout.getvalue(), "Connecting to robot via ap on 192.168.2.1 ...\n")
        message = str(caught.exception)
        self.assertIn("Check power, network mode, and serial number.", message)
        self.assertIn("SDK detail: Robot: Connection Failed, target ('192.0.2.1', 40923).", message)
        self.assertIn("(6 more SDK log messages hidden)", message)
        self.assertNotIn("recv_task", message)

    def test_sdk_logging_restored_after_connect(self):
        propagate = self.sdk_logger.propagate
        make_robot()
        self.assertEqual(self.handler.level, logging.NOTSET)
        self.assertEqual(self.sdk_logger.propagate, propagate)
        self.sdk_logger.error("after connect")
        self.assertIn("after connect", self.stream.getvalue())

    def test_successful_connect_has_no_sdk_detail_noise(self):
        fake = FakeRobot()
        def initialize(**kwargs):
            self.sdk_logger.error("transient warning during handshake")
            return True
        fake.initialize = initialize
        make_robot(fake)
        self.assertEqual(self.stream.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
