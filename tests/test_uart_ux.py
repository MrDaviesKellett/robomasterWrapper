import unittest

from robowrap import RoboMaster
from tests.fakes import FakeRobot


class UartUxTests(unittest.TestCase):
    def test_receive_error_includes_output_alternative(self):
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake)
        with self.assertRaises(NotImplementedError) as raised:
            robot.uart.subscribe(lambda _: None)
        self.assertEqual(str(raised.exception),
            'UART receive is unavailable: the upstream SDK receive path crashes on send ACKs. '
            'Use send() for output; receive stays unavailable until the upstream SDK is fixed.')
        self.assertEqual(fake.uart.calls, [])
        self.assertTrue(robot.uart.send(b'hello'))
