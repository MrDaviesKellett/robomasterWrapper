import unittest

from robowrap import RoboMaster
from robowrap.helperFuncs import clamp, ensure_color, ensure_int_range, ensure_range
from tests.fakes import FakeRobot


class ValidatorUxTests(unittest.TestCase):
    def test_type_messages_name_parameter_and_types(self):
        robot = RoboMaster(_sdk_robot=FakeRobot())
        cases = [
            (lambda: robot.move(x='oops'), 'x must be of type int or float.'),
            (lambda: robot.arm.move(x='oops'), 'x must be of type int.'),
            (lambda: ensure_range('speed', None, 0, 1), 'speed must be of type int or float.'),
            (lambda: ensure_int_range('count', 1.5, 0, 2), 'count must be of type int.'),
            (lambda: ensure_color('rgb', []), 'rgb must be of type tuple with three int values (r, g, b).'),
            (lambda: ensure_color('rgb', (0, 'x', 0)), 'rgb[1] must be of type int.'),
            (lambda: clamp('x', 0, 1), 'val must be of type int or float.'),
            (lambda: clamp(0, 'x', 1), 'min_val must be of type int or float.'),
            (lambda: clamp(0, 0, 'x'), 'max_val must be of type int or float.'),
        ]
        for call, message in cases:
            with self.subTest(message=message), self.assertRaises(TypeError) as raised:
                call()
            self.assertEqual(str(raised.exception), message)
