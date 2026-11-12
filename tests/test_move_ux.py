import unittest

from robowrap import RoboMaster
from tests.fakes import FakeRobot


class MoveUxTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeRobot()
        self.robot = RoboMaster(_sdk_robot=self.fake)

    def test_omitted_coordinates_rejected_with_accepted_arguments(self):
        for kwargs in ({}, {'speed': 1}, {'blocking': False}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError) as raised:
                self.robot.move(**kwargs)
            self.assertEqual(str(raised.exception),
                'move() requires x, y, or angle. Accepted arguments: x, y, angle, '
                'speed, turn_speed, blocking, timeout. Use move(x=0, y=0) for an intentional stop.')
        self.assertEqual(self.fake.chassis.calls, [])

    def test_explicit_zero_coordinates_accepted(self):
        for kwargs in ({'x': 0, 'y': 0}, {'x': 0}, {'angle': 0}):
            with self.subTest(kwargs=kwargs):
                self.assertTrue(self.robot.move(**kwargs))
                self.assertEqual(self.fake.chassis.last_call()[2],
                    dict(x=0, y=0, z=0, xy_speed=1, z_speed=90))
