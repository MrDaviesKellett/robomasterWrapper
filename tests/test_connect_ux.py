import contextlib
import io
import socket
import time
import unittest
from threading import Event
from unittest.mock import patch

from robowrap import RoboMaster
from robowrap._sdk import connection_target
from tests.fakes import FakeRobot


class ConnectUxTests(unittest.TestCase):
    def test_progress_precedes_initialize(self):
        output = io.StringIO()
        fake = FakeRobot()
        def initialize(**kwargs):
            self.assertEqual(output.getvalue(), 'Connecting to robot via ap on 192.168.2.1 ...\n')
            return True
        fake.initialize = initialize
        with contextlib.redirect_stdout(output), patch('robowrap._sdk.sdk_robot', None):
            RoboMaster(_sdk_robot=fake)

    def test_actual_route_and_serial_description(self):
        with patch('robowrap._sdk.sdk_robot') as sdk:
            sdk.config.ROBOT_IP_STR = '203.0.113.1'
            self.assertEqual(connection_target('ap'), '203.0.113.1')
        with patch('robowrap._sdk.sdk_robot', None):
            self.assertEqual(connection_target('rndis'), '192.168.42.2')
            self.assertEqual(connection_target('sta', 'ABC'), 'network discovery (serial ABC)')

    def test_timeout_bounds_unreachable_udp_route_and_closes_late_result(self):
        # A bound loopback UDP sink never replies: deterministic unreachable
        # robot protocol route without relying on the host's network routing.
        fake = FakeRobot()
        closed = Event()
        fake.close = lambda: closed.set()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink:
            sink.bind(('127.0.0.1', 0))
            def initialize(**kwargs):
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
                    client.settimeout(0.3)
                    client.sendto(b'robot discovery', sink.getsockname())
                    client.recvfrom(1024)
            fake.initialize = initialize
            started = time.monotonic()
            with self.assertRaisesRegex(RuntimeError, 'Check power, network mode, and serial number.*connect_timeout of 0.05 seconds expired'):
                RoboMaster(_sdk_robot=fake, connect_timeout=0.05)
            self.assertLess(time.monotonic() - started, 0.25)
            self.assertTrue(closed.wait(1))

    def test_heartbeat_during_blocking_initialize(self):
        fake = FakeRobot()
        release = Event()
        fake.initialize = lambda **kwargs: release.wait(1)
        output = io.StringIO()
        # Advance the displayed clock to the ten-second heartbeat, then let
        # initialization complete after the heartbeat is printed.
        def write(message):
            if 'Still connecting' in message:
                release.set()
            return io.StringIO.write(output, message)
        output.write = write
        with contextlib.redirect_stdout(output), patch('robowrap.robowrap.monotonic', side_effect=[0, 10, 10, 10]):
            RoboMaster(_sdk_robot=fake)
        self.assertIn('Still connecting to robot via ap on', output.getvalue())
        self.assertIn('(10 seconds elapsed)', output.getvalue())

    def test_invalid_timeout(self):
        for value in (0, -1, float('inf'), float('nan')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                RoboMaster(_sdk_robot=FakeRobot(), connect_timeout=value)
