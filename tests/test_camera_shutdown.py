import queue
import threading
import unittest
from types import SimpleNamespace
from robowrap import RoboMaster
from tests.fakes import FakeRobot

class CameraShutdown(unittest.TestCase):
    def test_full_queues_allow_shutdown_sentinel_and_restart(self):
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake)
        frames, packets = queue.Queue(1), queue.Queue(1)
        frames.put('frame')
        packets.put('packet')
        fake.camera._liveview = SimpleNamespace(
            _video_frame_queue=frames,
            _video_stream_conn=SimpleNamespace(_sock_queue=packets))
        def stop():
            packets.put(None)
            frames.put(None)
            return True
        fake.camera.stop_video_stream = stop
        robot.cam.streaming = True
        finished = threading.Event()
        def shutdown():
            robot.cam.stop()
            finished.set()
        worker = threading.Thread(target=shutdown, daemon=True)
        worker.start()
        self.assertTrue(finished.wait(1), 'shutdown blocked on a full media queue')
        self.assertEqual((frames.maxsize, packets.maxsize), (1, 1))
        self.assertFalse(robot.cam.streaming)

    def test_queue_bounds_restored_after_sdk_exception(self):
        fake = FakeRobot()
        robot = RoboMaster(_sdk_robot=fake)
        packets = queue.Queue(1)
        fake.camera._liveview = SimpleNamespace(
            _video_stream_conn=SimpleNamespace(_sock_queue=packets))
        def stop():
            raise RuntimeError('transport error')
        fake.camera.stop_video_stream = stop
        robot.cam.streaming = True
        with self.assertRaisesRegex(RuntimeError, 'transport error'):
            robot.cam.stop()
        self.assertEqual(packets.maxsize, 1)
