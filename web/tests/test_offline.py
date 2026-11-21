"""Pruebas sin hardware:  python -m unittest discover -s tests -v   (desde la carpeta web)"""
import os
import sys
import threading
import time
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from app.fomo import FomoDetector, camera_to_model, prepare_image, to_gray  # noqa: E402
from app.mission import (CAMERA, COMPLETE, IDLE, MOVE, BatchCapture,  # noqa: E402
                         MissionController, TrajectoryStateMachine)
from app.pose import SetpointServer, SimPose  # noqa: E402

MODEL = os.path.normpath(os.path.join(ROOT, "..", "models", "fire_hazard-cpp-mcu-v7-impulse-#1.zip"))


class FakeCamera:
    """Entrega frames a 30 fps en un hilo, con la misma API que CameraWorker."""

    def __init__(self, frame):
        self.frame, self.id = frame, 0
        self.cv = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            time.sleep(1 / 30)
            with self.cv:
                self.id += 1
                self.cv.notify_all()

    def latest(self):
        with self.cv:
            return self.frame, self.id, time.time()

    def wait_newer(self, fid, timeout):
        end = time.time() + timeout
        with self.cv:
            while self.id <= fid:
                if end - time.time() <= 0:
                    return None
                self.cv.wait(end - time.time())
            return self.frame, self.id, time.time()


class TestFomo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.det = FomoDetector(MODEL, 0.5)

    def test_metadata(self):
        i = self.det.info
        self.assertEqual((i.width, i.height, i.channels), (180, 180, 1))
        self.assertEqual(i.labels, ["Fire"])
        self.assertAlmostEqual(i.threshold, 0.3)
        self.assertEqual((i.out_width, i.out_height), (23, 23))

    def test_black_image_no_detection(self):
        # Igual que el README del proyecto: imagen negra => status 2, sin deteccion.
        r = self.det.infer_model_ready(np.zeros((180, 180), np.uint8))
        self.assertEqual(r.status, 2)
        self.assertFalse(r.detected)
        self.assertEqual(r.confidence, 0.0)

    def test_result_ids_increase(self):
        a = self.det.infer_model_ready(np.zeros((180, 180), np.uint8)).result_id
        b = self.det.infer_model_ready(np.full((180, 180), 255, np.uint8)).result_id
        self.assertGreater(b, a)

    def test_preprocessing_matches_matlab(self):
        img = np.zeros((2, 2, 3), np.uint8)
        img[0, 0] = (0, 0, 255)  # BGR rojo puro -> round(0.299*255) = 76
        self.assertEqual(int(to_gray(img)[0, 0]), 76)
        big = np.random.default_rng(0).integers(0, 256, (720, 1280, 3), dtype=np.uint8)
        self.assertEqual(prepare_image(big, self.det.info).shape, (180, 180))
        self.assertEqual(camera_to_model(big, self.det.info).shape, (180, 180))

    def test_random_images_valid_results(self):
        rng = np.random.default_rng(1)
        for _ in range(5):
            r = self.det.infer(rng.integers(0, 256, (480, 640, 3), dtype=np.uint8))
            self.assertIn(r.status, (1, 2))
            self.assertTrue(0 <= r.confidence <= 100)
            for d in r.detections:
                self.assertLessEqual(d.x + d.width, 180)
                self.assertLessEqual(d.y + d.height, 180)


class TestStateMachine(unittest.TestCase):
    def test_full_mission_with_sim_pose(self):
        cfg = {"settling_time": 0.5, "Ts": 0.05}
        wps = [[0.5, 0.0, 1.0, 0.0], [0.5, 0.5, 1.0, 0.0]]
        sm = TrajectoryStateMachine(wps, cfg)
        pose = SimPose({"sim_noise": 0.0})
        # ImgCaptured enganchado: baja en el flanco de startCamera y sube 5 ciclos despues.
        ready, prev_cam, ticks, seen, photos = True, False, -1, set(), 0
        for _ in range(2000):
            sp, cam, home = sm.step(True, pose.read(), ready)
            seen.add(sm.state)
            if cam and not prev_cam:
                ready, ticks, photos = False, 5, photos + 1
            elif ticks > 0:
                ticks -= 1
                ready = ticks == 0
            prev_cam = cam
            pose.update(sp if home else np.array([0, 0, 1, 0.0]), 0.05)
            if sm.state == COMPLETE:
                break
        self.assertEqual(sm.state, COMPLETE)
        self.assertIn(CAMERA, seen)
        self.assertEqual(photos, 2)

    def test_invalid_feedback_never_advances(self):
        sm = TrajectoryStateMachine([[0, 0, 1, 0]], {})
        for _ in range(200):
            sm.step(True, [np.nan] * 4, False)
        self.assertEqual(sm.state, MOVE)

    def test_idle_without_start(self):
        sm = TrajectoryStateMachine([[1, 1, 1, 0]], {})
        sp, cam, home = sm.step(False, [0.2, -0.3, 0.8, 0.1], False)
        self.assertEqual(sm.state, IDLE)
        np.testing.assert_allclose(sp, [0.2, -0.3, 1.0, 0.0])
        self.assertTrue(home)


class TestBatch(unittest.TestCase):
    def test_batch_and_mission_controller(self):
        det = FomoDetector(MODEL, 0.5)
        cam = FakeCamera(np.zeros((480, 640, 3), np.uint8))
        cfg = {"settling_time": 0.3, "batch_images": 5, "capture_seconds": 0.5,
               "waypoints": [[0.2, 0.0, 1.0, 0.0], [0.2, 0.2, 1.0, 0.0]]}
        mc = MissionController(cam, det, SimPose({"sim_noise": 0.0, "sim_speed": 2.0}),
                               SetpointServer({}), cfg)
        mc.start()
        mc.start_mission()
        deadline = time.time() + 20
        while time.time() < deadline and mc.snapshot()["state"] != "COMPLETE":
            time.sleep(0.1)
        snap = mc.snapshot()
        mc.stop()
        self.assertEqual(snap["state"], "COMPLETE")
        self.assertEqual(len(snap["results"]), 2)
        self.assertFalse(any(r["objectDetected"] for r in snap["results"]))  # imagenes negras
        self.assertEqual(snap["setpoint"], [0.0, 0.0, 1.0, 0.0])  # Home al terminar

    def test_capture_fails_without_frames(self):
        class DeadCamera(FakeCamera):
            def _run(self):
                pass
        det = FomoDetector(MODEL, 0.5)
        bc = BatchCapture(DeadCamera(np.zeros((480, 640, 3), np.uint8)), det,
                          {"batch_images": 3, "capture_seconds": 0.2})
        bc.trigger(0, [0, 0, 1, 0])
        bc._thread.join(5)
        self.assertEqual(bc.status, -2)
        self.assertFalse(bc.ready)


if __name__ == "__main__":
    unittest.main()
