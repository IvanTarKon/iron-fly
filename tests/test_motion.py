import math
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from motion import MotionController, MotionCommand, get_motion_command, to_servo_degrees, MAP


class MotionTest(unittest.TestCase):
    def setUp(self):
        self.c = MotionController()

    def tick(self, mode='stop', **kw):
        return self.c.update_motion(get_motion_command(self.c.time, mode, gait=self.c.gait, **kw), .02)

    def stand(self):
        self.tick('start')
        for _ in range(299):
            out = self.tick()
        self.assertEqual(out.state, 'IDLE')
        return out

    def test_stop_timeout_and_rearm(self):
        self.stand()
        for _ in range(40):
            out = self.tick('move', vx=.03)
        self.assertEqual(out.state, 'WALK')
        for _ in range(50):
            out = self.tick()
        self.assertEqual(out.state, 'IDLE')
        self.assertTrue(out.armed and out.powered)
        for _ in range(40):
            self.tick('move', vx=.03)
        out = self.c.update_motion(MotionCommand(timestamp=-2), .02)
        self.assertFalse(out.armed)
        for _ in range(50):
            out = self.tick('move', vx=.03)
        self.assertEqual(out.state, 'IDLE')
        self.assertTrue(out.powered)
        self.tick('start')
        self.assertEqual(self.tick('move', vx=.03).state, 'WALK')

    def test_invalid_input_keeps_loop_alive_and_stops(self):
        self.stand()
        for command, dt in [(None, .02), (MotionCommand(vx=float('nan')), .02),
                            (MotionCommand(mode=[]), .02), (MotionCommand(gait=[]), .02),
                            (MotionCommand(timestamp='x'), .02), (MotionCommand(), 0),
                            (MotionCommand(), float('inf')), (MotionCommand(), 'bad')]:
            self.tick('start')
            for _ in range(20):
                self.tick('move', vx=.03)
            before = self.c.time
            out = self.c.update_motion(command, dt)
            self.assertGreater(out.timestamp, before)
            self.assertFalse(out.armed)
            for _ in range(50):
                out = self.tick('move', vx=.03)
            self.assertEqual(out.state, 'IDLE')
            self.assertTrue(out.valid)

    def test_freshness_and_envelope(self):
        self.stand()
        for command in [MotionCommand('move', timestamp=self.c.time + 1),
                        MotionCommand('move', vx=.301, timestamp=self.c.time),
                        MotionCommand('move', height=.09, timestamp=self.c.time)]:
            out = self.c.update_motion(command, .02)
            self.assertFalse(out.armed)
        # A new start after a bad start must work; no edge-trigger lockout.
        self.c.update_motion(MotionCommand('start', timestamp=-1), .02)
        self.assertTrue(self.tick('start').armed)

    def test_configured_velocity_boundaries(self):
        for vx, vy, yaw in ((.3,.3,math.radians(60)),(-.3,-.3,-math.radians(60))):
            command=get_motion_command(self.c.time,'move',vx,vy,yaw)
            self.assertIsNone(self.c._validate(command))
        for kw in ({'vx':.3001},{'vy':-.3001},{'yaw_rate':math.radians(60.01)}):
            out=self.c.update_motion(get_motion_command(self.c.time,'move',**kw),.02)
            self.assertFalse(out.armed)
            self.assertIn('configured velocity limits',out.diagnostic)

    def test_gait_changes_only_at_rest(self):
        self.stand()
        for gait in ('wave', 'ripple', 'tripod'):
            out = self.c.update_motion(get_motion_command(self.c.time, gait=gait), .02)
            self.assertEqual(out.gait, gait)
            for _ in range(30):
                out = self.tick('move', vx=.03)
            self.assertEqual(out.state, 'WALK')
            different = 'wave' if gait != 'wave' else 'tripod'
            out = self.c.update_motion(get_motion_command(self.c.time, 'move', gait=different), .02)
            self.assertEqual(out.gait, gait)
            self.assertFalse(out.armed)
            for _ in range(50):
                self.tick()
            self.tick('start')

    def test_all_channels_mapping(self):
        # Independent expected channel order; distinct values detect permutations.
        angles = tuple(math.radians(i - 9) for i in range(18))
        actual = to_servo_degrees(angles, self.c.config)
        expected = []
        for public_leg in (3, 4, 5, 2, 1, 0):
            c, f, t = range(public_leg * 3 - 9, public_leg * 3 - 6)
            expected.extend([-c, f, -t - 163])
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b)
        out = self.stand()
        for a, b in zip(to_servo_degrees(out.angles, self.c.config), self.c.fw.servos()):
            self.assertAlmostEqual(a, b, places=5)

    def test_config_rejected_before_firmware(self):
        from unittest.mock import patch
        for section in ('kinematics', 'hardware', 'gaits', 'safety'):
            cfg = dict(self.c.config)
            cfg[section] = {}
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / 'invalid.yml'
                path.write_text(yaml.safe_dump(cfg))
                with patch('motion.Firmware') as factory:
                    with self.assertRaises(ValueError):
                        MotionController(path)
                    factory.assert_not_called()

    def test_overcurrent_latches_and_requires_start(self):
        self.stand()
        self.c.fw.set_current(15)
        out = self.tick('move', vx=.03)
        self.assertEqual(out.state, 'FAULT')
        self.assertFalse(out.armed or out.powered)
        self.c.fw.set_current(0)
        self.assertEqual(self.tick('move', vx=.03).state, 'FAULT')
        self.stand()

if __name__ == '__main__':
    unittest.main()
