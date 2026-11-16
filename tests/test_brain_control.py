"""Real controller integration: retained intentions, boost, stops and fault recovery."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from motion import MotionController, PERIOD
from brain_control import BrainControl, validate_intent


class BrainTest(unittest.TestCase):
    def setUp(self):
        self.controller = MotionController()
        self.brain = BrainControl(self.controller)

    def tick(self, count=1):
        for _ in range(count):
            command = self.brain.next_command(self.controller)
            out = self.controller.update_motion(command, PERIOD)
            self.brain.observe(out)
        return out

    def accept(self, command):
        self.brain.accept(validate_intent({'command':command}), self.controller)

    def test_first_direction_starts_and_latches_without_new_messages(self):
        self.accept('forward')
        out = self.tick(350)
        self.assertEqual(out.state, 'WALK')
        self.assertTrue(out.armed and out.valid)
        self.assertAlmostEqual(self.brain.next_command(self.controller).vx, .1)
        self.accept('stop')
        out = self.tick(100)
        self.assertEqual(out.state, 'IDLE')
        self.assertTrue(out.powered)
        self.accept('backward')
        self.assertEqual(self.tick(50).state, 'WALK')
        self.assertLess(self.brain.next_command(self.controller).vx, 0)

    def test_boost_persists_through_directions_and_stop_until_normal(self):
        self.accept('forward'); self.tick(350)
        self.accept('jump'); self.tick(100)
        self.assertAlmostEqual(self.brain.next_command(self.controller).vx, .15)
        self.accept('backward')
        self.assertAlmostEqual(self.brain.next_command(self.controller).vx, -.15)
        for direction, sign in [('left',1),('right',-1)]:
            self.accept(direction)
            command = self.brain.next_command(self.controller)
            self.assertEqual((command.vx, command.vy), (0,0))
            self.assertAlmostEqual(command.yaw_rate, sign * math.radians(15))
        self.accept('stop'); self.tick(100)
        self.assertTrue(self.brain.boost)
        self.accept('normal'); self.accept('forward')
        self.assertAlmostEqual(self.brain.next_command(self.controller).vx, .1)

    def test_fault_never_auto_restarts_from_retained_or_new_direction(self):
        self.accept('forward'); self.tick(350)
        self.controller.fw.set_current(15)
        self.assertFalse(self.tick().armed)
        self.controller.fw.set_current(0)
        self.accept('backward'); self.tick(50)
        self.assertTrue(self.brain.blocked)
        self.assertEqual(self.brain.next_command(self.controller).mode, 'stop')
        self.accept('start'); self.tick(350)
        self.assertFalse(self.brain.blocked)
        self.accept('forward')
        self.assertEqual(self.tick(50).state, 'WALK')

    def test_jump_at_rest_only_changes_multiplier(self):
        self.accept('jump')
        out = self.tick(50)
        self.assertFalse(out.armed)
        self.assertFalse(self.brain.blocked)
        self.assertTrue(self.brain.boost)
        self.accept('forward')
        self.assertEqual(self.tick(350).state, 'WALK')
        self.assertAlmostEqual(self.brain.next_command(self.controller).vx, .15)

    def test_invalid_command_and_speed(self):
        for raw in (None, [], {}, {'command':[]}, {'command':'fly'}, {'command':'forward','vx':1}):
            with self.assertRaises(ValueError): validate_intent(raw)
        for speed in (.201, float('nan'), 0, True):
            with self.assertRaises(ValueError): BrainControl(self.controller, speed=speed)
        BrainControl(self.controller, speed=.2)
        self.accept('forward'); self.tick(350)
        self.brain.fail('invalid request')
        self.accept('forward')
        self.assertEqual(self.tick(100).state, 'IDLE')
        self.assertTrue(self.brain.blocked)

if __name__ == '__main__': unittest.main()
