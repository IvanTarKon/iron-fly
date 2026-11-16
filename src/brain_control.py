"""Latched high-level intentions; only the controller worker calls these methods."""
import math
from motion import get_motion_command

COMMANDS = ('forward', 'backward', 'left', 'right', 'stop', 'jump', 'normal', 'start')


def validate_intent(raw):
    if not isinstance(raw, dict) or set(raw) - {'command'}:
        raise ValueError('Expected {"command": "forward|backward|left|right|stop|jump|normal|start"}')
    command = raw.get('command')
    if not isinstance(command, str) or command not in COMMANDS:
        raise ValueError('Unknown brain command')
    return command


class BrainControl:
    def __init__(self, controller, speed=.1, turn_rate=math.radians(15)):
        limits = controller.config['safety']
        for value, limit in ((speed, limits['lin_vel_max'] / 1500),
                             (turn_rate, math.radians(limits['ang_vel_max']))):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= limit:
                raise ValueError('Brain speed must leave room for 1.5x boost; turn rate must fit configured limits')
        self.speed, self.turn_rate = speed, turn_rate
        self.direction = 'stop'
        self.boost = False
        self.active = False
        self.blocked = False
        self.pending_start = False
        self.error = ''
        self.was_armed = controller.armed

    def fail(self, reason):
        self.active = True
        self.direction = 'stop'
        self.pending_start = False
        self.blocked = True
        self.error = reason

    def release(self):
        self.active = False
        self.direction = 'stop'
        self.pending_start = False

    def accept(self, command, controller):
        self.active = True
        if command == 'start':
            self.blocked = False
            self.error = ''
            self.direction = 'stop'
            self.pending_start = True
        elif command == 'stop':
            self.direction = 'stop'
            self.pending_start = False
        elif command in ('jump', 'normal'):
            self.boost = command == 'jump'
        elif command in ('forward', 'backward', 'left', 'right'):
            if self.blocked:
                return
            self.direction = command
            if not controller.armed:
                self.pending_start = True
        else:
            self.fail('Unknown brain command; start required')

    def next_command(self, controller):
        mode, vx, yaw = 'stop', 0., 0.
        if not self.blocked:
            if self.pending_start:
                mode = 'start'
                self.pending_start = False
            elif self.direction != 'stop' and controller.state in ('IDLE', 'WALK'):
                mode = 'move'
                if self.direction in ('forward', 'backward'):
                    vx = self.speed * (1.5 if self.boost else 1) * (1 if self.direction == 'forward' else -1)
                else:
                    yaw = self.turn_rate * (1 if self.direction == 'left' else -1)
        return get_motion_command(controller.time, mode, vx=vx, yaw_rate=yaw, gait=controller.gait)

    def observe(self, output):
        lost_permission = not output.armed and (self.was_armed or self.direction != 'stop' or output.state in ('FAULT', 'SHUTDOWN'))
        self.was_armed = output.armed
        if lost_permission or not output.valid:
            self.fail(output.diagnostic)

    def status(self):
        return dict(active=self.active, command=self.direction, boost=self.boost,
                    multiplier=1.5 if self.boost else 1., speed=self.speed,
                    turn_rate=self.turn_rate, blocked=self.blocked, error=self.error)
