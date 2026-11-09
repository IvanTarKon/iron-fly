"""Simulation-only adapter over the pinned firmware. SI units, body X forward."""
from dataclasses import dataclass
from pathlib import Path
import math
import ctypes
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'vendor/Hexapod-Simulation'
sys.path[:0] = [str(SIM), str(SIM / 'Hexapod-Controller')]
from simulation import Firmware, SimTransport
from hexapod import HexapodClient, GaitId
from hexapod.client import DEFAULT_CONFIG

LEGS = ('front_left', 'middle_left', 'rear_left', 'front_right', 'middle_right', 'rear_right')
MAP = (5, 4, 3, 0, 1, 2)
PERIOD = 0.02
TIMEOUT = 0.5
GAITS = {name.lower(): value for name, value in GaitId.__members__.items()}

@dataclass(frozen=True)
class MotionCommand:
    mode: str = 'stop'
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    height: float = 0.08
    timestamp: float = 0.0  # seconds on this controller's logical clock
    gait: str = 'tripod'

@dataclass(frozen=True)
class MotionOutput:
    angles: tuple
    state: str
    diagnostic: str
    timestamp: float
    valid: bool
    armed: bool
    powered: bool
    gait: str
    pose_rejected: bool
    rejected_poses: int


def load_config(path):
    config = yaml.safe_load(Path(path).read_text())
    reference = yaml.safe_load(DEFAULT_CONFIG.read_text())
    # This milestone supports exactly the verified model, including calibration
    # and gait/safety settings. Serial settings are unused by this adapter.
    if not isinstance(config, dict):
        raise ValueError('config must be a mapping')
    for section in ('kinematics', 'hardware', 'safety', 'gaits'):
        if config.get(section) != reference[section]:
            raise ValueError(f'{section}: only the pinned demonstration profile is supported')
    return config


def to_servo_degrees(angles, config):
    """Public LF..RR kinematic radians -> upstream FR..FL servo degrees."""
    if len(angles) != 18 or not all(math.isfinite(q) for q in angles):
        raise ValueError('expected 18 finite angles')
    result = [0.0] * 18
    for leg, index, offset in zip(LEGS, MAP, range(0, 18, 3)):
        for joint in range(3):
            result[3 * index + joint] = (
                math.degrees(angles[offset + joint]) * config['hardware']['direction'][leg][joint]
                + config['hardware']['trim'][leg][joint])
    return result


class MotionController:
    def __init__(self, config=ROOT / 'config/demo.yml'):
        self.config = load_config(config)  # before constructing or enabling firmware
        self.fw = Firmware(str(ROOT / 'build/libmuha_fw.so'))
        self.fw._lib.fw_pose_rejected.argtypes = [ctypes.c_void_p]
        self.fw._lib.fw_pose_rejected.restype = ctypes.c_int
        self.fw._lib.fw_rejected_poses.argtypes = [ctypes.c_void_p]
        self.fw._lib.fw_rejected_poses.restype = ctypes.c_uint
        self.bot = HexapodClient(SimTransport(self.fw))
        self.bot.provision(self.config)
        self.time = 0.0
        self.armed = False
        self.diagnostic = 'start required; simulation only'
        self.gait = 'tripod'
        self.state = 'OFF'

    def _reject(self, reason):
        self.armed = False
        self.diagnostic = reason + '; explicit start required'

    def _validate(self, command):
        if not isinstance(command, MotionCommand):
            return 'invalid command type'
        if command.mode not in ('start', 'move', 'stop'):
            return 'invalid mode'
        if not isinstance(command.gait, str) or command.gait not in GAITS:
            return 'invalid gait'
        values = (command.vx, command.vy, command.yaw_rate, command.height, command.timestamp)
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in values):
            return 'nonfinite or nonnumeric command'
        if not -1e-9 <= self.time - command.timestamp <= TIMEOUT + 1e-9:
            return 'stale or future command'
        # Match the core's per-axis limits; reject rather than silently clamp.
        limits = self.config['safety']
        linear = limits['lin_vel_max'] / 1000
        angular = math.radians(limits['ang_vel_max'])
        if max(abs(command.vx), abs(command.vy)) > linear + 1e-9 or abs(command.yaw_rate) > angular + 1e-9:
            return 'outside configured velocity limits'
        if abs(command.height - .08) > 1e-9:
            return 'only demonstration height 0.08 m is supported'
        if command.gait != self.gait and self.state not in ('OFF', 'IDLE'):
            return 'stop and wait for IDLE before switching gait'
        return None

    def update_motion(self, command, dt):
        """Invalid input latches a stop; still tick using 20 ms for invalid dt.

        Caller must continue ticking. This is not a process/hardware watchdog.
        Freshness uses time at entry, so timestamp=controller.time is valid.
        """
        dt_ok = (not isinstance(dt, bool) and isinstance(dt, (int, float))
                 and math.isfinite(dt) and 0 < dt <= .1)
        reason = self._validate(command) if dt_ok else 'invalid dt; fallback tick 0.02 s'
        step = dt if dt_ok else PERIOD
        if reason:
            self._reject(reason)
        else:
            if command.gait != self.gait:
                self.bot.set_gait(GAITS[command.gait])
                self.gait = command.gait
            if command.mode == 'start':
                self.bot.enable()
                self.armed = True
                self.diagnostic = 'simulation only'
        if not reason and self.armed and command.mode == 'move':
            self.bot.set_velocity(command.vx * 1000, command.vy * 1000,
                                  math.degrees(command.yaw_rate))
        else:
            self.bot.set_velocity(0, 0, 0)
        self.bot.heartbeat()
        self.fw.update(step)
        self.time += step
        self.state = self.bot.get_telemetry().state.name
        if self.state in ('FAULT', 'SHUTDOWN'):
            self._reject('core ' + self.state)
        pose_rejected = bool(self.fw._lib.fw_pose_rejected(self.fw._h))
        rejected_poses = self.fw._lib.fw_rejected_poses(self.fw._h)
        if pose_rejected and self.state in ('IDLE', 'WALK'):
            self._reject('pose rejected; holding previous complete pose')
            self.bot.stop()
        servo = self.fw.servos()
        angles = tuple(math.radians((servo[3 * index + j] - self.config['hardware']['trim'][leg][j])
                                   / self.config['hardware']['direction'][leg][j])
                       for leg, index in zip(LEGS, MAP) for j in range(3))
        ranges = [self.config['safety'][key + '_range'] for key in ('coxa', 'femur', 'tibia')]
        valid = all(math.isfinite(q) and ranges[i % 3][0] - 1e-4 <= q <= ranges[i % 3][1] + 1e-4
                    for i, q in enumerate(servo))
        if not valid:
            self._reject('invalid core output')
            self.bot.stop()
        return MotionOutput(angles, self.state, self.diagnostic, self.time, valid,
                            self.armed, self.fw.powered(), self.gait, pose_rejected, rejected_poses)


def get_motion_command(now, mode='stop', vx=0., vy=0., yaw_rate=0., height=.08, gait='tripod'):
    return MotionCommand(mode, vx, vy, yaw_rate, height, now, gait)
