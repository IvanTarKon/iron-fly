"""Thin adapter to the existing BulletInterface; no real actuator output."""
import math
from motion import to_servo_degrees
from simulation.bullet.interface import BulletInterface

# URDF +Y is forward: R_world_urdf = R_world_body * Rz(-pi/2).
URDF_YAW_OFFSET = -math.pi / 2

class SimulationSink:
    def __init__(self, physics, robot_id, config):
        self.interface = BulletInterface(physics, robot_id)
        self.config = config
        expected = [f'leg_{leg}_{joint}' for leg in range(1, 7)
                    for joint in ('coxa', 'femur', 'tibia')]
        actual = [physics.getJointInfo(robot_id, j)[1].decode() for j in self.interface.joints]
        if actual != expected:
            raise ValueError('URDF joint ordering differs from the pinned model')

    def apply(self, output):
        if not output.valid:
            raise ValueError('invalid motion output; simulation aborted')
        self.interface.apply(to_servo_degrees(output.angles, self.config), output.powered)
