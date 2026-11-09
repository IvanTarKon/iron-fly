"""Reproducible, headless end-to-end demo via the public motion API."""
import argparse
from dataclasses import asdict
import hashlib
import importlib.metadata
import platform
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from motion import MotionController, get_motion_command, PERIOD
from simulation_sink import SimulationSink, URDF_YAW_OFFSET
from simulation import paths
import pybullet as p
import pybullet_data

DIRECTIONS = {'forward': (.03, 0, 0), 'backward': (-.03, 0, 0),
              'left': (0, .03, 0), 'right': (0, -.03, 0),
              'ccw': (0, 0, .15), 'cw': (0, 0, -.15)}

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def run_case(gait, yaw, direction):
    controller = MotionController()
    client = p.connect(p.DIRECT)
    records = []
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(.004)
        p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        p.loadURDF('plane.urdf')
        robot = p.loadURDF(str(paths.default_urdf()), [0, 0, .20],
                           p.getQuaternionFromEuler([0, 0, yaw + URDF_YAW_OFFSET]),
                           flags=p.URDF_USE_INERTIA_FROM_FILE)
        sink = SimulationSink(p, robot, controller.config)
        def tick(mode, velocity=(0, 0, 0), *, stale=False, dt=PERIOD, next_gait=None):
            command = get_motion_command(-1 if stale else controller.time, mode, *velocity,
                                         gait=next_gait or controller.gait)
            out = controller.update_motion(command, dt)
            sink.apply(out)
            for _ in range(5):
                p.stepSimulation()
            pos, rotation = p.getBasePositionAndOrientation(robot)
            rpy = p.getEulerFromQuaternion(rotation)
            records.append({'dt': dt, 'command': asdict(command), 'output': asdict(out),
                            'position': pos, 'urdf_rpy': rpy,
                            'body_yaw': wrap(rpy[2] - URDF_YAW_OFFSET)})
            assert out.valid and out.powered, out
            return out
        tick('start', next_gait=gait)
        for _ in range(299):
            out = tick('stop')
        assert out.state == 'IDLE', out
        start = records[-1]
        for _ in range(300):
            out = tick('move', DIRECTIONS[direction])
        assert out.state == 'WALK', out
        end = records[-1]
        for _ in range(100):
            out = tick('stop')
        assert out.state == 'IDLE' and out.armed and out.powered, out
        dx = end['position'][0] - start['position'][0]
        dy = end['position'][1] - start['position'][1]
        heading = start['body_yaw']
        body_dx = math.cos(heading) * dx + math.sin(heading) * dy
        body_dy = -math.sin(heading) * dx + math.cos(heading) * dy
        turn = wrap(end['body_yaw'] - start['body_yaw'])
        vx, vy, wz = DIRECTIONS[direction]
        checks = {'direction': (body_dx * math.copysign(1, vx) > .03 if vx else
                                body_dy * math.copysign(1, vy) > .03 if vy else
                                turn * math.copysign(1, wz) > .15),
                  'cross_track': (abs(body_dy) < abs(body_dx) * .5 if vx else
                                  abs(body_dx) < abs(body_dy) * .5 if vy else True),
                  'upright': all(max(abs(r['urdf_rpy'][0]), abs(r['urdf_rpy'][1])) < .35
                                 and r['position'][2] > .04 for r in records[299:]),
                  'stopped': math.dist(records[-1]['position'], records[-26]['position']) < .005
                             and abs(wrap(records[-1]['body_yaw'] - records[-26]['body_yaw'])) < .03}
        # One physical lifecycle per gait, in addition to all direction cases.
        if yaw == 0 and direction == 'forward':
            for failure in ('stale', 'mode', 'dt'):
                for _ in range(30):
                    tick('move', (.03, 0, 0))
                out = tick('bad-mode' if failure == 'mode' else 'move', (.03, 0, 0),
                           stale=failure == 'stale', dt=0 if failure == 'dt' else PERIOD)
                assert not out.armed
                for _ in range(50):
                    out = tick('move', (.03, 0, 0))
                assert out.state == 'IDLE' and not out.armed and out.powered
                assert math.dist(records[-1]['position'], records[-26]['position']) < .005
                tick('start')
            for next_gait in ('tripod', 'wave', 'ripple'):
                out = tick('stop', next_gait=next_gait)
                assert out.gait == next_gait
                for _ in range(50):
                    out = tick('move', (.03, 0, 0))
                assert out.state == 'WALK'
                for _ in range(50):
                    out = tick('stop')
                assert out.state == 'IDLE'
            checks['physical_failures_and_gait_switches'] = True
        checks['no_active_pose_rejection'] = not any(
            r['output']['pose_rejected'] for r in records if r['output']['state'] in ('IDLE', 'WALK'))
        return {'gait': gait, 'initial_body_yaw': yaw, 'direction': direction,
                'body_displacement': [body_dx, body_dy], 'yaw_change': turn,
                'checks': checks, 'passed': all(checks.values()), 'records': records}
    finally:
        p.disconnect(client)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--quick', action='store_true', help='one smoke case only')
    args = parser.parse_args()
    cases = [('tripod', 0., 'forward')] if args.quick else [
        (g, yaw, d) for g in ('tripod', 'wave', 'ripple') for yaw in (0., math.pi / 2)
        for d in DIRECTIONS]
    results = []
    for case in cases:
        result = run_case(*case)
        results.append(result)
        print(case, 'PASS' if result['passed'] else 'FAIL',
              result['body_displacement'], result['yaw_change'], result['checks'], flush=True)
    def sha(path):
        return subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    report = {'model': 'upstream demonstration, not real Muha', 'seed': 0,
              'dt': PERIOD, 'physics_dt': .004, 'config': MotionController().config,
              'environment': {'python': sys.version, 'platform': platform.platform(),
                              'pybullet': importlib.metadata.version('pybullet')},
              'criteria': {'translation_min_m': .03, 'cross_track_ratio_max': .5,
                           'turn_min_rad': .15, 'tilt_max_rad': .35, 'height_min_m': .04,
                           'last_half_second_drift_max_m': .005, 'last_half_second_turn_max_rad': .03},
              'versions': {name: sha(ROOT / 'vendor/Hexapod-Simulation' / sub)
                           for name, sub in [('simulation', '.'), ('firmware', 'Hexapod-Firmware'),
                                             ('hardware', 'Hexapod-Hardware'), ('controller', 'Hexapod-Controller')]},
              'source_hashes': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in [ROOT / 'src/motion.py', ROOT / 'src/simulation_sink.py',
                                             Path(__file__), ROOT / 'config/demo.yml', ROOT / 'patches/0001-reject-whole-pose.patch',
                                             ROOT / 'src/bridge.cpp', ROOT / 'build/libmuha_fw.so']},
              'passed': all(r['passed'] for r in results), 'cases': results}
    output = ROOT / 'reports' / ('simulation-smoke.json' if args.quick else 'simulation.json')
    output.write_text(json.dumps(report, ensure_ascii=False))
    print(f'{len(results)} scenarios; report: {output}')
    return 0 if report['passed'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
