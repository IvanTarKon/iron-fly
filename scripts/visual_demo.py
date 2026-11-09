"""Live PyBullet window driven exclusively by the public MotionController API.

W/S forward/back, A/D left/right, Q/E yaw. Hold a key to move; release to stop.
Space stops and requires Enter to rearm; Enter starts. 1/2/3 select a gait after
stopping. Escape exits. No physical output. --headless runs a short smoke demo.
"""
import argparse
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from motion import MotionController, get_motion_command, PERIOD
from simulation_sink import SimulationSink, URDF_YAW_OFFSET
from simulation import paths
import pybullet as p
import pybullet_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--seconds', type=float, default=0, help='0: until window closes')
    args = parser.parse_args()
    if args.headless and args.seconds <= 0:
        args.seconds = 10
    c = MotionController()
    connection = p.connect(p.DIRECT if args.headless else p.GUI)
    if connection < 0:
        raise RuntimeError('Cannot open PyBullet; try --headless to check installation')
    print('WASD: move | Q/E: turn | Space: stop | Enter: start | 1/2/3: gait | Esc: exit', flush=True)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(.004)
        p.loadURDF('plane.urdf')
        robot = p.loadURDF(str(paths.default_urdf()), [0, 0, .20],
                           p.getQuaternionFromEuler([0, 0, URDF_YAW_OFFSET]),
                           flags=p.URDF_USE_INERTIA_FROM_FILE)
        sink = SimulationSink(p, robot, c.config)
        if not args.headless:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(.9, 35, -30, [0, 0, .06])
            p.addUserDebugText('WASD move | Q/E turn | SPACE stop | ENTER start | 1/2/3 gait',
                               [-.4, -.35, .32], [0.1, .1, .1], 1.1)
        moving_allowed = True
        pending_gait = c.gait
        text_id = -1
        tick = 0
        while p.isConnected():
            started = time.monotonic()
            if args.seconds and c.time >= args.seconds:
                break
            keys = {} if args.headless else p.getKeyboardEvents()
            def pressed(key):
                return bool(keys.get(key, 0) & p.KEY_WAS_TRIGGERED)
            def held(key):
                return bool(keys.get(key, 0) & p.KEY_IS_DOWN)
            if pressed(27):
                break
            if pressed(32):
                moving_allowed = False
            if pressed(13):
                moving_allowed = True
            for key, gait in ((ord('1'), 'tripod'), (ord('2'), 'wave'), (ord('3'), 'ripple')):
                if pressed(key):
                    pending_gait = gait
            mode = 'start' if tick == 0 or pressed(13) else 'stop'
            vx = vy = wz = 0.
            if moving_allowed and pending_gait == c.gait and mode != 'start':
                vx = .03 * (held(ord('w')) - held(ord('s')))
                vy = .03 * (held(ord('a')) - held(ord('d')))
                wz = .15 * (held(ord('q')) - held(ord('e')))
                norm = math.hypot(vx, vy)
                if norm > .03:
                    vx *= .03 / norm
                    vy *= .03 / norm
                if vx or vy or wz:
                    mode = 'move'
            if args.headless and 6 <= c.time < 8:
                mode, vx = 'move', .03
            gait = pending_gait if c.state in ('OFF', 'IDLE') else c.gait
            out = c.update_motion(get_motion_command(c.time, mode, vx, vy, wz, gait=gait), PERIOD)
            sink.apply(out)
            for _ in range(5):
                p.stepSimulation()
            if not args.headless and tick % 5 == 0:
                position, _ = p.getBasePositionAndOrientation(robot)
                label = f'{out.gait.upper()} | {out.state} | t={out.timestamp:.1f}s'
                if not moving_allowed:
                    label += ' | SPACE stop: press ENTER'
                if not out.armed:
                    label += ' | ' + out.diagnostic
                text_id = p.addUserDebugText(label, [position[0] - .25, position[1], .35],
                                            [.1, .1, .1], 1.3, replaceItemUniqueId=text_id)
                if tick % 50 == 0:
                    p.resetDebugVisualizerCamera(.9, 35, -30, position)
            tick += 1
            if not args.headless:
                time.sleep(max(0., PERIOD - (time.monotonic() - started)))
        print(f'Finished {tick} ticks; state={c.state}; gait={c.gait}', flush=True)
    finally:
        if p.isConnected(connection):
            p.disconnect(connection)

if __name__ == '__main__':
    main()
