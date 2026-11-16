"""Run the angle generator without PyBullet, HTTP, neural network or motors."""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from motion import MotionController, PERIOD
from brain_control import BrainControl


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=10)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 3600:
        parser.error('--seconds must be 1..3600')
    controller = MotionController()
    brain = BrainControl(controller)
    brain.accept('forward', controller)
    deadline = time.monotonic()
    try:
        # Stop two seconds before ending; short runs only raise/hold the stance.
        for tick in range(args.seconds * 50):
            if tick >= max(0, args.seconds - 2) * 50:
                brain.accept('stop', controller)
            output = controller.update_motion(brain.next_command(controller), PERIOD)
            brain.observe(output)
            if tick % 50 == 0:
                print(json.dumps({'time':round(output.timestamp,2), 'state':output.state,
                                  'armed':output.armed, 'valid':output.valid,
                                  'angles_rad':output.angles}), flush=True)
            deadline += PERIOD
            time.sleep(max(0, deadline-time.monotonic()))
    except KeyboardInterrupt:
        pass  # No actuator or external process is connected by this example.
    print('Finished. No PyBullet or physical motor output.', flush=True)


if __name__ == '__main__':
    main()
