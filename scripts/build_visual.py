"""Build a self-contained, compact replay from the measured PyBullet report."""
import argparse
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATES = ['SETUP', 'IDLE', 'WALK', 'SHUTDOWN', 'FAULT', 'OFF']

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'visual/iron-fly.html')
    args = parser.parse_args()
    report = json.loads((ROOT / 'reports/simulation.json').read_text())
    data = {'mounts': [], 'cases': {}}
    for name in ('front_left', 'middle_left', 'rear_left', 'front_right', 'middle_right', 'rear_right'):
        mount = report['config']['kinematics']['mounts'][name]
        data['mounts'].append([*[x / 1000 for x in mount['position']], math.radians(mount['orientation'][2])])
    for case in report['cases']:
        if case['initial_body_yaw'] != 0:
            continue
        rows = []
        for row in case['records'][:700:5]:
            values = [*row['position'], *row['urdf_rpy'], *row['output']['angles']]
            assert len(values) == 24 and all(math.isfinite(x) for x in values)
            rows.append([*[round(x, 5) for x in values], STATES.index(row['output']['state'])])
        assert len(rows) == 140
        data['cases'][case['gait'] + '/' + case['direction']] = rows
    assert len(data['cases']) == 18
    template = (ROOT / 'visual/replay.html').read_text()
    assert template.count('__DATA__') == 1
    result = template.replace('__DATA__', json.dumps(data, separators=(',', ':'), allow_nan=False))
    assert len(result.encode()) < 1_000_000
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(result)
    print(f'{args.output.resolve()}: {len(data["cases"])} cases, {len(result.encode())} bytes')

if __name__ == '__main__':
    main()
