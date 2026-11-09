"""Fail before building if the pinned stack or its recorded core patch differs."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'vendor/Hexapod-Simulation'
PINS = {'.': 'fc03ebf8b222031c40dc003c95f96d7f96302cf9',
        'Hexapod-Firmware': '82d94e01df3e977c624bb671db4933b5fa1e572c',
        'Hexapod-Controller': '0f68c0c15e1d194675c294022baa5efbade28f6a',
        'Hexapod-Hardware': 'f8ee8084d48470283d5ca23394d5a4b9590ad9e8'}
for sub, expected in PINS.items():
    actual = subprocess.check_output(['git', '-C', str(SIM / sub), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != expected:
        raise SystemExit(f'{sub}: expected {expected}, got {actual}')
patch = subprocess.check_output(['git', '-C', str(SIM / 'Hexapod-Firmware'), 'diff', 'HEAD', '--', 'src/core/robot.hpp'])
if patch != (ROOT / 'patches/0001-reject-whole-pose.patch').read_bytes():
    raise SystemExit('Core differs from recorded patch; apply patches/0001-reject-whole-pose.patch or update its audit')
for sub in PINS:
    changed = subprocess.check_output(['git', '-C', str(SIM / sub), 'diff', 'HEAD', '--name-only', '--ignore-submodules=all'], text=True).splitlines()
    expected = ['src/core/robot.hpp'] if sub == 'Hexapod-Firmware' else []
    if changed != expected:
        raise SystemExit(f'Unexpected upstream changes in {sub}: {changed}')
print('Four pinned repositories and the recorded minimal patch verified')
