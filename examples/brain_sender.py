"""Example of a second process sending decisions to the local simulation."""
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from brain_client import send_command

if __name__ == '__main__':
    try:
        send_command('normal')
        send_command('forward')
        time.sleep(3)
        send_command('jump')
        time.sleep(2)
    finally:
        send_command('stop')  # Best effort; cannot stop an unreachable server.
