"""Send one high-level command from another process; standard library only."""
import argparse
import json
from urllib.request import Request, urlopen


def send_command(command, url='http://127.0.0.1:8766', timeout=3):
    request = Request(url.rstrip('/') + '/api/brain',
                      json.dumps({'command': command}).encode(),
                      headers={'Content-Type':'application/json', 'X-Muha-UI':'1'}, method='POST')
    with urlopen(request, timeout=timeout) as response:
        return json.load(response)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('forward','backward','left','right','stop','jump','normal','start'))
    parser.add_argument('--url', default='http://127.0.0.1:8766')
    args = parser.parse_args()
    print(send_command(args.command, args.url))
