#!/usr/bin/env python3
"""Lock or unlock the Franka joint brakes via the Desk web API, run over ssh on each control pc.

    python3 lock_arms.py              # lock both robots
    python3 lock_arms.py 2            # lock robot 2 only
    python3 lock_arms.py --unlock     # unlock both robots

Credentials come from ORIO_DESK_USER / ORIO_DESK_PASSWORD, else a prompt.
Mirrors the Desk page's own brake button (System 3.0.2, pre control-token).
"""
import argparse
import base64
import getpass
import hashlib
import json
import os
import ssl
import subprocess
import sys
import urllib.error
import urllib.request

ROBOTS = {
    '1': os.environ.get('ORIO_DOC_HOST', 'student@iam-doc'),
    '2': os.environ.get('ORIO_LUISA_HOST', 'snaak@iam-luisa'),
}
ROBOT_IP = '172.16.0.2'


def post(url, body, headers, timeout=30):
    req = urllib.request.Request(url, data=body, headers=headers, method='POST')
    try:
        with urllib.request.urlopen(req, context=ssl._create_unverified_context(), timeout=timeout) as r:
            return r.status, r.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()


def login_body(user, password):
    digest = ','.join(str(b) for b in hashlib.sha256(f'{password}#{user}@franka'.encode()).digest())
    return json.dumps({'login': user, 'password': base64.b64encode(digest.encode()).decode()}).encode()


def set_brakes(ip, user, password, unlock):
    action = 'open-brakes' if unlock else 'close-brakes'
    base = 'https://' + ip
    try:
        code, jwt = post(base + '/admin/api/login', login_body(user, password), {'Content-Type': 'application/json'})
        if code != 200:
            print(f'FAILED: login -> {code}: {jwt.strip()[:300]}')
            return 1
        print('logged in')
        code, text = post(base + '/desk/api/robot/' + action, b'', {
            'Content-Type': 'application/x-www-form-urlencoded', 'Cookie': 'authorization=' + jwt})
    except OSError as e:
        print(f'FAILED: {e}')
        return 1
    if code >= 400:
        print(f'FAILED: {action} -> {code}: {text.strip()[:300]}')
        return 1
    print('brakes unlocked' if unlock else 'brakes locked')
    return 0


def credentials():
    user = os.environ.get('ORIO_DESK_USER') or input('Desk username: ')
    return user, os.environ.get('ORIO_DESK_PASSWORD') or getpass.getpass('Desk password: ')


def main():
    parser = argparse.ArgumentParser(description='Lock or unlock the Franka joint brakes.')
    parser.add_argument('robots', nargs='*', help='robot numbers (default: all)')
    parser.add_argument('--unlock', action='store_true', help='open the brakes instead (arm moves slightly)')
    args = parser.parse_args()
    robots = args.robots or sorted(ROBOTS)
    if set(robots) - set(ROBOTS):
        parser.error(f'robots must be among {sorted(ROBOTS)}')

    user, password = credentials()
    with open(__file__) as f:
        source = f.read()

    failed = 0
    for num in robots:
        host = ROBOTS[num]
        # Credentials travel on stdin, not the ssh command line.
        payload = f'_REMOTE = {(ROBOT_IP, user, password, args.unlock)!r}\n' + source
        try:
            out = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host, 'python3', '-'],
                                 input=payload, capture_output=True, text=True, timeout=90)
            lines, ok = out.stdout.splitlines() + out.stderr.splitlines(), out.returncode == 0
        except subprocess.TimeoutExpired:
            lines, ok = ['FAILED: timed out'], False
        for line in lines:
            print(f'[robot {num} {host}]', line)
        failed += not ok
    return 1 if failed else 0


if __name__ == '__main__':
    if '_REMOTE' in globals():
        sys.exit(set_brakes(*_REMOTE))
    sys.exit(main())
