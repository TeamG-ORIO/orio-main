#!/usr/bin/env python3
"""Lock or unlock the Franka joint brakes via the Desk web API, run over ssh on each control pc.

    python3 lock_arms.py              # lock both robots
    python3 lock_arms.py 2            # lock robot 2 only
    python3 lock_arms.py --unlock     # unlock both robots (sets the end effector to "Other" first)
    python3 lock_arms.py --end-effector-status   # only report which end effector Desk has selected

Before unlocking, the script makes sure Desk's Settings -> End-Effector is "Other" rather than
"Franka Hand". If it is not, it does what the page's Apply button does: selects it, keeps the
current mass / centre of mass / inertia / transform, closes the brakes and restarts the robot,
then waits for it to come back before opening the brakes. Already "Other" -> nothing to change.

Credentials come from ORIO_DESK_USER / ORIO_DESK_PASSWORD, loaded automatically from
`.env.local` at the repo root (git-ignored, so the Desk password never enters the repo).
A real environment variable wins over the file; with neither, you are prompted.

On a fresh clone, recreate it with the lab's Desk login:

    printf 'export ORIO_DESK_USER=%s\nexport ORIO_DESK_PASSWORD=%s\n' <user> <pass> > .env.local

Mirrors the Desk page's own brake button and End-Effector Apply button (System 3.0.2,
pre control-token); the endpoints and the "Other" rule were read from Desk's admin app bundle.
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
import time
import urllib.error
import urllib.request

def _load_env_local():
    """Load `.env.local` from the repo root into os.environ (if present).

    Keeps the Desk password out of the repo (the file is git-ignored) while still letting
    the script run without `source .env.local` first. An already-set environment variable
    always wins, so an explicit `ORIO_DESK_PASSWORD=... python3 lock_arms.py` still works.
    """
    # .../src/devel_packages/orio_bringup/lock_arms.py -> repo root is 3 levels up.
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    env_file = os.environ.get('ORIO_ENV_FILE') or os.path.join(repo_root, '.env.local')
    try:
        with open(env_file) as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if line.startswith('export '):
                    line = line[len('export '):]
                key, sep, value = line.partition('=')
                if not sep:
                    continue
                key, value = key.strip(), value.strip().strip('"').strip("'")
                os.environ.setdefault(key, value)   # never override a real env var
    except OSError:
        pass   # no .env.local — fall back to env vars / prompt


_load_env_local()

ROBOTS = {
    '1': os.environ.get('ORIO_DOC_HOST', 'student@iam-doc'),
    '2': os.environ.get('ORIO_LUISA_HOST', 'snaak@iam-luisa'),
}
ROBOT_IP = '172.16.0.2'

# Desk's internal ids for Settings -> End-Effector. There is no id for "Other": Desk stores it as
# "None" (ee-no-gripper) with a configuration that differs from the "None" defaults below, and
# shows "Other" whenever that is the case. Choosing "Other" in the UI keeps the current numbers.
EE_NONE = 'ee-no-gripper'
EE_LABELS = {EE_NONE: 'None', 'ee-gripper': 'Franka Hand', 'ee-cobot-pump': 'Schmalz Cobot Pump'}
EE_NONE_DEFAULTS = {
    'mass': 0.0,
    'centerOfMass': [0.0, 0.0, 0.0],
    'inertia': [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001],
    'transformation': [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],   # column-major 4x4
}
EE_CONFIG_KEYS = ('mass', 'centerOfMass', 'inertia', 'transformation')
RESTART_TIMEOUT = 180   # seconds to wait for the robot to come back after the end-effector restart


def request(method, url, body, headers, timeout=30):
    req = urllib.request.Request(url, data=body, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, context=ssl._create_unverified_context(), timeout=timeout) as r:
            return r.status, r.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()


def post(url, body, headers, timeout=30):
    return request('POST', url, body, headers, timeout)


def login_body(user, password):
    digest = ','.join(str(b) for b in hashlib.sha256(f'{password}#{user}@franka'.encode()).digest())
    return json.dumps({'login': user, 'password': base64.b64encode(digest.encode()).decode()}).encode()


class DeskError(Exception):
    pass


class Desk:
    """One logged-in Desk session on the robot, talking the same way the Desk web pages do."""

    def __init__(self, ip, user, password):
        self.base = 'https://' + ip
        self.user, self.password = user, password
        self.cookie = None

    def login(self):
        code, jwt = post(self.base + '/admin/api/login', login_body(self.user, self.password),
                         {'Content-Type': 'application/json'})
        if code != 200:
            raise DeskError(f'login -> {code}: {jwt.strip()[:300]}')
        self.cookie = 'authorization=' + jwt

    def call(self, method, path, data=None, timeout=30):
        """Request with the login cookie; JSON body if data is given. Returns (status, body)."""
        headers = {'Cookie': self.cookie}
        body = b''
        if data is not None:
            headers['Content-Type'] = 'application/json'
            body = json.dumps(data).encode()
        elif method == 'POST':
            headers['Content-Type'] = 'application/x-www-form-urlencoded'
        code, text = request(method, self.base + path, body, headers, timeout)
        try:
            return code, json.loads(text) if text.strip() else None
        except ValueError:
            return code, text

    def must(self, method, path, data=None, what=None, timeout=30):
        code, body = self.call(method, path, data, timeout)
        if code >= 400:
            raise DeskError(f'{what or path} -> {code}: {str(body).strip()[:300]}')
        return body

    # -- end effector (Settings -> End-Effector) --------------------------------------------

    def end_effector(self):
        """Return (label, selection, config) the way the Desk settings page shows them."""
        selection = self.must('GET', '/admin/api/end-effectors/selection')
        config = self.must('GET', '/admin/api/end-effector-configuration')
        if not isinstance(config, dict) or any(k not in config for k in EE_CONFIG_KEYS):
            raise DeskError(f'unexpected end-effector configuration: {str(config)[:300]}')
        label = EE_LABELS.get(selection, str(selection))
        if selection == EE_NONE and not _same_config(config, EE_NONE_DEFAULTS):
            label = 'Other'
        return label, selection, config

    def set_end_effector_other(self):
        """Make Settings -> End-Effector read "Other", exactly like pressing Apply in Desk.

        Desk's Apply does: PUT selection, POST configuration, close-brakes, restart, then waits
        for the robot to start up again. "Other" keeps whatever numbers are currently loaded.
        Returns True if a change (and so a robot restart) was needed.
        """
        label, selection, config = self.end_effector()
        print(f'end effector: {label}')
        if label == 'Other':
            return False
        if selection == EE_NONE:
            print('WARNING: end effector is "None" with the default numbers; Desk would show that as '
                  '"None" even after applying "Other". Set the numbers once in Desk. Leaving it.')
            return False
        new_config = {k: config[k] for k in EE_CONFIG_KEYS}
        print('setting end effector to Other (keeps current mass/inertia/transform); robot restarts')
        self.must('PUT', '/admin/api/end-effectors/selection', EE_NONE, 'select end effector')
        self.must('POST', '/admin/api/end-effector-configuration', new_config, 'end-effector configuration')
        self.must('POST', '/desk/api/robot/close-brakes', what='close-brakes')
        self.must('POST', '/desk/api/robot/restart', what='restart', timeout=60)
        self.wait_for_startup()
        label = self.end_effector()[0]
        print(f'end effector: {label}')
        if label != 'Other':
            raise DeskError(f'end effector still reads "{label}" after applying')
        return True

    def wait_for_startup(self):
        """Poll /admin/api/startup-phase (as Desk does) until the robot reports Started again."""
        started = time.time()
        seen_down = False
        last = None
        while time.time() - started < RESTART_TIMEOUT:
            try:
                code, body = self.call('GET', '/admin/api/startup-phase', timeout=10)
            except OSError:
                code, body = None, None
            if code == 401:          # the restart may have dropped our session
                self.login()
                continue
            tag = body.get('tag') if isinstance(body, dict) else None
            if code != 200 or tag != 'Started':
                seen_down = True
                if tag != last:
                    print(f'  robot: {tag or code or "unreachable"}')
                    last = tag
            elif seen_down or time.time() - started > 10:
                # Back up again (or it never left Started within 10 s: nothing more to wait for).
                print('  robot: Started')
                return
            time.sleep(1)
        raise DeskError(f'robot did not come back within {RESTART_TIMEOUT}s of the restart')

    # -- brakes ------------------------------------------------------------------------------

    def set_brakes(self, unlock):
        action = 'open-brakes' if unlock else 'close-brakes'
        self.must('POST', '/desk/api/robot/' + action, what=action)
        print('brakes unlocked' if unlock else 'brakes locked')


def _same_config(a, b, tol=1e-6):
    def flat(cfg):
        out = []
        for k in EE_CONFIG_KEYS:
            v = cfg[k]
            out.extend(v if isinstance(v, list) else [v])
        return out
    try:
        fa, fb = flat(a), flat(b)
        return len(fa) == len(fb) and all(abs(float(x) - float(y)) <= tol for x, y in zip(fa, fb))
    except (TypeError, ValueError):
        return False


def remote_main(ip, user, password, unlock, ee_status_only):
    """Runs on the control pc: log in, then either report, or set the end effector + brakes."""
    desk = Desk(ip, user, password)
    try:
        desk.login()
        print('logged in')
        if ee_status_only:
            label, selection, config = desk.end_effector()
            print(f'end effector: {label} (selection={selection}, mass={config["mass"]}, '
                  f'centerOfMass={config["centerOfMass"]})')
            return 0
        if unlock:
            desk.set_end_effector_other()
        desk.set_brakes(unlock)
    except (DeskError, OSError) as e:
        print(f'FAILED: {e}')
        return 1
    return 0


def credentials():
    user = os.environ.get('ORIO_DESK_USER') or input('Desk username: ')
    return user, os.environ.get('ORIO_DESK_PASSWORD') or getpass.getpass('Desk password: ')


def main():
    parser = argparse.ArgumentParser(description='Lock or unlock the Franka joint brakes.')
    parser.add_argument('robots', nargs='*', help='robot numbers (default: all)')
    parser.add_argument('--unlock', action='store_true',
                        help='open the brakes instead (arm moves slightly); sets the end effector to "Other" first')
    parser.add_argument('--end-effector-status', action='store_true',
                        help='only print which end effector Desk has selected; touches nothing')
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
        payload = f'_REMOTE = {(ROBOT_IP, user, password, args.unlock, args.end_effector_status)!r}\n' + source
        try:
            out = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host, 'python3', '-'],
                                 input=payload, capture_output=True, text=True, timeout=90 + RESTART_TIMEOUT)
            lines, ok = out.stdout.splitlines() + out.stderr.splitlines(), out.returncode == 0
        except subprocess.TimeoutExpired:
            lines, ok = ['FAILED: timed out'], False
        for line in lines:
            print(f'[robot {num} {host}]', line)
        failed += not ok
    return 1 if failed else 0


if __name__ == '__main__':
    if '_REMOTE' in globals():
        sys.exit(remote_main(*_REMOTE))
    sys.exit(main())
