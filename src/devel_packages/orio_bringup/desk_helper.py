#!/usr/bin/env python3
"""Runs lock_arms.py on the host for the dashboard's LOCK and UNLOCK & PRIME buttons.

The dashboard runs in the main container, which cannot reach Desk (only the control PCs
can) and has neither the ssh keys nor .env.local. run_full_demo.sh starts this on the
host for the length of a run; it listens on 127.0.0.1 only.

    python3 desk_helper.py [--port 8766] [--parent-pid PID]

    GET  /ping                                            -> {"ok": true}
    GET  /status   -> {"ok": true, "robots": {"1": {"brakes_open", "mode", "errors", "t"}, ...}}
         each robot's Desk state, refreshed every STATUS_S and after each lock/unlock
    POST /lock_arms {"robot": "1", "action": "unlock"}    -> {"ok": bool, "lines": [...]}
         action "unlock" = lock_arms.py --unlock N, "lock" = lock_arms.py N

An unlock that has to switch the end effector to "Other" restarts the robot, so a call
can take a few minutes. One call per robot at a time.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
LOCK_ARMS = os.path.join(HERE, 'lock_arms.py')
sys.path.insert(0, HERE)
import lock_arms  # noqa: E402  (also loads .env.local into os.environ)

DEFAULT_PORT = int(os.environ.get('ORIO_DESK_HELPER_PORT', '8766'))
CALL_TIMEOUT_S = 90 + lock_arms.RESTART_TIMEOUT + 30   # lock_arms.py's own ssh timeout, plus slack
_busy = {n: threading.Lock() for n in lock_arms.ROBOTS}
STATUS_S = 15.0
STATUS_LINE = re.compile(r'^\[robot (\S+) [^\]]*\] status: (\{.*\})\s*$')
_status = {}                     # robot -> {'brakes_open', 'mode', 'errors', 't'} or {'error', 't'}
_status_lock = threading.Lock()
_refresh_now = threading.Event()


def read_status():
    """Every robot's Desk state through lock_arms.py --status (read-only)."""
    now = time.time()
    if not have_credentials():
        return {n: {'error': 'no Desk login in .env.local', 't': now} for n in lock_arms.ROBOTS}
    try:
        out = subprocess.run([sys.executable, LOCK_ARMS, '--status'], stdin=subprocess.DEVNULL,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=90)
        lines = out.stdout.splitlines()
    except subprocess.TimeoutExpired:
        lines = []
    found = {}
    for line in lines:
        m = STATUS_LINE.match(line)
        if m:
            try:
                found[m.group(1)] = dict(json.loads(m.group(2)), t=now)
            except ValueError:
                pass
    failed = [l for l in lines if 'FAILED' in l]
    for n in lock_arms.ROBOTS:
        mine = [l for l in failed if l.startswith('[robot %s ' % n)]
        found.setdefault(n, {'error': (mine or ['Desk not reachable'])[-1], 't': now})
    return found


def status_loop():
    while True:
        result = read_status()
        with _status_lock:
            _status.update(result)
        _refresh_now.wait(STATUS_S)
        _refresh_now.clear()


def have_credentials():
    return bool(os.environ.get('ORIO_DESK_USER') and os.environ.get('ORIO_DESK_PASSWORD'))


def run_lock_arms(robot, action):
    """Returns (ok, output lines)."""
    if not have_credentials():
        return False, ['FAILED: no Desk login; put ORIO_DESK_USER and ORIO_DESK_PASSWORD in '
                       '.env.local (see lock_arms.py)']
    cmd = [sys.executable, LOCK_ARMS, robot] + (['--unlock'] if action == 'unlock' else [])
    try:
        out = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, timeout=CALL_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return False, ['FAILED: lock_arms.py timed out after %ds' % CALL_TIMEOUT_S]
    return out.returncode == 0, out.stdout.splitlines()


class Handler(BaseHTTPRequestHandler):
    def _reply(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/ping':
            self._reply(200, {'ok': True, 'robots': sorted(lock_arms.ROBOTS)})
        elif self.path == '/status':
            with _status_lock:
                robots = {n: dict(v) for n, v in _status.items()}
            self._reply(200, {'ok': True, 'robots': robots})
        else:
            self._reply(404, {'ok': False, 'lines': ['unknown path %s' % self.path]})

    def do_POST(self):
        if self.path != '/lock_arms':
            return self._reply(404, {'ok': False, 'lines': ['unknown path %s' % self.path]})
        try:
            req = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
        except ValueError:
            return self._reply(400, {'ok': False, 'lines': ['FAILED: request is not JSON']})
        robot, action = str(req.get('robot')), req.get('action')
        if robot not in lock_arms.ROBOTS or action not in ('lock', 'unlock'):
            return self._reply(400, {'ok': False, 'lines': ['FAILED: bad request %r' % req]})
        if not _busy[robot].acquire(blocking=False):
            return self._reply(409, {'ok': False, 'lines': [
                'FAILED: robot %s is already being locked or unlocked' % robot]})
        try:
            print('robot %s: %s requested from the dashboard' % (robot, action), flush=True)
            ok, lines = run_lock_arms(robot, action)
            for line in lines:
                print(line, flush=True)
            print('robot %s: %s %s' % (robot, action, 'done' if ok else 'FAILED'), flush=True)
        finally:
            _busy[robot].release()
            _refresh_now.set()   # show the new brake state soon
        self._reply(200, {'ok': ok, 'lines': lines})

    def log_message(self, *_args):
        pass   # no per-request access log


def watch_parent(pid):
    """Exit once the launcher that started us is gone (it may die without cleaning up)."""
    while True:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            os._exit(0)
        except PermissionError:
            pass
        time.sleep(2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--port', type=int, default=DEFAULT_PORT)
    ap.add_argument('--parent-pid', type=int, default=None,
                    help='exit when this process ends (run_full_demo.sh passes its pid)')
    args = ap.parse_args()
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    except OSError as exc:
        print('cannot listen on 127.0.0.1:%d (%s); another desk_helper.py is probably '
              'running and the dashboard will use that one' % (args.port, exc), flush=True)
        return 1
    server.daemon_threads = True
    threading.Thread(target=status_loop, daemon=True, name='status').start()
    if args.parent_pid:
        threading.Thread(target=watch_parent, args=(args.parent_pid,), daemon=True).start()
    print('dashboard lock/unlock buttons ready (127.0.0.1:%d)%s' % (
        args.port, '' if have_credentials() else
        '; WARNING: no Desk login in .env.local, so they will fail'), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
