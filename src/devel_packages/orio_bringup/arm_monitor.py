#!/usr/bin/env python3
"""Live dashboard for both Franka arms, served at http://localhost:8050.

    python3 arm_monitor.py [--port 8050] [--robots 1 2]

Desk websockets (via ssh tunnel to each control pc) give joints, pose, brakes, mode and
errors whenever the arm is powered; frankapy's RobotState over ROS adds velocities,
torques and external force while the stack is up. Read-only. Needs anaconda python (aiohttp).
"""
import argparse
import asyncio
import json
import math
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
ROS_PATHS = ['/opt/ros/noetic/lib/python3/dist-packages', str(REPO / 'devel/lib/python3/dist-packages')]
PUSH_HZ = 20
DESK_STREAMS = {
    'configuration': '/desk/api/robot/configuration',
    'status': '/desk/api/robot/status',
    'system': '/desk/api/system/status',
}


def pose_fields(m):
    """Column-major 4x4 -> position (m) and roll/pitch/yaw (deg)."""
    r = lambda i, j: m[j * 4 + i]
    rpy = [math.atan2(r(2, 1), r(2, 2)), math.asin(max(-1.0, min(1.0, -r(2, 0)))), math.atan2(r(1, 0), r(0, 0))]
    return {'xyz': [m[12], m[13], m[14]], 'rpy': [math.degrees(a) for a in rpy]}


# ---- ROS bridge (child process, restarted whenever roscore goes away or restarts) ----

def ros_bridge(robots):
    sys.path[:0] = [p for p in ROS_PATHS if p not in sys.path]
    import rosgraph
    import rospy
    from franka_interface_msgs.msg import RobotState

    try:
        run_id = rosgraph.Master('/arm_monitor').getParam('/run_id')
    except Exception:
        sys.exit(3)
    rospy.init_node('arm_monitor', anonymous=True, disable_signals=True)
    latest = {}

    def cb(msg, num):
        errors = msg.current_errors
        latest[num] = {
            'q': list(msg.q), 'dq': list(msg.dq), 'tau': list(msg.tau_J),
            'tau_ext': list(msg.tau_ext_hat_filtered), 'f_ext': list(msg.O_F_ext_hat_K),
            'pose': pose_fields(msg.O_T_EE), 'mode': msg.robot_mode,
            'errors': [k for k in errors.__slots__ if getattr(errors, k) is True],
            'success_rate': msg.control_command_success_rate,
        }

    for n in robots:
        rospy.Subscriber(f'/robot_state_publisher_node_{n}/robot_state', RobotState, cb, callback_args=n, queue_size=1)
    last_check = time.time()
    while True:
        time.sleep(1 / PUSH_HZ)
        if latest:
            out = dict(latest)
            latest.clear()
            print(json.dumps(out), flush=True)
        if time.time() - last_check > 2:
            last_check = time.time()
            try:
                if rosgraph.Master('/arm_monitor').getParam('/run_id') != run_id:
                    sys.exit(4)
            except Exception:
                sys.exit(3)


# ---- server ----

def main():
    parser = argparse.ArgumentParser(description='Live dashboard for the Franka arms.')
    parser.add_argument('--port', type=int, default=8050)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--robots', nargs='*', default=None, help='robot numbers (default: all)')
    parser.add_argument('--ros-bridge', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()

    sys.path.insert(0, str(HERE))
    from lock_arms import ROBOTS, ROBOT_IP, credentials, login_body
    robots = args.robots or sorted(ROBOTS)
    if set(robots) - set(ROBOTS):
        parser.error(f'robots must be among {sorted(ROBOTS)}')
    if args.ros_bridge:
        return ros_bridge(robots)

    import aiohttp
    from aiohttp import web

    user, password = credentials()
    state = {n: {'host': ROBOTS[n], 'desk': {}, 'desk_t': {}, 'desk_error': 'connecting', 'ros': None, 'ros_t': 0}
             for n in robots}
    ros_status = {'text': 'starting'}
    clients = set()

    async def open_tunnel(num, port):
        proc = await asyncio.create_subprocess_exec(
            'ssh', '-N', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ExitOnForwardFailure=yes',
            '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
            '-L', f'127.0.0.1:{port}:{ROBOT_IP}:443', ROBOTS[num],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        for _ in range(60):
            if proc.returncode is not None:
                err = (await proc.stderr.read()).decode().strip().splitlines()
                raise ConnectionError(f'ssh tunnel failed: {err[-1] if err else proc.returncode}')
            try:
                _, w = await asyncio.open_connection('127.0.0.1', port)
                w.close()
                return proc
            except OSError:
                await asyncio.sleep(0.25)
        proc.kill()
        raise ConnectionError('ssh tunnel timed out')

    async def desk_stream(http, url, headers, key, s):
        async with http.ws_connect(url, headers=headers, ssl=False) as ws:
            while True:
                msg = await ws.receive(timeout=10)
                if msg.type != aiohttp.WSMsgType.TEXT:
                    raise ConnectionError(f'{key} stream closed')
                s['desk'][key] = json.loads(msg.data)
                s['desk_t'][key] = time.time()
                s['desk_error'] = None

    async def desk_loop(num, port):
        s = state[num]
        while True:
            tunnel = None
            try:
                tunnel = await open_tunnel(num, port)
                async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=False)) as http:
                    base = f'127.0.0.1:{port}'
                    r = await http.post(f'https://{base}/admin/api/login', data=login_body(user, password),
                                        headers={'Content-Type': 'application/json'})
                    jwt = await r.text()
                    if r.status != 200:
                        raise PermissionError(f'Desk login {r.status}: {jwt.strip()}')
                    headers = {'Cookie': 'authorization=' + jwt}
                    await asyncio.gather(*(desk_stream(http, f'wss://{base}{path}', headers, key, s)
                                           for key, path in DESK_STREAMS.items()))
            except PermissionError as e:
                s['desk_error'], delay = str(e), 30
            except Exception as e:
                s['desk_error'], delay = str(e) or type(e).__name__, 3
            finally:
                if tunnel and tunnel.returncode is None:
                    tunnel.kill()
                    await tunnel.wait()
            await asyncio.sleep(delay)

    async def ros_loop():
        while True:
            proc = await asyncio.create_subprocess_exec(
                sys.executable, str(Path(__file__).resolve()), '--ros-bridge', '--robots', *robots,
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            ros_status['text'] = 'connecting'
            try:
                async for line in proc.stdout:
                    now = time.time()
                    for num, fields in json.loads(line).items():
                        state[num]['ros'], state[num]['ros_t'] = fields, now
                    ros_status['text'] = 'live'
            finally:
                if proc.returncode is None:
                    proc.kill()
                code = await proc.wait()
            err = (await proc.stderr.read()).decode().strip().splitlines()
            ros_status['text'] = {3: 'no ROS master', 4: 'roscore restarted'}.get(
                code, f'bridge exited: {err[-1] if err else code}')
            await asyncio.sleep(2)

    def snapshot():
        now = time.time()
        out = {}
        for num, s in state.items():
            d, t = s['desk'], s['desk_t']
            desk = {'age': now - t['configuration'] if 'configuration' in t else None, 'error': s['desk_error']}
            if 'configuration' in d:
                desk['q'] = d['configuration'].get('jointAngles')
                desk['pose'] = pose_fields(d['configuration']['cartesianPose'])
            if 'status' in d:
                desk['errors'] = [k for k, v in d['status'].get('robotErrors', {}).items() if v]
            if 'system' in d:
                sysd = d['system']
                desk.update(brakes=sysd.get('brakesOpen'), mode=sysd.get('operationalMode'),
                            reason=sysd.get('operationalModeTransitionReason'),
                            joints_in_error=sysd.get('jointsInError'),
                            connected=sysd.get('slavesOperational') and sysd.get('masterStatus') == 'OP')
            ros = dict(s['ros'], age=now - s['ros_t']) if s['ros'] else None
            out[num] = {'host': s['host'], 'desk': desk, 'ros': ros}
        return {'t': now, 'ros_status': ros_status['text'], 'robots': out}

    async def push_loop():
        while True:
            await asyncio.sleep(1 / PUSH_HZ)
            if clients:
                msg = json.dumps(snapshot())
                for ws in list(clients):
                    try:
                        await ws.send_str(msg)
                    except ConnectionError:
                        clients.discard(ws)

    async def index(_):
        return web.FileResponse(HERE / 'arm_monitor.html')

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        clients.add(ws)
        try:
            async for _ in ws:
                pass
        finally:
            clients.discard(ws)
        return ws

    async def start_tasks(app):
        tasks = [asyncio.create_task(desk_loop(n, 18440 + int(n))) for n in robots]
        tasks += [asyncio.create_task(ros_loop()), asyncio.create_task(push_loop())]
        app['tasks'] = tasks

    async def stop_tasks(app):
        for task in app['tasks']:
            task.cancel()
        await asyncio.gather(*app['tasks'], return_exceptions=True)

    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_get('/ws', ws_handler)
    app.on_startup.append(start_tasks)
    app.on_cleanup.append(stop_tasks)
    print(f'arm monitor: http://{"localhost" if args.host == "127.0.0.1" else args.host}:{args.port}')
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == '__main__':
    sys.exit(main())
