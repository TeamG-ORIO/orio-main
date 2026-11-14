#!/usr/bin/env python3
"""
scan_dashboard.py - the window for the autonomous RFID scan demo: start and stop the robot, watch the run, see the
inventory fill in. Runs on the PC (run_scan_demo.sh starts it); the robot is reached through scripts/demo_agent.py
in orio_stretch, the inventory through sync/pc_receiver.py on this PC.

    python3 scan_dashboard.py [--conf demo.conf] [--agent http://robot:8123] [--receiver http://127.0.0.1:8000] [--token T]

Top bar      START / PAUSE / STOP and E-STOP (also the Esc key). E-STOP stops all motion and latches: release it with
             the button on the robot's head (hold 2 s) or RELEASE RUNSTOP here.
Setup        real run or the replay of an old run (no robot motion), laps, start waypoint, localisation method.
Checks       one line per thing that has to work, green when it does: the list to look at when something is wrong.
Map          the room in 2D (from above) or 3D: shelves, the map's point cloud, waypoints, the driven path, the robot and
             every item where the localiser has placed it so far (filled dot = placed in this scan, grey ring = last known
             place, red cross = missing). Buttons 2D / 3D / CAMERA; drag, wheel and double-click move the view; point at
             an item or click it (or its row in the inventory) for its name and uncertainty. Needs numpy and Pillow;
             without them only the route is drawn.
Inventory    every item of the PC's database: on shelf / not seen yet / MISSING (after the run), and its shelf when
             the item localiser has placed it.
             In manual mode (no run going) CAMERA shows the head camera, about 6 pictures a second.
Manual       MANUAL ON starts the driver and then, by itself, raises the lift, retracts the arm, turns the wrist in to
             30 degrees short of stowed and points the head as a run does (it asks first). Everything on the pad is
             hold-to-move: base (W A S D / Q E), lift (R / F; the lift's whole homed range, 0 to 1.10 m; below 0.42 m the
             panel can touch the base) and camera (I J K L). The camera's middle button puts it back. MANUAL OFF puts the head back to its usual pose before the driver stops, so every run
             starts with the camera looking the same way. POSE AGAIN repeats the pose move. Hold a pad button, or W A S D / Q E with the pointer over the pad.
             The robot moves only while a button is held (it stops by itself 0.5 s after the last command).

Standard library only (tkinter). The same colours as the arms' dashboard (orio_dashboard.py).
"""
import argparse
import json
import math
import os
import threading
import time
import tkinter as tk
import urllib.error
import urllib.request
from tkinter import font as tkfont, messagebox, ttk

try:
    import io
    from PIL import Image, ImageTk
except ImportError:      # the camera picture then shows a note instead
    Image = ImageTk = None
try:
    import scene_view    # numpy + Pillow: the 2D / 3D map
except ImportError:
    scene_view = None

HERE = os.path.dirname(os.path.abspath(__file__))

# Catppuccin Mocha, as in orio_dashboard.py
CRUST, MANTLE, BASE = '#11111b', '#181825', '#1e1e2e'
SURF0, SURF1, SURF2 = '#313244', '#45475a', '#585b70'
OVERLAY, SUBTEXT, TEXT = '#6c7086', '#a6adc8', '#cdd6f4'
BLUE, MAUVE, GREEN, YELLOW = '#89b4fa', '#cba6f7', '#a6e3a1', '#f9e2af'
PEACH, RED, TEAL, SKY = '#fab387', '#f38ba8', '#94e2d5', '#89dceb'
ESTOP = '#d20f39'
DOT = {'ok': GREEN, 'wait': YELLOW, 'bad': RED, 'off': SURF2}
PHASE_COLOR = {'idle': SURF2, 'driving': GREEN, 'replaying': TEAL, 'paused': YELLOW, 'done': GREEN, 'replay finished': GREEN,
               'failed': RED, 'E-STOPPED': RED, 'stopped': PEACH, 'stopping': PEACH}
V_STEP, W_STEP = 0.10, 0.35          # manual speed at full scale (the robot clamps to 0.15 m/s, 0.5 rad/s)
DRIVE_KEYS, JOG_KEYS = 'wasdqe', 'rfijkl'     # held keys / buttons: base, then lift up / down and camera up / left / down / right


def read_conf(path):
    conf = {}
    for line in open(path):
        line = line.strip()
        if line and not line.startswith('#') and '=' in line:
            k, v = line.split('=', 1)
            conf[k.strip()] = v.strip()
    return conf


def http(url, payload=None, token='', timeout=2.5):
    data = None if payload is None else json.dumps(payload).encode()
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json', 'X-Orio-Token': token})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def ago(t):
    if not t:
        return 'never'
    s = max(0.0, time.time() - t)
    return '%.0f s ago' % s if s < 90 else '%.0f min ago' % (s / 60) if s < 5400 else '%.1f h ago' % (s / 3600)


class Net:
    """polls the robot's agent and the PC's receiver in the background; the window only reads the newest answers"""

    def __init__(self, agent, receiver, token):
        self.agent, self.receiver, self.token = agent, receiver, token
        self.a, self.a_at, self.a_err, self.a_ms = None, 0.0, 'not tried yet', 0
        self.r, self.r_at, self.r_err = None, 0.0, 'not tried yet'
        threading.Thread(target=self._poll, args=('a', agent + '/status', 0.5), daemon=True).start()
        threading.Thread(target=self._poll, args=('r', receiver + '/state', 0.5), daemon=True).start()
        self.want_cam, self.cam, self.cam_n = False, None, 0       # head-camera frames, fetched only while the window asks
        threading.Thread(target=self._poll_cam, daemon=True).start()

    def _poll_cam(self):
        while True:
            if not self.want_cam:
                self.cam = None; time.sleep(0.4)
                continue
            t0 = time.time()
            try:
                req = urllib.request.Request(self.agent + '/camera.jpg', headers={'X-Orio-Token': self.token})
                with urllib.request.urlopen(req, timeout=2.0) as r:
                    self.cam = r.read(); self.cam_n += 1
            except Exception:
                self.cam = None
            time.sleep(max(0.02, 0.16 - (time.time() - t0)))

    def _poll(self, key, url, period):
        while True:
            t0 = time.time()
            try:
                d = http(url, token=self.token)
                setattr(self, key, d); setattr(self, key + '_at', time.time()); setattr(self, key + '_err', None)
                if key == 'a':
                    self.a_ms = int((time.time() - t0) * 1000)
            except urllib.error.HTTPError as e:
                setattr(self, key + '_err', 'refused (%d): wrong token?' % e.code)
            except Exception as e:
                setattr(self, key + '_err', getattr(e, 'reason', None) and str(e.reason) or str(e))
            time.sleep(max(0.05, period - (time.time() - t0)))

    def robot_up(self):
        return time.time() - self.a_at < 3.0

    def pc_up(self):
        return time.time() - self.r_at < 4.0

    def cmd(self, c, done=None, timeout=20.0):
        def run():
            try:
                rep = http(self.agent + '/cmd', c, self.token, timeout)
            except Exception as e:
                rep = {'ok': False, 'message': 'robot not reachable: %s' % e}
            if done:
                done(c, rep)
        threading.Thread(target=run, daemon=True).start()


class App:
    def __init__(self, root, conf, net, view='2d'):
        self.root, self.conf, self.net, self.start_view = root, conf, net, view
        self.preflight = None            # the last preflight result (the agent only remembers its newest job)
        self.held, self.msgs = {}, []
        self.jog_last, self.jog_msg = {}, None
        self.view = self.scene_data = self.map_sig = self.sel_uid = self.hover_uid = self.mdrag = self._kick_id = self._settle_id = None
        self.cam_was_ok, self.map_view = False, self.start_view
        if scene_view is not None:
            try:
                self.view = scene_view.SceneView(scene_view.Scene(os.path.join(HERE, '..', 'loc', 'site')))
            except Exception as e:           # no scene files for this site: the plain route map is used
                print('map: %r - drawing the route only' % e)
        self.f = {n: tkfont.Font(family='DejaVu Sans', size=s, weight=w) for n, s, w in (
            ('title', 17, 'bold'), ('h', 10, 'bold'), ('b', 10, 'normal'), ('small', 9, 'normal'), ('btn', 12, 'bold'),
            ('big', 20, 'bold'), ('mono', 9, 'normal'))}
        self.f['mono'].configure(family='DejaVu Sans Mono')
        root.title('ORIO scan'); root.configure(bg=CRUST); root.minsize(1000, 700)
        self._build()
        root.bind('<Escape>', lambda e: self.estop())
        root.protocol('WM_DELETE_WINDOW', self.close)
        self._tick(); self._drive_tick()

    # ------------------------------------------------------------------ widgets
    def _btn(self, parent, text, cmd, bg, fg=CRUST, font='btn', **kw):
        return tk.Button(parent, text=text, command=cmd, font=self.f[font], relief='flat', bd=0, bg=bg, fg=fg, activebackground=bg,
                         activeforeground=fg, disabledforeground=OVERLAY, cursor='hand2', padx=kw.pop('padx', 12), pady=kw.pop('pady', 6), **kw)

    def _panel(self, parent, title):
        fr = tk.Frame(parent, bg=MANTLE, highlightbackground=SURF0, highlightthickness=1)
        tk.Label(fr, text=title, font=self.f['h'], bg=MANTLE, fg=SUBTEXT, anchor='w').pack(fill='x', padx=10, pady=(8, 4))
        return fr

    def _build(self):
        c = self.conf
        # ---- top bar
        top = tk.Frame(self.root, bg=BASE); top.pack(fill='x')
        tk.Label(top, text='ORIO  scan', font=self.f['title'], bg=BASE, fg=TEXT).pack(side='left', padx=(14, 10), pady=10)
        self.phase = tk.Label(top, text='IDLE', font=self.f['btn'], bg=SURF2, fg=CRUST, padx=12, pady=4); self.phase.pack(side='left')
        self.sub = tk.Label(top, text='', font=self.f['small'], bg=BASE, fg=SUBTEXT); self.sub.pack(side='left', padx=10)
        self.b_estop = self._btn(top, '■  E-STOP', self.estop, ESTOP, fg='white', padx=20, pady=10); self.b_estop.pack(side='right', padx=(6, 14), pady=8)
        self.b_stop = self._btn(top, 'STOP', lambda: self.send({'cmd': 'stop'}), PEACH); self.b_stop.pack(side='right', padx=4)
        self.b_pause = self._btn(top, 'PAUSE', self.pause, YELLOW); self.b_pause.pack(side='right', padx=4)
        self.b_start = self._btn(top, '▶  START', self.start, GREEN); self.b_start.pack(side='right', padx=4)
        links = tk.Frame(self.root, bg=CRUST); links.pack(fill='x', padx=10, pady=(6, 0))
        self.links = {}
        for key, name in (('robot', 'ROBOT'), ('pc', 'PC RECEIVER'), ('reads', 'READS → PC'), ('loc', 'LOCALISER')):
            self.links[key] = tk.Label(links, text=name, font=self.f['small'], bg=SURF0, fg=TEXT, padx=10, pady=3)
            self.links[key].pack(side='left', padx=(0, 6))
        self.status = tk.Label(links, text='', font=self.f['small'], bg=CRUST, fg=SUBTEXT, anchor='e'); self.status.pack(side='right', fill='x', expand=True)

        body = tk.Frame(self.root, bg=CRUST); body.pack(fill='both', expand=True, padx=10, pady=8)
        body.columnconfigure(1, weight=1); body.rowconfigure(0, weight=1)
        left = tk.Frame(body, bg=CRUST); left.grid(row=0, column=0, sticky='ns')
        mid = tk.Frame(body, bg=CRUST); mid.grid(row=0, column=1, sticky='nsew', padx=8)
        right = tk.Frame(body, bg=CRUST); right.grid(row=0, column=2, sticky='ns')

        # ---- setup
        p = self._panel(left, 'SETUP'); p.pack(fill='x')
        g = tk.Frame(p, bg=MANTLE); g.pack(fill='x', padx=10, pady=(0, 8))
        self.v_src = tk.StringVar(value='robot'); self.v_replay = tk.StringVar()
        self.v_laps = tk.StringVar(value=c.get('LAPS', '1')); self.v_at = tk.StringVar(value=c.get('START_AT', '1'))
        self.v_method = tk.StringVar(value=c.get('METHOD', 'F4'))
        opt = dict(bg=MANTLE, fg=TEXT, selectcolor=SURF0, activebackground=MANTLE, activeforeground=TEXT, font=self.f['b'], highlightthickness=0, anchor='w')
        tk.Radiobutton(g, text='Robot drives (real run)', variable=self.v_src, value='robot', **opt).grid(row=0, column=0, columnspan=4, sticky='w')
        tk.Radiobutton(g, text='Replay an old run (no motion)', variable=self.v_src, value='replay', **opt).grid(row=1, column=0, columnspan=4, sticky='w')
        self.cb_replay = ttk.Combobox(g, textvariable=self.v_replay, state='readonly', width=30, font=self.f['small'])
        self.cb_replay.grid(row=2, column=0, columnspan=4, sticky='we', pady=(0, 6))
        lab = dict(bg=MANTLE, fg=SUBTEXT, font=self.f['small'])
        tk.Label(g, text='Map', **lab).grid(row=3, column=0, sticky='w')
        tk.Label(g, text=c.get('MAP', '?'), bg=MANTLE, fg=TEXT, font=self.f['small']).grid(row=3, column=1, columnspan=3, sticky='w')
        tk.Label(g, text='Laps', **lab).grid(row=4, column=0, sticky='w', pady=2)
        sp = dict(width=4, font=self.f['b'], bg=SURF0, fg=TEXT, buttonbackground=SURF1, relief='flat', insertbackground=TEXT, highlightthickness=0)
        tk.Spinbox(g, from_=1, to=9, textvariable=self.v_laps, **sp).grid(row=4, column=1, sticky='w')
        tk.Label(g, text='Start on', **lab).grid(row=4, column=2, sticky='e', padx=(8, 4))
        tk.Spinbox(g, from_=1, to=20, textvariable=self.v_at, **sp).grid(row=4, column=3, sticky='w')
        tk.Label(g, text='Method', **lab).grid(row=5, column=0, sticky='w')
        mf = tk.Frame(g, bg=MANTLE); mf.grid(row=5, column=1, columnspan=3, sticky='w')
        for m in ('F1', 'F2', 'F3', 'F4'):
            tk.Radiobutton(mf, text=m, variable=self.v_method, value=m, **dict(opt, font=self.f['small'])).pack(side='left')
        bf = tk.Frame(p, bg=MANTLE); bf.pack(fill='x', padx=10, pady=(0, 10))
        self.b_pre = self._btn(bf, 'PREFLIGHT', lambda: self.send({'cmd': 'preflight'}), BLUE, font='h', padx=8); self.b_pre.pack(side='left')
        self.b_home = self._btn(bf, 'HOME', self.home, MAUVE, font='h', padx=8); self.b_home.pack(side='left', padx=6)
        self.b_clean = self._btn(bf, 'CLEAN UP', lambda: self.send({'cmd': 'clean'}), SURF1, fg=TEXT, font='h', padx=8); self.b_clean.pack(side='left')

        # ---- checks
        p = self._panel(left, 'CHECKS'); p.pack(fill='x', pady=8)
        self.checks = {}
        for key, name in (('agent', 'Robot agent'), ('pc', 'PC receiver'), ('teleop', 'Teleop server'), ('pre', 'Preflight'),
                          ('nav', 'Navigation'), ('reader', 'RFID reader'), ('pose', 'Antenna pose'), ('stream', 'Reads at the PC'),
                          ('route', 'Route'), ('presence', 'Presence'), ('engine', 'Localiser'), ('loc', 'Locations')):
            row = tk.Frame(p, bg=MANTLE); row.pack(fill='x', padx=10, pady=1)
            dot = tk.Canvas(row, width=12, height=12, bg=MANTLE, highlightthickness=0); dot.pack(side='left')
            oid = dot.create_oval(2, 2, 11, 11, fill=SURF2, outline='')
            tk.Label(row, text=name, font=self.f['small'], bg=MANTLE, fg=TEXT, width=13, anchor='w').pack(side='left', padx=(4, 0))
            val = tk.Label(row, text='', font=self.f['small'], bg=MANTLE, fg=SUBTEXT, anchor='w', width=24); val.pack(side='left')
            self.checks[key] = (dot, oid, val)
        tk.Frame(p, bg=MANTLE, height=6).pack()

        # ---- manual control (under the map)
        p = self._panel(mid, 'MANUAL CONTROL'); p.pack(side='bottom', fill='x', pady=(8, 0))
        mrow = tk.Frame(p, bg=MANTLE); mrow.pack(fill='x', padx=10)
        self.b_manual = self._btn(mrow, 'MANUAL ON', self.manual, SURF1, fg=TEXT, font='h', padx=8); self.b_manual.pack(side='left')
        self.b_rel = self._btn(mrow, 'RELEASE RUNSTOP', lambda: self.send({'cmd': 'runstop', 'on': False}), SURF1, fg=TEXT, font='h', padx=8)
        self.b_rel.pack(side='left', padx=6)
        self.b_pose = self._btn(mrow, 'POSE AGAIN', self.pose, SURF1, fg=TEXT, font='h', padx=8)
        self.b_pose.pack(side='left')
        pad = tk.Frame(p, bg=MANTLE); pad.pack(padx=10, pady=6, anchor='w')
        self.pad = pad
        for txt, key, r, col in (('↺', 'q', 0, 0), ('▲', 'w', 0, 1), ('↻', 'e', 0, 2), ('◄', 'a', 1, 0), ('▼', 's', 1, 1), ('►', 'd', 1, 2)):
            b = tk.Label(pad, text=txt, font=self.f['btn'], bg=SURF0, fg=TEXT, width=3, pady=4, cursor='hand2')
            b.grid(row=r, column=col, padx=2, pady=2)
            b.bind('<ButtonPress-1>', lambda e, k=key: self._hold(k, True)); b.bind('<ButtonRelease-1>', lambda e, k=key: self._hold(k, False))
        def hold_btn(parent, txt, key, font='h', **kw):          # moves only while pressed, like the base buttons
            b = tk.Label(parent, text=txt, font=self.f[font], bg=SURF0, fg=TEXT, cursor='hand2', **kw)
            b.bind('<ButtonPress-1>', lambda e: self._hold(key, True)); b.bind('<ButtonRelease-1>', lambda e: self._hold(key, False))
            return b
        lf = tk.Frame(pad, bg=MANTLE); lf.grid(row=0, column=3, rowspan=2, padx=(14, 0))
        tk.Label(lf, text='Lift', **lab).pack()
        hold_btn(lf, '▲', 'r', padx=10, pady=2).pack(pady=1)
        self.l_lift = tk.Label(lf, text='– m', font=self.f['small'], bg=MANTLE, fg=TEXT); self.l_lift.pack()
        hold_btn(lf, '▼', 'f', padx=10, pady=2).pack(pady=1)
        self.v_speed = tk.DoubleVar(value=0.6)
        sf = tk.Frame(pad, bg=MANTLE); sf.grid(row=0, column=4, rowspan=2, padx=(14, 0))
        tk.Label(sf, text='Speed', **lab).pack()
        tk.Scale(sf, from_=1.0, to=0.2, resolution=0.1, variable=self.v_speed, orient='vertical', length=70, showvalue=False, bg=MANTLE, fg=TEXT,
                 troughcolor=SURF0, highlightthickness=0, bd=0, sliderlength=16, width=10).pack()
        cf = tk.Frame(pad, bg=MANTLE); cf.grid(row=0, column=5, rowspan=2, padx=(14, 0))          # head camera: turn, tilt, back to usual
        tk.Label(cf, text='Camera', **lab).grid(row=0, column=0, columnspan=3)
        for txt, r, col, key in (('▲', 1, 1, 'i'), ('◄', 2, 0, 'j'), ('►', 2, 2, 'l'), ('▼', 3, 1, 'k')):
            hold_btn(cf, txt, key, font='small', padx=6, pady=0).grid(row=r, column=col, padx=1, pady=1)
        self._btn(cf, '⌂', lambda: self.send({'cmd': 'head', 'home': True}), SURF0, fg=TEXT, font='small', padx=6, pady=0).grid(row=2, column=1, padx=1, pady=1)
        for k in DRIVE_KEYS + JOG_KEYS:
            self.root.bind('<KeyPress-%s>' % k, lambda e, k=k: self._key(k, True)); self.root.bind('<KeyRelease-%s>' % k, lambda e, k=k: self._key(k, False))
        self.l_robot = tk.Label(p, text='', font=self.f['small'], bg=MANTLE, fg=SUBTEXT, anchor='w', justify='left'); self.l_robot.pack(fill='x', padx=10, pady=(0, 8))

        # ---- map: 2D / 3D room with the route and the items, or the head camera in manual mode
        p = tk.Frame(mid, bg=MANTLE, highlightbackground=SURF0, highlightthickness=1); p.pack(fill='both', expand=True)
        head = tk.Frame(p, bg=MANTLE); head.pack(fill='x', padx=10, pady=(8, 4))
        tk.Label(head, text='MAP', font=self.f['h'], bg=MANTLE, fg=SUBTEXT).pack(side='left', padx=(0, 8))
        self.v_view, self.view_btn = tk.StringVar(value=self.start_view), {}
        for key, text in (('2d', '2D'), ('3d', '3D'), ('cam', 'CAMERA')):
            b = tk.Button(head, text=text, command=lambda k=key: self._set_view(k), font=self.f['small'], relief='flat', bd=0, padx=8, pady=1,
                          bg=SURF0, fg=TEXT, activebackground=SURF1, activeforeground=TEXT, disabledforeground=OVERLAY, cursor='hand2', highlightthickness=0)
            b.pack(side='left', padx=(0, 3)); self.view_btn[key] = b
        self.v_cloud, self.v_regions, self.v_truth = tk.BooleanVar(value=True), tk.BooleanVar(value=False), tk.BooleanVar(value=True)
        ck = dict(bg=MANTLE, fg=SUBTEXT, selectcolor=SURF0, activebackground=MANTLE, activeforeground=TEXT, font=self.f['small'], highlightthickness=0,
                  command=lambda: self._render_map())
        opts = tk.Frame(p, bg=MANTLE); opts.pack(fill='x', padx=6)
        for var, text in ((self.v_cloud, 'point cloud'), (self.v_regions, 'uncertainty'), (self.v_truth, 'tape positions')):
            tk.Checkbutton(opts, text=text, variable=var, **ck).pack(side='left')
        self.map = tk.Canvas(p, bg=CRUST, highlightthickness=0, height=300); self.map.pack(fill='both', expand=True, padx=10)
        m = self.map
        m.bind('<ButtonPress-1>', lambda e: self._m_down(e, 1)); m.bind('<ButtonPress-3>', lambda e: self._m_down(e, 3))
        m.bind('<B1-Motion>', self._m_move); m.bind('<B3-Motion>', self._m_move)
        m.bind('<ButtonRelease-1>', self._m_up); m.bind('<ButtonRelease-3>', self._m_up)
        m.bind('<Double-Button-1>', self._m_reset); m.bind('<Motion>', self._m_hover); m.bind('<Leave>', lambda e: self._m_hover(None))
        m.bind('<MouseWheel>', lambda e: self._m_wheel(e, 1 if e.delta > 0 else -1))                    # Windows / macOS
        m.bind('<Button-4>', lambda e: self._m_wheel(e, 1)); m.bind('<Button-5>', lambda e: self._m_wheel(e, -1))   # X11
        self.l_run = tk.Label(p, text='', font=self.f['b'], bg=MANTLE, fg=TEXT, anchor='w'); self.l_run.pack(fill='x', padx=10, pady=(6, 0))
        self.bar = tk.Canvas(p, bg=SURF0, highlightthickness=0, height=8); self.bar.pack(fill='x', padx=10, pady=(4, 10))
        p = self._panel(left, 'LOG'); p.pack(fill='both', expand=True)
        self.log = tk.Text(p, height=5, width=20, bg=CRUST, fg=SUBTEXT, font=self.f['mono'], relief='flat', wrap='none', state='disabled', highlightthickness=0)
        self.log.pack(fill='both', expand=True, padx=10, pady=(0, 10))
        self.log.tag_configure('msg', foreground=YELLOW); self.log.tag_configure('err', foreground=RED)

        # ---- inventory
        p = self._panel(right, 'INVENTORY'); p.pack(fill='both', expand=True)
        nums = tk.Frame(p, bg=MANTLE); nums.pack(fill='x', padx=10)
        self.n = {}
        for i, (key, name, col) in enumerate((('on', 'ON SHELF', GREEN), ('miss', 'NOT SEEN', SUBTEXT), ('loc', 'LOCATED', BLUE))):
            fr = tk.Frame(nums, bg=MANTLE); fr.grid(row=0, column=i, padx=(0, 16), sticky='w')
            self.n[key] = tk.Label(fr, text='–', font=self.f['big'], bg=MANTLE, fg=col); self.n[key].pack(anchor='w')
            self.n[key + '_l'] = tk.Label(fr, text=name, font=self.f['small'], bg=MANTLE, fg=SUBTEXT); self.n[key + '_l'].pack(anchor='w')
        st = ttk.Style(); st.theme_use('clam')
        st.configure('Inv.Treeview', background=CRUST, fieldbackground=CRUST, foreground=TEXT, font=self.f['small'], rowheight=20, borderwidth=0)
        st.configure('Inv.Treeview.Heading', background=SURF0, foreground=SUBTEXT, font=self.f['small'], relief='flat')
        st.map('Inv.Treeview', background=[('selected', SURF1)])
        st.configure('TCombobox', fieldbackground=SURF0, background=SURF1, foreground=TEXT, arrowcolor=TEXT, bordercolor=SURF1,
                     lightcolor=SURF0, darkcolor=SURF0, selectbackground=SURF0, selectforeground=TEXT)
        st.map('TCombobox', fieldbackground=[('readonly', SURF0)], foreground=[('readonly', TEXT)], selectbackground=[('readonly', SURF0)],
               selectforeground=[('readonly', TEXT)])
        for k, v in (('background', SURF0), ('foreground', TEXT), ('selectBackground', SURF2), ('selectForeground', TEXT)):
            self.root.option_add('*TCombobox*Listbox.' + k, v)
        tf = tk.Frame(p, bg=MANTLE); tf.pack(fill='both', expand=True, padx=10, pady=(6, 10))
        self.tree = ttk.Treeview(tf, columns=('item', 'state', 'shelf', 'seen'), show='headings', style='Inv.Treeview', height=20)
        for col, name, w, anchor in (('item', 'Item', 150, 'w'), ('state', 'State', 80, 'w'), ('shelf', 'Shelf', 44, 'center'), ('seen', 'Last seen', 66, 'e')):
            self.tree.heading(col, text=name); self.tree.column(col, width=w, anchor=anchor, stretch=col == 'item')
        sb = ttk.Scrollbar(tf, orient='vertical', command=self.tree.yview); self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side='left', fill='both', expand=True); sb.pack(side='right', fill='y')
        for tag, col in (('on', GREEN), ('wait', OVERLAY), ('miss', RED)):
            self.tree.tag_configure(tag, foreground=col)
        self.tree_sig = None
        self.tree.bind('<<TreeviewSelect>>', self._tree_select)

    # ------------------------------------------------------------------ commands
    def say(self, text, err=False):
        self.msgs.append((time.strftime('%H:%M:%S'), text, err)); del self.msgs[:-6]
        self.status.configure(text=text, fg=RED if err else SUBTEXT)

    def send(self, c, **kw):
        self.net.cmd(c, lambda c, rep: self.root.after(0, self.say, '%s: %s' % (c['cmd'], rep.get('message', rep)), not rep.get('ok')), **kw)

    def run(self):
        return (self.net.a or {}).get('run') if self.net.robot_up() else None

    def start(self):
        c, replay = self.conf, self.v_src.get() == 'replay'
        if replay:
            if not self.v_replay.get():
                return self.say('pick an old run to replay', True)
            return self.send({'cmd': 'start', 'replay_from': self.v_replay.get(), 'speed': 1, 'method': self.v_method.get()})
        if not messagebox.askyesno('Start the run', 'The robot will drive by itself.\n\n• parked on waypoint %s, facing as when it was marked\n'
                                   '• route clear, nobody next to the robot\n• preflight passed and homed this boot\n\n'
                                   'The teleop server is stopped automatically.\n\nStart?' % self.v_at.get(), icon='warning'):
            return
        self.send({'cmd': 'start', 'map': c['MAP'], 'run_dir': c['RUN_DIR'], 'waypoints': c['WAYPOINTS'], 'laps': int(self.v_laps.get()),
                   'start_at': int(self.v_at.get()), 'method': self.v_method.get()})

    def pause(self):
        r = self.run()
        self.send({'cmd': 'resume' if r and r['nav']['paused'] else 'pause'})

    def estop(self):
        self.held.clear()
        self.send({'cmd': 'estop'}, timeout=5.0)

    def home(self):
        if messagebox.askyesno('Home with the payload', 'The lift goes to the very top of the mast and the wrist swings in.\n'
                               'Nothing above the arm, nobody near the robot.\n\nHome now?', icon='warning'):
            self.send({'cmd': 'home'})

    def manual(self):
        m = (self.net.a or {}).get('manual')
        if m and m['on']:
            return self.send({'cmd': 'manual', 'on': False}, timeout=40.0)       # the head goes back to its usual pose first
        if messagebox.askyesno('Manual mode', 'The driver starts (about 20 s). Then, by itself, the lift goes UP to the saved height,\n'
                               'the arm retracts, the wrist swings in (it stops 30 degrees short of stowed) and the head\n'
                               'goes to its usual pose.\n\n'
                               'Nothing above the arm, nobody near it. The teleop server is stopped.\n\nSwitch manual mode on?', icon='warning'):
            self.send({'cmd': 'manual', 'on': True}, timeout=30.0)

    def pose(self):
        if messagebox.askyesno('Pose again', 'The lift goes UP to the saved height first, then the arm retracts, the wrist swings in\n'
                               '(it stops 30 degrees short of stowed) and the head tilts down. Nothing above the arm, nobody near it.\n\nMove now?', icon='warning'):
            self.send({'cmd': 'pose'})

    def _hold(self, key, down):
        if down:
            self.held[key] = time.time()
        else:
            self.held.pop(key, None)
        self._jog()                      # the lift / camera start and stop at once, not at the next tick

    def _key(self, key, down):
        """keys move the robot only while the pointer is over the pad (so typing elsewhere never moves it). X11 repeats a
        held key as release + press pairs: a release only counts if no press follows within 60 ms."""
        x, y = self.root.winfo_pointerxy(); w = self.root.winfo_containing(x, y)
        over = w is not None and (w is self.pad or str(w).startswith(str(self.pad)))
        if down and over:
            self.held[key] = time.time()
        elif not down and key in self.held:
            t = self.held[key]
            self.root.after(60, lambda: (self.held.pop(key, None), self._jog()) if self.held.get(key) == t else None)

    def _jog(self):
        """lift (R / F) and camera (I J K L) buttons that are held, to the robot; once more with zeros when they are let go"""
        h = self.held
        j = {'lift': ('r' in h) - ('f' in h), 'pan': ('j' in h) - ('l' in h), 'tilt': ('i' in h) - ('k' in h)}
        if (any(j.values()) or any(self.jog_last.values())) and ((self.net.a or {}).get('manual') or {}).get('ready'):
            self.net.cmd(dict(j, cmd='jog', speed=self.v_speed.get()), self._jog_reply, timeout=1.0)
        self.jog_last = j

    def _jog_reply(self, c, rep):
        msg = None if rep.get('ok') else rep.get('message', str(rep))
        if msg != self.jog_msg:                               # five replies a second: only a new refusal is shown
            self.jog_msg = msg
            if msg:
                self.root.after(0, self.say, msg, True)

    def _drive_tick(self):
        h = self.held
        if any(k in h for k in DRIVE_KEYS) and (self.net.a or {}).get('can_drive'):
            s = self.v_speed.get()
            v = (V_STEP if 'w' in h else 0.0) - (V_STEP if 's' in h else 0.0)
            w = (W_STEP if 'a' in h or 'q' in h else 0.0) - (W_STEP if 'd' in h or 'e' in h else 0.0)
            self.net.cmd({'cmd': 'drive', 'v': round(v * s, 3), 'w': round(w * s, 3)}, timeout=1.0)
        self._jog()
        self.root.after(200, self._drive_tick)

    def close(self):
        r = self.run()
        if r and r['alive'] and not messagebox.askyesno('Close the window', 'A run is still going. It keeps going on the robot without this window.\n\nClose anyway?'):
            return
        self.root.destroy()

    # ------------------------------------------------------------------ drawing
    def _check(self, key, state, text):
        dot, oid, val = self.checks[key]
        dot.itemconfigure(oid, fill=DOT[state]); val.configure(text=text, fg=RED if state == 'bad' else SUBTEXT)

    def _tick(self):
        try:
            self._draw()
        except Exception as e:           # a drawing error must not stop the buttons from working
            self.status.configure(text='display error: %r' % e, fg=RED)
        self.root.after(250, self._tick)

    def _draw(self):
        net, a, r = self.net, self.net.a or {}, self.net.r or {}
        up, pc = net.robot_up(), net.pc_up()
        run = a.get('run') if up else None
        live = r.get('live') or {}
        alive = bool(run and run['alive'])
        phase = run['phase'] if run else 'idle'
        job = a.get('job') if up else None
        if job and job['name'] == 'preflight':
            self.preflight = job
        # top bar
        self.phase.configure(text=phase.upper() if up else 'NO ROBOT', bg=PHASE_COLOR.get(phase, BLUE) if up else RED)
        self.sub.configure(text=(run['label'] + ('   %d:%02d' % divmod(run['elapsed_s'], 60))) if run else '')
        busy = alive or bool(job and job['running'])
        self.b_start.configure(state='disabled' if busy or not up else 'normal')
        self.b_pause.configure(state='normal' if alive and not run['replay'] else 'disabled', text='RESUME' if run and run['nav']['paused'] else 'PAUSE')
        self.b_stop.configure(state='normal' if alive else 'disabled')
        for b in (self.b_pre, self.b_home, self.b_clean):
            b.configure(state='disabled' if busy or not up else 'normal')
        man = a.get('manual') or {}
        self.b_manual.configure(text='MANUAL OFF' if man.get('on') else 'MANUAL ON', bg=YELLOW if man.get('on') else SURF1, fg=CRUST if man.get('on') else TEXT,
                                state='disabled' if alive or not up else 'normal')
        rep = a.get('replayable') or []
        if tuple(self.cb_replay['values']) != tuple(rep):
            self.cb_replay['values'] = rep
            if rep and not self.v_replay.get():
                self.v_replay.set(rep[-1])
        # link pills
        reads_ok = pc and alive and live.get('reads_run') == run['label'] and live.get('reads_at') and time.time() - live['reads_at'] < 5
        loc_ok = pc and live.get('loc_at') and time.time() - live['loc_at'] < 30
        # the localiser reports to the receiver every round (about 2 s): heard within 20 s = it is running
        eng_age = (r.get('now', 0) - live['engine_at']) if pc and live.get('engine_at') else None
        eng_up = eng_age is not None and eng_age < 20 and not live.get('engine_error')
        eng_wanted = a.get('engine') == 'pc' or (a.get('engine') == 'robot' and alive)
        for key, ok, off in (('robot', up, False), ('pc', pc, False), ('reads', reads_ok, not alive or a.get('engine') != 'pc'),
                             ('loc', eng_up, not (up and pc and eng_wanted))):
            self.links[key].configure(bg=SURF0 if off and not ok else GREEN if ok else RED, fg=TEXT if off and not ok else CRUST)
        # checks
        C = self._check
        C('agent', 'ok' if up else 'bad', '%s, %d ms' % (a.get('host', '?'), net.a_ms) if up else str(net.a_err)[:34])
        C('pc', 'ok' if pc else 'bad', '%d items in the database' % r.get('n_items', 0) if pc else str(net.r_err)[:34])
        n_t = a.get('teleop_server', 0)
        C('teleop', 'off' if not up else 'wait' if n_t else 'ok', '' if not up else 'running (START stops it)' if n_t else 'stopped')
        pf = self.preflight
        if not pf:
            C('pre', 'off', 'not run from here yet')
        elif pf['running']:
            C('pre', 'wait', 'running, %d s' % pf['elapsed_s'])
        elif pf['rc'] == 0:
            C('pre', 'ok', 'passed')
        else:
            fix = [l.strip()[4:].strip() for l in pf['lines'] if l.strip().startswith('FIX')]
            C('pre', 'bad', '%d to fix: %s' % (len(fix), fix[0][:60] if fix else 'see the log'))
        rob = a.get('robot') or {}
        if not run:
            C('nav', 'off', ''); C('reader', 'off', ''); C('pose', 'off', ''); C('stream', 'off', ''); C('route', 'off', '')
        else:
            nav, rd, pose = run['nav'], run['reads'], run['pose']
            if run['replay']:
                C('nav', 'off', 'replay: no navigation')
            elif rob.get('localized'):
                C('nav', 'ok', 'localised in the map')
            elif rob.get('seeded'):
                C('nav', 'ok', 'on odometry from the start pose')
            else:
                C('nav', 'wait' if alive else 'off', phase if alive else '')
            started = rd['rows'] > 0
            C('reader', ('ok' if rd['rate'] > 100 else 'bad' if started else 'wait') if alive else 'off',
              '%d reads/s, %d tags' % (rd['rate'], rd['tags']) if alive else '%d reads, %d tags' % (rd['rows'], rd['tags']))
            C('pose', ('ok' if pose and pose['age_s'] < 3 else 'bad' if started else 'wait') if alive else 'off',
              'antenna %.2f m up, %.1f s old' % (pose['ant_z'], pose['age_s']) if pose and alive else '')
            if a.get('engine') != 'pc':
                C('stream', 'off', 'not sent (localiser: %s)' % a.get('engine'))
            elif live.get('reads_run') == run['label']:
                lag = rd['rows'] - live.get('reads_rows', 0)
                C('stream', 'ok' if lag < 3000 else 'wait', '%d rows, %d behind' % (live.get('reads_rows', 0), max(lag, 0)))
            else:
                C('stream', 'wait' if alive and not started else 'bad', 'nothing received for this run')
            C('route', 'bad' if nav['fail'] else 'ok' if nav['done'] or nav['i'] else 'wait',
              '%d of %d reached%s%s' % (nav['ok'], nav['n'], ', %d FAILED' % nav['fail'] if nav['fail'] else '', ', %d retries' % nav['retries'] if nav['retries'] else ''))
        if pc:
            n_on, n_all = r.get('n_on_shelf', 0), r.get('n_items', 0)
            C('presence', 'ok' if n_on else 'wait' if alive else 'off', '%d of %d on shelf, update %s' % (n_on, n_all, ago(live.get('sightings_at'))))
            if not up or not eng_wanted:
                C('engine', 'off', 'off' if up and a.get('engine') == 'off' else 'starts with a run, on the robot' if up and a.get('engine') == 'robot' else '')
            elif eng_up:
                C('engine', 'ok', (live.get('engine_note') or 'running')[:60])
            elif eng_age is None:
                C('engine', 'bad', 'NOT RUNNING (never heard from it): logs/engine.log')
            else:
                C('engine', 'bad', ('%s' if live.get('engine_error') else 'NOT RUNNING, last heard %s: %%s' % ago(live['engine_at'])) % (live.get('engine_note') or '')[:70])
            # reads are flowing but no positions have come for half a minute: the localiser is down (or far behind)
            silent = alive and a.get('engine') != 'off' and run['reads']['rows'] > 5000 and not loc_ok
            C('loc', 'bad' if silent else 'ok' if loc_ok else 'wait' if alive and a.get('engine') != 'off' else 'off',
              'NO UPDATE for 30 s: see logs/engine.log' if silent else
              '%d placed (%s), %s' % (r.get('n_located', 0), live.get('loc_method') or '–', ago(live.get('loc_at'))) if live.get('loc_at') else 'localiser has not reported')
        else:
            C('presence', 'off', ''); C('loc', 'off', ''); C('engine', 'off', '')
        # manual + robot line
        self.l_lift.configure(text='%.2f m' % rob['lift_m'] if rob.get('lift_m') is not None else '– m')
        bits = []
        if man.get('on'):
            bits.append('manual: ' + ('moving ...' if man.get('moving') else 'ready, hold a pad button' if man.get('ready') else 'driver starting ...'))
        if man.get('on') and rob.get('arm_m') is not None and rob['arm_m'] > 0.05:
            bits.append('arm out %.2f m: the lift buttons wait for the pose move' % rob['arm_m'])
        for j, why in (rob.get('jog_blocked') or {}).items() if man.get('on') else ():
            bits.append('%s stopped: %s' % ('lift' if j == 'lift' else 'camera', why))
        if man.get('on') and rob.get('lift_m') is not None and rob['lift_m'] < 0.42:
            bits.append('lift below 0.42 m: watch the panel over the base')
        if man.get('on') and rob.get('head_pan') is not None:
            bits.append('camera pan %+.2f  tilt %+.2f rad' % (rob['head_pan'], rob['head_tilt']))
        if rob:
            bits.append('battery %s V   mode %s%s' % (rob.get('battery_v', '?'), rob.get('mode', '?'), '   RUNSTOP ON' if rob.get('runstop') else ''))
        self.l_robot.configure(text='\n'.join(bits), fg=RED if rob.get('runstop') else SUBTEXT)
        self.b_rel.configure(state='normal' if rob.get('runstop') else 'disabled')
        self.b_pose.configure(state='normal' if man.get('ready') and not man.get('moving') else 'disabled')
        cam_ok = bool(man.get('ready')) and not alive
        if cam_ok and not self.cam_was_ok:
            self.v_view.set('cam')                       # manual mode just became ready: show the camera
        elif not cam_ok and self.v_view.get() == 'cam':
            self.v_view.set(self.map_view)
        self.cam_was_ok = cam_ok
        self.net.want_cam = cam_ok and self.v_view.get() == 'cam'
        for key, b in self.view_btn.items():
            on = self.v_view.get() == key
            b.configure(bg=BLUE if on else SURF0, fg=CRUST if on else TEXT, state='normal' if (cam_ok if key == 'cam' else self.view is not None) else 'disabled')
        self._draw_map(run, r if pc else None, alive)
        self._draw_inventory(r if pc else None, alive, run)
        self._draw_log(a if up else {}, run, job)

    def _draw_camera(self, c, W, H):
        """manual mode: the head camera in the route panel"""
        self.l_run.configure(text='head camera (manual mode)', fg=TEXT); self.bar.delete('all')
        self.map_sig = None                              # the map is redrawn when the view goes back to it
        if Image is None:
            c.delete('all'); c.create_text(W / 2, H / 2, text='The camera picture needs Pillow (pip install pillow).', fill=OVERLAY, font=self.f['b'])
            return
        if self.net.cam is None:
            c.delete('all'); self.cam_shown = None
            c.create_text(W / 2, H / 2, text='Waiting for the camera ...', fill=OVERLAY, font=self.f['b'])
            return
        if getattr(self, 'cam_shown', None) == (self.net.cam_n, W, H):
            return                                     # same frame, same size: leave the picture as it is
        try:
            im = Image.open(io.BytesIO(self.net.cam)); k = min(W / im.width, H / im.height)
            self.cam_img = ImageTk.PhotoImage(im.resize((max(1, int(im.width * k)), max(1, int(im.height * k)))))
        except Exception:
            return
        c.delete('all'); c.create_image(W / 2, H / 2, image=self.cam_img); self.cam_shown = (self.net.cam_n, W, H)

    def _run_line(self, run):
        """the line and the progress bar under the map"""
        self.bar.delete('all')
        if not run:
            return self.l_run.configure(text='')
        nav = run['nav']; goal = ('at %s' % nav['name']) if nav['name'] else ''
        self.l_run.configure(text='waypoint %d of %d  %s      %d reads, %d tags%s' % (
            nav['i'], nav['n'], goal, run['reads']['rows'], run['reads']['tags'], '      ' + run['error'][:70] if run.get('error') else ''),
            fg=RED if run.get('error') else TEXT)
        if nav['n']:
            self.bar.create_rectangle(0, 0, max(self.bar.winfo_width(), 10) * (nav['ok'] + nav['fail']) / nav['n'], 8, fill=RED if nav['fail'] else GREEN, outline='')

    def _draw_map(self, run, r, alive):
        c = self.map
        W, H = max(c.winfo_width(), 50), max(c.winfo_height(), 50)
        self._run_line(run)
        if self.net.want_cam:
            return self._draw_camera(c, W, H)
        self.cam_shown = None
        if self.view is None:
            return self._draw_route_plain(c, W, H, run)
        data = self._scene_data(run, r, alive)
        sig = (W, H, self.v_view.get(), tuple((i['uid'], i['x'], i['y'], i['z'], i['state']) for i in data['items']),
               data['robot'] and tuple(round(v, 2) for v in data['robot'][:4]) + (data['robot'][4],), 0 if data['path'] is None else len(data['path']),
               tuple(w[3] for w in data['waypoints']), tuple(data['hud']))
        self.scene_data = data
        if sig != self.map_sig:
            self.map_sig = sig; self._render_map()

    def _scene_data(self, run, r, alive):
        """what the map shows, in the room frame: items from the PC's database, robot / path / waypoints from the robot (map frame -> room)"""
        sc, items, started = self.view.s, [], (r or {}).get('scan_started') or 0
        scanned = bool(run) and started > 0

        def add(i, listed):
            if i.get('x_m') is None:
                return
            fresh = started > 0 and (i.get('loc_updated_s') or i.get('updated_s') or 0) >= started - 1
            state = 'new' if fresh else 'missing' if listed and scanned and not alive and not i.get('is_on_shelf') else 'old'
            note = '%s · ±%.2f m · %s reads · %s' % ('S%d' % i['shelf'] if i.get('shelf') is not None else 'no shelf', i.get('region90_m') or 0, i.get('n_reads'), i.get('method'))
            items.append(dict(uid=i['rfid_uid'], name=i.get('item_name') or i['rfid_uid'][-8:], x=i['x_m'], y=i['y_m'], z=i['z_m'], shelf=i.get('shelf'),
                              r90=i.get('region90_m'), state=state,
                              note=note if state == 'new' else ('MISSING · last known place' if state == 'missing' else 'last known place, from an earlier scan')))
        for i in (r or {}).get('items') or ():
            add(i, True)
        for i in (r or {}).get('located_other') or ():
            add(i, False)
        robot = path = None; wps = []
        if run:
            p, nav = run['pose'], run['nav']
            if p:
                x, y = sc.room_xy([[p['base_x'], p['base_y']]])[0]; robot = (float(x), float(y), p['base_yaw'] + sc.yaw0, p['ant_z'], bool(run['alive']))
            if len(run['path']) > 1:
                path = sc.room_xy(run['path'])
            if run['waypoints']:
                seen = set()
                for k, ((x, y), w) in enumerate(zip(sc.room_xy([[w['x'], w['y']] for w in run['waypoints']]), run['waypoints'])):
                    res = next((q for q in nav['results'] if q['i'] == k + 1), None)
                    st = ('ok' if res['ok'] else 'fail') if res else 'goal' if nav['i'] == k + 1 and run['alive'] else ''
                    key = (round(w['x'], 1), round(w['y'], 1)); name = '' if key in seen else w['name'].split('_')[0]; seen.add(key)
                    wps.append((float(x), float(y), name, st))
        n_new = sum(1 for i in items if i['state'] == 'new'); n_old = len(items) - n_new
        hud = ['%s · %d items placed in this scan%s' % (((r or {}).get('live') or {}).get('loc_method') or 'no method yet', n_new,
                                                        ' · %d at their last known place' % n_old if n_old else '')]
        return dict(items=items, robot=robot, path=path, waypoints=wps, hud=hud)

    def _render_map(self, fast=False):
        if self.view is None or self.scene_data is None or self.net.want_cam or ImageTk is None:
            return
        c = self.map; W, H = max(c.winfo_width(), 50), max(c.winfo_height(), 50)
        self.view.mode = '3d' if self.v_view.get() == '3d' else '2d'
        data = dict(self.scene_data, selected=self.sel_uid, hover=self.hover_uid, cloud=self.v_cloud.get(), regions=self.v_regions.get(),
                    truth=self.v_truth.get(), fast=fast)
        if data['truth']:                # as on the replay page: how far each estimate is from its tape position, and the median
            tr, errs = self.view.s.truth, []
            items = []
            for i in data['items']:
                if i['state'] == 'new' and i['uid'] in tr:
                    e = math.dist((i['x'], i['y'], i['z']), tr[i['uid']]); errs.append(e); i = dict(i, note='%s · %.0f cm from the tape position' % (i['note'], 100 * e))
                items.append(i)
            data['items'] = items
            if errs:
                errs.sort(); n = len(errs); med = errs[n // 2] if n % 2 else (errs[n // 2 - 1] + errs[n // 2]) / 2
                data['hud'] = data['hud'] + ['median 3D error now %.0f cm (%d items with a tape position)' % (100 * med, n)]
        try:
            im = self.view.render(W, H, data)
        except Exception as e:
            return self.status.configure(text='map error: %r' % e, fg=RED)
        self.map_img = ImageTk.PhotoImage(im); c.delete('all'); c.create_image(0, 0, anchor='nw', image=self.map_img)

    def _kick(self, fast=False):
        """redraw the map soon (at most about 60 times a second); after a fast (thinned) picture a full one follows"""
        if self._kick_id is None:
            def go():
                self._kick_id = None; self._render_map(fast)
            self._kick_id = self.root.after(15, go)
        if fast:
            if self._settle_id is not None:
                self.root.after_cancel(self._settle_id)

            def settle():
                self._settle_id = None; self._render_map()
            self._settle_id = self.root.after(350, settle)

    def _set_view(self, key):
        self.v_view.set(key)
        if key != 'cam':
            self.map_view = key
        self.map_sig = None                              # the next tick draws it; the buttons recolour there too
        self._kick()

    def _map_live(self):
        return self.view is not None and not self.net.want_cam

    def _m_down(self, e, button):
        self.mdrag = [e.x, e.y, 0, button]

    def _m_move(self, e):
        if not self.mdrag or not self._map_live():
            return
        dx, dy = e.x - self.mdrag[0], e.y - self.mdrag[1]; self.mdrag[0], self.mdrag[1] = e.x, e.y; self.mdrag[2] += abs(dx) + abs(dy)
        self.view.drag(dx, dy, pan=self.mdrag[3] == 3 or bool(e.state & 0x0001)); self._kick(fast=True)

    def _m_up(self, e):
        d, self.mdrag = self.mdrag, None
        if d and d[2] < 4 and d[3] == 1 and self._map_live():     # a click, not a drag: pick the item under it
            self._select(self.view.pick(e.x, e.y))

    def _m_wheel(self, e, steps):
        if self._map_live():
            self.view.zoom(steps, e.x, e.y); self._kick(fast=True)

    def _m_reset(self, e):
        if self._map_live():
            self.view.reset(); self._kick()

    def _m_hover(self, e):
        uid = self.view.pick(e.x, e.y) if e is not None and self._map_live() and not self.mdrag else None
        if uid != self.hover_uid:
            self.hover_uid = uid; self._kick()

    def _select(self, uid):
        self.sel_uid = uid
        if uid and self.tree.exists(uid):
            self.tree.selection_set(uid); self.tree.see(uid)
        elif self.tree.selection():
            self.tree.selection_remove(*self.tree.selection())
        self._kick()

    def _tree_select(self, _e):
        sel = self.tree.selection(); uid = sel[0] if sel else None
        if uid != self.sel_uid:
            self.sel_uid = uid; self._kick()

    def _draw_route_plain(self, c, W, H, run):
        """without numpy / Pillow or without scene files: the route only, in the robot's map frame"""
        c.delete('all')
        if not run or not run['waypoints']:
            c.create_text(W / 2, H / 2, text='The route appears here when a run starts.', fill=OVERLAY, font=self.f['b'])
            return
        pts = [(w['x'], w['y']) for w in run['waypoints']] + [tuple(p) for p in run['path']]
        x0, x1 = min(p[0] for p in pts) - 0.8, max(p[0] for p in pts) + 0.8
        y0, y1 = min(p[1] for p in pts) - 0.8, max(p[1] for p in pts) + 0.8
        s = min(W / (x1 - x0), H / (y1 - y0))
        ox, oy = (W - s * (x1 - x0)) / 2, (H - s * (y1 - y0)) / 2

        def P(x, y):                      # map x right, map y up
            return ox + (x - x0) * s, H - oy - (y - y0) * s
        for gx in range(math.floor(x0), math.ceil(x1) + 1):
            c.create_line(*P(gx, y0), *P(gx, y1), fill=MANTLE)
        for gy in range(math.floor(y0), math.ceil(y1) + 1):
            c.create_line(*P(x0, gy), *P(x1, gy), fill=MANTLE)
        nav = run['nav']
        if len(run['path']) > 1:
            c.create_line(*[v for p in run['path'] for v in P(*p)], fill=SKY, width=2)
        seen = {}
        for i, w in enumerate(run['waypoints']):
            res = next((x for x in nav['results'] if x['i'] == i + 1), None)
            col = (GREEN if res['ok'] else RED) if res else YELLOW if nav['i'] == i + 1 and run['alive'] else SURF2
            x, y = P(w['x'], w['y']); k = (round(w['x'], 1), round(w['y'], 1)); seen[k] = seen.get(k, 0) + 1
            if seen[k] == 1:
                c.create_text(x, y - 16, text=w['name'].split('_')[0], fill=SUBTEXT, font=self.f['small'])
            rad = 7 if seen[k] == 1 else 4
            c.create_oval(x - rad, y - rad, x + rad, y + rad, fill=col if res or col == YELLOW else '', outline=col, width=2)
        p = run['pose']
        if p:
            x, y = P(p['base_x'], p['base_y']); th = p['base_yaw']
            tri = [(x + 13 * math.cos(th + d), y - 13 * math.sin(th + d)) for d in (0, 2.5, -2.5)]
            c.create_polygon(*[v for q in tri for v in q], fill=BLUE if run['alive'] else SURF2, outline=CRUST)
        c.create_text(8, H - 8, anchor='sw', text='1 m grid · route only (the 2D / 3D map needs numpy and Pillow)', fill=OVERLAY, font=self.f['small'])

    def _draw_inventory(self, r, alive, run):
        if r is None:
            return
        items = r['items']; started = r.get('scan_started') or 0
        scanned = bool(run) and started > 0
        n_on = sum(1 for i in items if i['is_on_shelf'])
        self.n['on'].configure(text=str(n_on)); self.n['loc'].configure(text=str(r.get('n_located', 0)))
        self.n['miss'].configure(text=str(len(items) - n_on) if scanned else '–', fg=RED if scanned and not alive and len(items) - n_on else SUBTEXT)
        self.n['miss_l'].configure(text='MISSING' if scanned and not alive else 'NOT SEEN YET')
        rows = []
        for i in items:
            if i['is_on_shelf']:
                state, tag = 'on shelf', 'on'
            elif scanned and not alive:
                state, tag = 'MISSING', 'miss'
            else:
                state, tag = 'not seen yet' if scanned else '–', 'wait'
            shelf = ('S%d' % i['shelf']) if i.get('shelf') is not None else ('·' if i.get('x_m') is not None else '')
            rows.append((0 if tag == 'miss' else 1 if tag == 'on' else 2, i['item_name'] or i['rfid_uid'][-8:], state, shelf,
                         (i['last_seen'] or '')[11:], tag, i['rfid_uid']))
        rows.sort(key=lambda x: (x[0], x[1]))
        sig = tuple((x[6], x[2], x[3], x[4]) for x in rows)
        if sig == self.tree_sig:
            return
        self.tree_sig = sig
        top = self.tree.yview()[0]
        self.tree.delete(*self.tree.get_children())
        for x in rows:
            self.tree.insert('', 'end', iid=x[6], values=x[1:5], tags=(x[5],))
        self.tree.yview_moveto(top)
        if self.sel_uid and self.tree.exists(self.sel_uid):
            self.tree.selection_set(self.sel_uid)

    def _draw_log(self, a, run, job):
        lines = []
        if job and (job['running'] or not run or not run['alive']):
            lines += [('', l) for l in job['lines'][-7:]]
        elif run:
            lines += [('', l) for l in run['log'][-7:]]
        man = a.get('manual')
        if man and man['on'] and not man['ready']:
            lines += [('', l) for l in man['lines'][-3:]]
        lines += [('err' if e else 'msg', '%s  %s' % (t, m)) for t, m, e in self.msgs[-3:]]
        text = '\n'.join(l for _, l in lines)
        if text == getattr(self, '_log_text', None):
            return
        self._log_text = text
        self.log.configure(state='normal'); self.log.delete('1.0', 'end')
        for tag, l in lines[-9:]:
            self.log.insert('end', l[:160] + '\n', tag)
        self.log.configure(state='disabled')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--conf', default=os.path.join(HERE, 'demo.conf'))
    ap.add_argument('--agent', default=None, help='robot agent address (default: from the conf file)')
    ap.add_argument('--receiver', default=None, help='pc_receiver.py address (default: this PC, port from the conf file)')
    ap.add_argument('--token', default=os.environ.get('ORIO_DEMO_TOKEN', ''))
    ap.add_argument('--geometry', default='1280x800')
    ap.add_argument('--view', default='2d', choices=['2d', '3d'], help='the map view to open with')
    args = ap.parse_args()
    conf = read_conf(args.conf)
    agent = (args.agent or 'http://%s:%s' % (conf['ROBOT_HOST'], conf.get('AGENT_PORT', '8123'))).rstrip('/')
    receiver = (args.receiver or 'http://127.0.0.1:%s' % conf.get('RECEIVER_PORT', '8000')).rstrip('/')
    root = tk.Tk(); root.geometry(args.geometry)
    App(root, conf, Net(agent, receiver, args.token), args.view)
    root.mainloop()


if __name__ == '__main__':
    main()
