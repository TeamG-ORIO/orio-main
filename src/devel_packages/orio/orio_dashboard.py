#!/usr/bin/env python3
"""Live dashboard for full_sequence.py: both arms, both label zones, and a Pause button.

full_sequence.py opens it at start (unless --no-dashboard) and closes it at the end.
To open it again by hand, inside the main container:
    cd /home/ros_ws/src/devel_packages/orio && python3 orio_dashboard.py

Reads the JSON status full_sequence.py publishes on /orio/dashboard/status and calls
/orio/pause and /orio/resume. The space bar also toggles pause.

Also shows the newest grasp panel (from the grasp planner) and label placement image
(from perception) of this run; click one to enlarge it. Each arm card has LOCK and
UNLOCK & PRIME buttons, which run lock_arms.py through desk_helper.py on the host
(run_full_demo.sh starts it).
"""

import argparse
import json
import os
import re
import signal
import threading
import time
import tkinter as tk
import urllib.error
import urllib.request
from tkinter import font as tkfont, messagebox

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from PIL import Image, ImageTk
except ImportError:  # the debug images then show a note instead
    Image = ImageTk = None

STATUS_TOPIC = '/orio/dashboard/status'
STALE_S = 2.0      # no status for this long = not connected
TICK_MS = 100      # redraw period

# Debug images: the newest PNG in each directory (the repo is mounted at /home/ros_ws).
GRASP_DIR = '/home/ros_ws/logging/demo/current.affordance'
LABEL_DIR = '/home/ros_ws/logging/demo/current.labels'
is_grasp_image = lambda name: name.endswith('_camera.png') or name.startswith('afford_')  # suction / DexNet
is_label_image = lambda name: name.startswith('label_') and name.endswith('.png')
POLL_S = 1.0       # how often the image directories are checked
SETTLE_S = 0.5     # an image this new may still be being written

# Brakes, through desk_helper.py on the host (the container cannot reach Desk).
DESK_HELPER = 'http://127.0.0.1:%d' % int(os.environ.get('ORIO_DESK_HELPER_PORT', '8766'))
DESK_TIMEOUT_S = 330   # an unlock that restarts the robot takes a few minutes
DESK_STATUS_S = 5.0   # how often the dashboard asks desk_helper.py for the brake state
DESK_STALE_S = 60.0   # older than this = brake state unknown
ARM_ROBOT = {'arm1': '1', 'arm2': '2'}   # lock_arms.py robot numbers
ROBOT_PREFIX = re.compile(r'^\[robot \d+ [^\]]*\]\s*')

# Catppuccin Mocha, as in the recovery and item setup windows
CRUST, MANTLE, BASE = '#11111b', '#181825', '#1e1e2e'
SURF0, SURF1, SURF2 = '#313244', '#45475a', '#585b70'
OVERLAY, SUBTEXT, TEXT = '#6c7086', '#a6adc8', '#cdd6f4'
BLUE, MAUVE, GREEN, YELLOW = '#89b4fa', '#cba6f7', '#a6e3a1', '#f9e2af'
PEACH, RED, TEAL, SKY = '#fab387', '#f38ba8', '#94e2d5', '#89dceb'
CARDBOARD, CARDBOARD_DARK = '#c8a27a', '#9c7a56'
ESTOP = '#d20f39'
FORCE_QUIT_AFTER_S = 3.0   # E-STOP turns into FORCE QUIT if the run is still up after this

ARM_COLOR = {'arm1': BLUE, 'arm2': MAUVE}
ARM_TITLE = {'arm1': 'ARM 1', 'arm2': 'ARM 2'}
ARM_ROLE = {'arm1': 'Pick & place', 'arm2': 'Labelling'}
ARM_STEPS = {
    'arm1': [('FETCH_INPUT', 'Bin pick'), ('DROP_TO_ZONE', 'To zone'),
             ('RETRIEVE', 'Retrieve'), ('PLACE_OUTPUT', 'Output')],
    'arm2': [('GET_LABEL', 'Get label'), ('APPLY', 'Apply label')],
}
STATE_NAME = {
    'DECIDE_HUB': 'Choosing next task', 'FETCH_INPUT': 'Picking from the bin',
    'DROP_TO_ZONE': 'Placing in a zone', 'RETRIEVE': 'Taking from a zone',
    'PLACE_OUTPUT': 'Placing in the output', 'HUMAN_RECOVERY_ARM1': 'Recovery',
    'GET_LABEL': 'Getting a label', 'APPLY': 'Applying a label',
    'HUMAN_RECOVERY_ARM2': 'Recovery',
}
ZONE_LOOK = {   # state -> (text, fill, outline)
    'EMPTY': ('Empty', SURF0, SURF1),
    'NEEDS_LABEL': ('Needs a label', '#3a3426', YELLOW),
    'READY_FOR_PICKUP': ('Labelled, ready', '#253a2c', GREEN),
}


# ── Drawing helpers ──────────────────────────────────────────────────────────

def rrect(c, x0, y0, x1, y1, r, **kw):
    """Rounded rectangle on canvas c."""
    r = max(0, min(r, (x1 - x0) / 2, (y1 - y0) / 2))
    pts = [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1,
           x1 - r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0]
    return c.create_polygon(pts, smooth=True, **kw)


def pill(c, x, y, text, fg, bg, font, anchor='e', padx=10, h=22):
    """A rounded label; (x, y) is its right (anchor 'e') or left ('w') middle."""
    w = font.measure(text) + 2 * padx
    x0 = x - w if anchor == 'e' else x
    rrect(c, x0, y - h / 2, x0 + w, y + h / 2, h / 2, fill=bg, outline='')
    c.create_text(x0 + w / 2, y, text=text, fill=fg, font=font)
    return w


def box(c, cx, cy, s, label=False):
    """A small cardboard item, with a white label on it if `label`."""
    rrect(c, cx - s, cy - s * 0.7, cx + s, cy + s * 0.7, s * 0.2,
          fill=CARDBOARD, outline=CARDBOARD_DARK, width=2)
    c.create_line(cx - s, cy - s * 0.25, cx + s, cy - s * 0.25, fill=CARDBOARD_DARK, width=2)
    if label:
        rrect(c, cx - s * 0.55, cy - s * 0.05, cx + s * 0.55, cy + s * 0.5, 4,
              fill='#f5f5f5', outline='')
        for k in (0.12, 0.3):
            c.create_line(cx - s * 0.4, cy + s * k, cx + s * 0.25, cy + s * k, fill=SURF2, width=2)


def sticker(c, cx, cy, s):
    """A label sticker."""
    rrect(c, cx - s, cy - s * 0.6, cx + s, cy + s * 0.6, 5, fill='#f5f5f5', outline=SURF2)
    for k in (-0.25, 0.05, 0.32):
        c.create_line(cx - s * 0.7, cy + s * k, cx + s * (0.5 if k < 0.3 else 0.1), cy + s * k,
                      fill=SURF2, width=2)


def shorten(text, font, width):
    if font.measure(text) <= width:
        return text
    while text and font.measure(text + '…') > width:
        text = text[:-1]
    return text + '…'


def ago(seconds):
    seconds = max(0, int(seconds))
    if seconds < 60:
        return "%d s ago" % seconds
    if seconds < 3600:
        return "%d min ago" % (seconds // 60)
    return "%d h ago" % (seconds // 3600)


# ── Debug images ─────────────────────────────────────────────────────────────

class ImageFeed:
    """The newest image in a directory whose name passes `wanted`. poll() runs off the Tk
    thread; images older than `since` (an earlier run's) are ignored."""

    def __init__(self, directory, wanted):
        self.dir, self.wanted = directory, wanted
        self.since = time.time()
        self._lock = threading.Lock()
        self._current = (0, None, 0.0, None)   # version, path, mtime, PIL image

    def get(self):
        with self._lock:
            return self._current

    def poll(self):
        if Image is None:
            return
        try:
            files = [(e.stat().st_mtime, e.path) for e in os.scandir(self.dir)
                     if self.wanted(e.name) and e.is_file()]
        except OSError:
            return   # directory missing (panels off) or file removed mid-scan
        if not files:
            return
        mtime, path = max(files)
        version, cur_path, cur_mtime, _ = self.get()
        if mtime < self.since or time.time() - mtime < SETTLE_S or (path, mtime) == (cur_path, cur_mtime):
            return
        try:
            with Image.open(path) as im:
                im = im.convert('RGB')
        except Exception:  # noqa: BLE001 — half-written: try again next poll
            return
        with self._lock:
            self._current = (version + 1, path, mtime, im)


class FeedView:
    """Draws an ImageFeed's newest image, scaled to fit, on a canvas."""

    def __init__(self, dash, canvas, feed, title, color, empty, hint, verdict=None):
        self.dash, self.c, self.feed = dash, canvas, feed
        self.title, self.color, self.empty, self.hint = title, color, empty, hint
        self.verdict = verdict   # () -> {'t', 'ok', 'msg'} of the latest attempt, or None
        self.photo, self.key = None, None

    def draw(self, t):
        c, d = self.c, self.dash
        c.delete('all')
        W, H = max(c.winfo_width(), 120), max(c.winfo_height(), 80)
        version, path, mtime, im = self.feed.get()
        fresh = im is not None and t - mtime < 3.0
        rrect(c, 1, 1, W - 1, H - 1, 14, fill=MANTLE, outline=self.color if fresh else '',
              width=2)
        c.create_text(14, 16, text=self.title, anchor='w', fill=self.color, font=d.f_smallb)
        if im is None:
            c.create_text(W / 2, H / 2 + 8, text=self.empty, fill=OVERLAY, font=d.f_small,
                          width=W - 40, justify='center')
            return
        left = 14 + d.f_smallb.measure(self.title) + 16
        c.create_text(W - 14, 16, anchor='e', fill=SUBTEXT, font=d.f_small,
                      text=shorten("%s · %s" % (ago(t - mtime), self.hint), d.f_small, W - left - 14))
        # The sequence turned the latest attempt down after this picture was drawn (e.g.
        # perception's tilt limit): say so in a bar under the picture.
        v = self.verdict() if self.verdict else None
        rejected = bool(v) and not v.get('ok') and v.get('t', 0) >= mtime - 1.0
        bw, bh = W - 16, H - 38 - (30 if rejected else 0)
        if bw < 10 or bh < 10:
            return
        if self.key != (version, bw, bh):
            scale = min(bw / im.width, bh / im.height)
            size = (max(1, int(im.width * scale)), max(1, int(im.height * scale)))
            self.photo = ImageTk.PhotoImage(im.resize(size, Image.LANCZOS), master=c)
            self.key = (version, bw, bh)
        c.create_image(W / 2, 30 + bh / 2, image=self.photo)

        if rejected:
            why = re.sub(r'^rejected( grasp)?[:,]?\s*', '', v.get('msg', ''), flags=re.I)
            rrect(c, 8, H - 34, W - 8, H - 8, 8, fill='#c62828', outline='')
            c.create_text(W / 2, H - 21, fill='white', font=d.f_smallb,
                          text=shorten("REJECTED · %s" % why, d.f_smallb, W - 32))
            rrect(c, 1, 1, W - 1, H - 1, 14, fill='', outline=RED, width=2)


# ── Dashboard ────────────────────────────────────────────────────────────────

class Dashboard:
    def __init__(self, root, parent_pid=None, grasp_dir=GRASP_DIR, label_dir=LABEL_DIR):
        self.root = root
        self.parent_pid = parent_pid
        self.status = None
        self.rx_time = 0.0
        self.pending = False      # a pause/resume call in flight
        self.estop_seen = None    # when the status first showed an E-stop
        self.item_name = None
        self.item_photo = None
        self.t0 = time.time()

        # Debug image feeds: (key, feed, title, colour, text when there is none yet)
        grasp_none = ("No grasp plan yet this run.\nPanels are off with --no-affordance "
                      "or --no-logging.")
        label_none = "No label placement yet this run."
        if Image is None:
            grasp_none = label_none = "Pillow is not installed: no images."
        self.feeds = [
            ('grasp', ImageFeed(grasp_dir, is_grasp_image), "GRASP PLAN", BLUE, grasp_none),
            ('label', ImageFeed(label_dir, is_label_image), "LABEL PLACEMENT", MAUVE, label_none),
        ]
        self.viewers = {}         # key -> (Toplevel, FeedView), the enlarged images

        # Brakes: per arm, what the last lock/unlock did (written by worker threads)
        self.desk_lock = threading.Lock()
        self.desk_state = {arm: {'busy': None, 'ok': None, 'msg': '', 't': 0.0}
                           for arm in ARM_ROBOT}
        self.helper_up = None     # desk_helper.py answering? None = not checked yet
        self.desk_robots = {}     # robot number -> Desk state from desk_helper.py /status

        fam = 'DejaVu Sans' if 'DejaVu Sans' in tkfont.families(root) else 'Helvetica'
        F = lambda size, weight='normal': tkfont.Font(root=root, family=fam, size=size, weight=weight)
        self.f_logo, self.f_h1, self.f_h2 = F(22, 'bold'), F(20, 'bold'), F(14, 'bold')
        self.f_body, self.f_bodyb, self.f_small = F(11), F(11, 'bold'), F(9)
        self.f_smallb, self.f_num, self.f_btn = F(9, 'bold'), F(26, 'bold'), F(18, 'bold')

        self._build()
        rospy.Subscriber(STATUS_TOPIC, String, self._on_status, queue_size=1)
        root.bind('<space>', lambda e: self.toggle())
        threading.Thread(target=self._poll_loop, daemon=True, name='poll').start()
        self._tick()

    # ── layout ───────────────────────────────────────────────────────────────
    def _build(self):
        r = self.root
        r.title("ORIO Dashboard")
        r.configure(bg=BASE)
        r.geometry("1280x900")
        r.minsize(1000, 760)

        # Header
        hdr = tk.Frame(r, bg=MANTLE, height=70)
        hdr.pack(fill='x')
        hdr.pack_propagate(False)
        left = tk.Frame(hdr, bg=MANTLE)
        left.pack(side='left', padx=20)
        tk.Label(left, text="ORIO", font=self.f_logo, fg=BLUE, bg=MANTLE).pack(side='left')
        tk.Label(left, text="  Dual-arm labelling cell · live", font=self.f_body,
                 fg=SUBTEXT, bg=MANTLE).pack(side='left', pady=(8, 0))
        right = tk.Frame(hdr, bg=MANTLE)
        right.pack(side='right', padx=20)
        self.conn = tk.Label(right, font=self.f_bodyb, bg=MANTLE)
        self.conn.pack(side='right')
        self.clock = tk.Label(right, font=self.f_body, fg=SUBTEXT, bg=MANTLE)
        self.clock.pack(side='right', padx=(0, 18))
        mid = tk.Frame(hdr, bg=MANTLE)
        mid.pack(side='right', padx=24)
        self.item_img = tk.Label(mid, bg=MANTLE)
        self.item_img.pack(side='left', padx=(0, 10))
        txt = tk.Frame(mid, bg=MANTLE)
        txt.pack(side='left')
        self.item_lbl = tk.Label(txt, font=self.f_bodyb, fg=TEXT, bg=MANTLE, anchor='w')
        self.item_lbl.pack(anchor='w')
        self.exp_lbl = tk.Label(txt, font=self.f_small, fg=SUBTEXT, bg=MANTLE, anchor='w')
        self.exp_lbl.pack(anchor='w')

        # Banner + buttons (packed right to left: E-STOP, SHUTDOWN, PAUSE)
        self.banner = tk.Frame(r, bg=SURF0, height=100)
        self.banner.pack(fill='x', padx=16, pady=(14, 8))
        self.banner.pack_propagate(False)
        self.estop_btn = tk.Button(self.banner, text="■  E-STOP", font=self.f_btn, relief='flat',
                                   bd=0, padx=26, pady=10, cursor='hand2', bg=ESTOP, fg='white',
                                   activebackground='#ff5c7a', activeforeground='white',
                                   command=self.estop)
        self.estop_btn.pack(side='right', padx=(8, 18), pady=14)
        self.shut_btn = tk.Button(self.banner, text="SHUTDOWN", font=self.f_h2, relief='flat',
                                  bd=0, padx=18, pady=14, cursor='hand2', bg=PEACH, fg=CRUST,
                                  activebackground='#fdd0b0', activeforeground=CRUST,
                                  command=self.shutdown)
        self.shut_btn.pack(side='right', padx=8, pady=14)
        self.btn = tk.Button(self.banner, font=self.f_btn, relief='flat', bd=0,
                             padx=30, pady=10, cursor='hand2', command=self.toggle)
        self.btn.pack(side='right', padx=8, pady=14)
        bt = tk.Frame(self.banner, bg=SURF0)
        bt.pack(side='left', fill='both', expand=True, padx=20)
        self.b_title = tk.Label(bt, font=self.f_h1, bg=SURF0, anchor='w')
        self.b_title.pack(fill='x', pady=(14, 0))
        self.b_sub = tk.Label(bt, font=self.f_body, bg=SURF0, anchor='w', justify='left')
        self.b_sub.pack(fill='x')

        # Cell map
        self.map = tk.Canvas(r, bg=BASE, highlightthickness=0, height=300)

        # Bottom: arm cards (with their brake buttons), then the newest debug images
        bot = tk.Frame(r, bg=BASE, height=330)
        bot.pack(side='bottom', fill='x', padx=16, pady=(8, 16))   # before the map, which then
        self.map.pack(fill='both', expand=True, padx=16)               # takes what is left
        bot.grid_propagate(False)   # keep its 330 px whatever its contents ask for
        bot.columnconfigure(0, weight=1, uniform='c')
        bot.columnconfigure(1, weight=1, uniform='c')
        bot.columnconfigure(2, weight=2, uniform='c')
        bot.rowconfigure(0, weight=1)
        self.cards, self.desk_btns, self.intervene_btns = {}, {}, {}
        for i, arm in enumerate(('arm1', 'arm2')):
            c = tk.Canvas(bot, bg=BASE, highlightthickness=0)
            c.grid(row=0, column=i, sticky='nsew', padx=(0, 10))
            self.cards[arm] = c
            row = tk.Frame(c, bg=MANTLE)
            row.place(x=20, rely=1.0, y=-14, anchor='sw')
            # One brake button: UNLOCK & PRIME while the arm is locked, LOCK while it is
            # not (set each tick from Desk's state, in _draw_brakes)
            brake = tk.Button(row, text="UNLOCK & PRIME", font=self.f_smallb, relief='flat', bd=0,
                              padx=12, pady=7, cursor='hand2', bg=SURF0, fg=CRUST,
                              disabledforeground=SURF2, state='disabled',
                              command=lambda a=arm: self._brake_pressed(a))
            brake.pack(side='left')
            help_btn = tk.Button(row, text="INTERVENE", font=self.f_smallb, relief='flat', bd=0,
                                 padx=12, pady=7, cursor='hand2', bg=PEACH, fg=CRUST,
                                 activebackground='#fdd0b0', activeforeground=CRUST,
                                 disabledforeground=SURF2,
                                 command=lambda a=arm: self.intervene(a))
            help_btn.pack(side='left', padx=(8, 0))
            self.desk_btns[arm] = brake
            self.intervene_btns[arm] = help_btn
        side = tk.Frame(bot, bg=BASE)
        side.grid(row=0, column=2, sticky='nsew')
        side.rowconfigure(0, weight=1)
        self.views = []
        for i, (key, feed, title, col, empty) in enumerate(self.feeds):
            side.columnconfigure(i, weight=1, uniform='img')
            c = tk.Canvas(side, bg=BASE, highlightthickness=0, cursor='hand2')
            c.grid(row=0, column=i, sticky='nsew', padx=(0, 8) if i == 0 else 0)
            c.bind('<Button-1>', lambda e, k=key: self._open_viewer(k))
            self.views.append(FeedView(self, c, feed, title, col, empty, "click to enlarge",
                                       verdict=self._grasp_verdict if key == 'grasp' else None))

    # ── data ─────────────────────────────────────────────────────────────────
    def _on_status(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.status, self.rx_time = s, time.time()

    def connected(self):
        return self.status is not None and time.time() - self.rx_time < STALE_S

    def toggle(self):
        if self.pending or not self.connected() or self.status.get('stopping'):
            return
        self._call('/orio/resume' if self.status.get('paused') else '/orio/pause')

    def estop(self):
        """Same as Ctrl-C in the terminal: both arms stop mid-motion. Once the run has
        been E-stopped for FORCE_QUIT_AFTER_S, a second press exits it at once."""
        if not self.connected():
            return
        if self.status.get('stopping') == 'estop':
            if self.estop_seen is None or time.time() - self.estop_seen < FORCE_QUIT_AFTER_S:
                return
            print("[Dashboard] Force quit requested", flush=True)
        self._call('/orio/estop', track=False)

    def shutdown(self):
        if not self.connected() or self.status.get('stopping'):
            return
        if messagebox.askyesno(
                "Shut down", "Finish the current moves, send both arms home and end the run?\n\n"
                "An item an arm is still holding is dropped at home.", parent=self.root):
            self._call('/orio/shutdown', track=False)

    def _call(self, service, track=True):
        """Call a Trigger service off the Tk thread. track: block the Pause button
        until it answers (E-STOP and SHUTDOWN never wait on anything)."""
        if track:
            self.pending = True
            self.btn.configure(state='disabled')

        def run():
            try:
                rospy.wait_for_service(service, timeout=2.0)
                resp = rospy.ServiceProxy(service, Trigger)()
                ok, note = resp.success, resp.message
            except Exception as exc:  # noqa: BLE001
                ok, note = False, str(exc).split('\n')[0]
            self.root.after(0, lambda: self._call_done(service, ok, note, track))
        threading.Thread(target=run, daemon=True).start()

    def _call_done(self, service, ok, note, track):
        if track:
            self.pending = False
        if not ok and service != '/orio/estop':   # a force quit never answers
            print("[Dashboard] %s failed: %s" % (service, note), flush=True)

    # ── brakes (lock_arms.py through desk_helper.py) ─────────────────────────
    def _brake_pressed(self, arm):
        """The brake button does whatever its label says (see _draw_brakes)."""
        self.desk(arm, 'lock' if self.desk_btns[arm].cget('text') == "LOCK" else 'unlock')

    def intervene(self, arm):
        """INTERVENE: both arms stop at their next pause point (as for Pause, never in the
        middle of touching an item) and this arm's recovery window opens, where the
        operator picks what it does next; resuming from it releases both."""
        if not self.connected() or self.status.get('stopping'):
            return
        if messagebox.askyesno(
                "Intervene on %s" % ARM_TITLE[arm],
                "Stop BOTH arms after their current moves and open %s's recovery "
                "window?\n\nThere you choose which step %s continues from. Both arms "
                "carry on when you resume from it." % (ARM_TITLE[arm], ARM_TITLE[arm]),
                parent=self.root):
            self._call('/orio/intervene/%s' % arm, track=False)

    def desk(self, arm, action):
        """LOCK or UNLOCK & PRIME button: confirm, then run lock_arms.py for this arm's
        robot off the Tk thread."""
        with self.desk_lock:
            if self.desk_state[arm]['busy']:
                return
        s = self.status if self.connected() else None
        if s is not None and s['arms'][arm].get('moving'):
            return
        robot = ARM_ROBOT[arm]
        if action == 'unlock':
            title = "Unlock & prime %s" % ARM_TITLE[arm]
            question = ("Unlock %s (robot %s)?\n\nIf Desk's end effector is not \"Other\" it is "
                        "set to Other first, which restarts the robot (up to ~3 min). Then the "
                        "brakes open: the arm moves slightly." % (ARM_TITLE[arm], robot))
        else:
            title = "Lock %s" % ARM_TITLE[arm]
            question = ("Lock %s (robot %s)?\n\nThe brakes close and the arm cannot move until "
                        "it is unlocked again. A running sequence fails its next move on this "
                        "arm." % (ARM_TITLE[arm], robot))
        if not messagebox.askyesno(title, question, parent=self.root):
            return
        with self.desk_lock:
            self.desk_state[arm] = {'busy': action, 'ok': None, 'msg': '', 't': time.time()}
        print("[Dashboard] %s: %s requested" % (ARM_TITLE[arm], action), flush=True)
        threading.Thread(target=self._desk_run, args=(arm, robot, action), daemon=True).start()

    def _desk_run(self, arm, robot, action):
        body = json.dumps({'robot': robot, 'action': action}).encode()
        req = urllib.request.Request(DESK_HELPER + '/lock_arms', data=body, method='POST',
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=DESK_TIMEOUT_S) as r:
                res = json.loads(r.read().decode())
        except urllib.error.HTTPError as exc:
            try:
                res = json.loads(exc.read().decode())
            except ValueError:
                res = {'ok': False, 'lines': ['FAILED: desk helper answered HTTP %d' % exc.code]}
        except (OSError, ValueError) as exc:
            reason = getattr(exc, 'reason', exc)
            res = {'ok': False, 'lines': ['FAILED: desk helper not reachable (%s); it runs on '
                                          'the host, started by run_full_demo.sh' % reason]}
        ok = bool(res.get('ok'))
        lines = [ROBOT_PREFIX.sub('', l).strip() for l in res.get('lines') or []]
        lines = [l for l in lines if l]
        failed = [l for l in lines if l.startswith('FAILED')]
        msg = (failed or lines or ['done' if ok else 'FAILED: no output'])[-1]
        print("[Dashboard] %s %s: %s" % (ARM_TITLE[arm], action, msg if not ok or msg.startswith('FAILED')
                                         else 'done (%s)' % msg), flush=True)
        with self.desk_lock:
            self.desk_state[arm] = {'busy': None, 'ok': ok, 'msg': msg, 't': time.time(),
                                    'action': action}
        self.helper_up = 'desk helper not reachable' not in msg

    def _fetch_desk_status(self):
        """Each robot's brakes, mode and active errors, from desk_helper.py (which reads
        Desk). False when the helper is not answering."""
        try:
            with urllib.request.urlopen(DESK_HELPER + '/status', timeout=2.0) as r:
                self.desk_robots = json.loads(r.read().decode()).get('robots') or {}
            return True
        except (OSError, ValueError):
            return False

    def _grasp_verdict(self):
        s = self.status if self.connected() else None
        return (s or {}).get('arms', {}).get('arm1', {}).get('grasp')

    # ── background polling: debug images, desk helper ───────────────────────
    def _poll_loop(self):
        last_desk = 0.0
        while True:
            for _key, feed, *_ in self.feeds:
                feed.poll()
            if time.time() - last_desk > DESK_STATUS_S:
                last_desk = time.time()
                self.helper_up = self._fetch_desk_status()
            time.sleep(POLL_S)

    def _open_viewer(self, key):
        """An enlarged, live copy of one debug image. Click or Esc closes it."""
        if key in self.viewers:
            self.viewers[key][0].lift()
            return
        _key, feed, title, col, empty = next(f for f in self.feeds if f[0] == key)
        top = tk.Toplevel(self.root, bg=BASE)
        top.title("ORIO · %s" % title.title())
        top.geometry("%dx%d" % (int(self.root.winfo_screenwidth() * 0.85),
                                int(self.root.winfo_screenheight() * 0.8)))
        c = tk.Canvas(top, bg=BASE, highlightthickness=0, cursor='hand2')
        c.pack(fill='both', expand=True, padx=12, pady=12)
        close = lambda e=None: self._close_viewer(key)
        c.bind('<Button-1>', close)
        top.bind('<Escape>', close)
        top.protocol('WM_DELETE_WINDOW', close)
        self.viewers[key] = (top, FeedView(self, c, feed, title, col, empty, "click or Esc to close",
                                           verdict=self._grasp_verdict if key == 'grasp' else None))

    def _close_viewer(self, key):
        top, _view = self.viewers.pop(key, (None, None))
        if top is not None:
            try:
                top.destroy()
            except tk.TclError:
                pass

    # ── main loop ────────────────────────────────────────────────────────────
    def _tick(self):
        t = time.time()
        if self.parent_pid and not self._parent_alive():
            self.root.destroy()
            return
        s = self.status if self.connected() else None
        if s is not None and s.get('started'):   # hand-opened mid-run: show this run's images
            for _key, feed, *_ in self.feeds:
                feed.since = min(self.t0, s['started'])
        try:
            self._draw_header(s, t)
            self._draw_banner(s)
            self._draw_map(s, t)
            for arm in ('arm1', 'arm2'):
                self._draw_card(arm, s, t)
            for view in self.views:
                view.draw(t)
        except tk.TclError:
            return  # window closed
        for key, (_top, view) in list(self.viewers.items()):
            try:
                view.draw(t)
            except tk.TclError:
                self._close_viewer(key)
        self.root.after(TICK_MS, self._tick)

    def _parent_alive(self):
        try:
            os.kill(self.parent_pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    # ── header / banner ──────────────────────────────────────────────────────
    def _draw_header(self, s, t):
        if s is None:
            self.conn.configure(text="●  NO SIGNAL", fg=RED)
            self.clock.configure(text="")
        else:
            self.conn.configure(text="●  LIVE", fg=GREEN if int(t * 2) % 2 else '#7fbf7f')
            if s.get('started'):
                el = int(s['t'] - s['started'])
                self.clock.configure(text="Run time  %d:%02d:%02d" % (el // 3600, el // 60 % 60, el % 60))
            else:
                self.clock.configure(text="Starting…")
            item = s.get('item') or {}
            if item.get('name') != self.item_name:
                self.item_name = item.get('name')
                self.item_lbl.configure(text=self.item_name or "")
                self.exp_lbl.configure(text=("Expires %s" % item['expiry']) if item.get('expiry') else "")
                self._load_item_image(self.item_name)

    def _load_item_image(self, name):
        self.item_photo = None
        try:
            from item_setup_gui import load_items
            from PIL import Image, ImageTk
            path = dict(load_items()).get(name)
            if path:
                im = Image.open(path).convert('RGBA')
                im.thumbnail((48, 48), Image.LANCZOS)
                self.item_photo = ImageTk.PhotoImage(im, master=self.root)
        except Exception:  # noqa: BLE001 — the picture is decoration
            self.item_photo = None
        self.item_img.configure(image=self.item_photo or '')

    def _stopped(self, s, arm):
        """True when `arm` is not moving and will not move until someone acts."""
        a = s['arms'][arm]
        return bool(a.get('held') or a.get('recovery') or a.get('state') is None
                    or (arm == 'arm1' and s.get('prompt')))

    def _draw_banner(self, s):
        if s is None:
            bg, fg = SURF0, SUBTEXT
            title = "Waiting for the sequence…"
            sub = "Start run_full_demo.sh. This window connects on its own."
        elif s.get('stopping') == 'estop':
            bg, fg = '#3a2229', RED
            title = "E-STOP: both arms stopped"
            sub = ("The run is ending. If this window stays open, press FORCE QUIT."
                   if self._force_quit_ready() else "The run is ending.")
        elif s.get('stopping') == 'home':
            bg, fg = '#3a2e24', PEACH
            title = "SHUTTING DOWN"
            sub = ("The arms finish their move, then go home. Suction turns off at home, "
                   "so a held item drops there.")
        elif s.get('shutdown'):
            bg, fg, title, sub = SURF0, PEACH, "STOPPING", "The run is ending."
        elif s['paused']:
            moving = [a for a in ('arm1', 'arm2') if not self._stopped(s, a)]
            if moving:
                bg, fg = '#3a3426', YELLOW
                title = "PAUSING…"
                sub = "Waiting for %s to finish %s." % (
                    " and ".join(ARM_TITLE[a] for a in moving),
                    "its move" if len(moving) == 1 else "their moves")
            else:
                bg, fg = '#3a3426', YELLOW
                title = "PAUSED: safe to swap the input items"
                sub = "Both arms are stopped. Anything an arm holds stays held. Press Resume when done."
        else:
            bg, fg = '#203026', GREEN
            title = "RUNNING"
            sub = "Pause stops both arms after their current move (Space bar works too)."
        if s is not None:
            for arm in ('arm1', 'arm2'):
                if s['arms'][arm].get('recovery'):
                    bg, fg = '#3a2229', RED
                    sub = "%s needs help: see its recovery window. " % ARM_TITLE[arm] + sub
            if s.get('prompt'):
                sub = "The terminal is waiting for you: %s" % s['prompt'].split('\n')[0]
        self.banner.configure(bg=bg)
        for w in (self.b_title, self.b_sub, self.b_title.master):
            w.configure(bg=bg)
        self.b_title.configure(text=title, fg=fg)
        self.b_sub.configure(text=sub, fg=TEXT if s else SUBTEXT)

        if s is not None and s['paused']:
            self.btn.configure(text="▶  RESUME", bg=GREEN, fg=CRUST, activebackground='#c3f0bd',
                               activeforeground=CRUST)
        else:
            self.btn.configure(text="❚❚  PAUSE", bg=YELLOW, fg=CRUST, activebackground='#fcefcf',
                               activeforeground=CRUST)
        usable = s is not None and not s.get('shutdown') and not self.pending
        self.btn.configure(state='normal' if usable else 'disabled',
                           disabledforeground=SURF2 if s is None else CRUST)
        stopping = s.get('stopping') if s else None
        if s is None or stopping or s.get('shutdown'):
            self.btn.configure(bg=SURF1, disabledforeground=SURF2)

        self.shut_btn.configure(state='normal' if s is not None and not stopping else 'disabled',
                                bg=PEACH if s is not None and not stopping else SURF1,
                                disabledforeground=SURF2)
        if stopping == 'estop' and self.estop_seen is None:
            self.estop_seen = time.time()
        if s is None:
            self.estop_btn.configure(text="■  E-STOP", state='disabled', bg=SURF1,
                                     disabledforeground=SURF2)
        elif stopping == 'estop':
            ready = self._force_quit_ready()
            self.estop_btn.configure(text="FORCE QUIT" if ready else "STOPPING…",
                                     state='normal' if ready else 'disabled', bg=ESTOP,
                                     disabledforeground='#f0c0c8')
        else:
            self.estop_btn.configure(text="■  E-STOP", state='normal', bg=ESTOP)
        # keep the sub-line clear of the buttons
        room = self.banner.winfo_width() - sum(
            b.winfo_width() + 20 for b in (self.btn, self.shut_btn, self.estop_btn)) - 50
        self.b_sub.configure(wraplength=max(300, room))

    def _force_quit_ready(self):
        return self.estop_seen is not None and time.time() - self.estop_seen >= FORCE_QUIT_AFTER_S

    # ── cell map ─────────────────────────────────────────────────────────────
    def _draw_map(self, s, t):
        c = self.map
        c.delete('all')
        W, H = max(c.winfo_width(), 600), max(c.winfo_height(), 300)
        tw, th = W * 0.17, H * 0.36           # station tile size
        top, bottom = H * 0.27, H * 0.75
        P = {   # centres: input, zone A and the dispenser on the bottom row
            'output': (W * 0.11, top), 'B': (W * 0.50, top), 'labels': (W * 0.89, top),
            'bin': (W * 0.11, bottom), 'A': (W * 0.50, bottom), 'dispenser': (W * 0.89, bottom),
            'arm1': (W * 0.30, H * 0.51), 'arm2': (W * 0.70, H * 0.51),
        }
        reach = {'arm1': ('bin', 'A', 'B', 'output'), 'arm2': ('dispenser', 'A', 'B')}
        arms = s['arms'] if s else {}

        # links (under everything)
        for arm, spots in reach.items():
            target = arms.get(arm, {}).get('target')
            for spot in spots:
                (x0, y0), (x1, y1) = P[arm], P[spot]
                if spot == target:
                    a = arms[arm]
                    off = -int(t * 40) % 24 if a.get('moving') else 0
                    c.create_line(x0, y0, x1, y1, fill=ARM_COLOR[arm], width=6,
                                  dash=(14, 10), dashoffset=off, capstyle='round')
                else:
                    c.create_line(x0, y0, x1, y1, fill=SURF0, width=3, dash=(3, 6))

        fully_paused = s is not None and s['paused'] and all(self._stopped(s, a) for a in ('arm1', 'arm2'))
        self._station_bin(c, P['bin'], tw, th, s, t, fully_paused)
        self._station_output(c, P['output'], tw, th, s)
        self._station_dispenser(c, P['dispenser'], tw, th, s)
        self._station_labels(c, P['labels'], tw, th, s)
        for z in ('A', 'B'):
            self._zone(c, z, P[z], tw * 1.15, th, s, t)
        for arm in ('arm1', 'arm2'):
            self._hub(c, arm, P[arm], min(W * 0.055, H * 0.2), s, t)

    def _tile(self, c, centre, w, h, title, outline=SURF1, fill=MANTLE, width=2):
        x, y = centre
        rrect(c, x - w / 2, y - h / 2, x + w / 2, y + h / 2, 16, fill=fill, outline=outline, width=width)
        c.create_text(x - w / 2 + 14, y - h / 2 + 16, text=title, anchor='w',
                      fill=SUBTEXT, font=self.f_smallb)
        return x - w / 2, y - h / 2, x + w / 2, y + h / 2

    def _station_bin(self, c, centre, w, h, s, t, fully_paused):
        active = s is not None and s['arms']['arm1'].get('target') == 'bin'
        outline = YELLOW if fully_paused else (BLUE if active else SURF1)
        x0, y0, x1, y1 = self._tile(c, centre, w, h, "INPUT BIN", outline=outline,
                                    width=3 if (active or fully_paused) else 2)
        cx, cy = centre
        bw, bh = w * 0.62, h * 0.36
        top, bot = cy - bh * 0.25, cy + bh * 0.75
        c.create_polygon(cx - bw / 2, top, cx + bw / 2, top, cx + bw * 0.4, bot, cx - bw * 0.4, bot,
                         fill=SURF0, outline=SURF2, width=2)
        for i, (dx, col) in enumerate(((-0.25, CARDBOARD), (0.0, TEAL), (0.24, PEACH))):
            c.create_rectangle(cx + bw * dx - 10, top - 12 + (i % 2) * 6, cx + bw * dx + 10,
                               top + 8 + (i % 2) * 6, fill=col, outline='')
        if fully_paused:
            note, col = "Swap items now", YELLOW
        elif active and s['arms']['arm1'].get('activity', '').startswith(('Planning', 'No grasp')):
            note, col = "Scanning" + "." * (int(t * 3) % 4), BLUE
        elif s is not None and 'bin empty' in (s['arms']['arm1'].get('activity') or '').lower():
            note, col = "Empty: add items", PEACH
        else:
            note, col = "", SUBTEXT
        if note:
            c.create_text(cx, y1 - 16, text=note, fill=col, font=self.f_bodyb)

    def _station_output(self, c, centre, w, h, s):
        active = s is not None and s['arms']['arm1'].get('target') == 'output'
        x0, y0, x1, y1 = self._tile(c, centre, w, h, "OUTPUT", outline=BLUE if active else SURF1,
                                    width=3 if active else 2)
        cx, cy = centre
        n = s['counts']['placed'] if s else 0
        for i in range(min(n, 6)):   # a little stack of labelled items
            box(c, cx - 26 + (i % 3) * 26, cy + 12 - (i // 3) * 22, 11, label=True)
        c.create_text(cx, y1 - 16, text="%d delivered" % n, fill=TEXT, font=self.f_bodyb)

    def _station_dispenser(self, c, centre, w, h, s):
        active = s is not None and s['arms']['arm2'].get('target') == 'dispenser'
        x0, y0, x1, y1 = self._tile(c, centre, w, h, "LABEL DISPENSER",
                                    outline=MAUVE if active else SURF1, width=3 if active else 2)
        cx, cy = centre
        c.create_oval(cx - 30, cy - 26, cx + 30, cy + 34, fill='#e8e8e8', outline=SURF2, width=2)
        c.create_oval(cx - 11, cy - 7, cx + 11, cy + 15, fill=MANTLE, outline=SURF2)
        c.create_line(cx + 30, cy + 4, cx + 52, cy + 30, fill='#e8e8e8', width=6)

    def _station_labels(self, c, centre, w, h, s):
        x0, y0, x1, y1 = self._tile(c, centre, w, h, "LABELS APPLIED")
        cx, cy = centre
        n = s['counts']['labels'] if s else 0
        c.create_text(cx, cy - 2, text=str(n), fill=MAUVE, font=self.f_num)
        item = (s or {}).get('item') or {}
        if item.get('name'):
            c.create_text(cx, y1 - 26, text=shorten(item['name'], self.f_small, w - 20),
                          fill=SUBTEXT, font=self.f_small)
            c.create_text(cx, y1 - 12, text="exp %s" % item.get('expiry', ''), fill=OVERLAY,
                          font=self.f_small)

    def _zone(self, c, z, centre, w, h, s, t):
        zs = s['zones'][z] if s else {'state': 'EMPTY', 'busy': None}
        text, fill, outline = ZONE_LOOK.get(zs['state'], (zs['state'], SURF0, SURF1))
        owner = zs.get('busy')
        width = 3
        if owner in ARM_COLOR:
            outline = ARM_COLOR[owner]
            width = 3 + (1 if int(t * 3) % 2 else 0)
        x0, y0, x1, y1 = self._tile(c, centre, w, h, "", outline=outline, fill=fill, width=width)
        cx, cy = centre
        c.create_text(x0 + 16, y0 + 20, text="ZONE %s" % z, anchor='w', fill=TEXT, font=self.f_h2)
        if owner in ARM_COLOR:
            pill(c, x1 - 12, y0 + 20, "%s working" % ARM_TITLE[owner], CRUST, ARM_COLOR[owner],
                 self.f_smallb)
        if zs['state'] != 'EMPTY':
            box(c, cx, cy + 4, 26, label=zs['state'] == 'READY_FOR_PICKUP')
        else:
            rrect(c, cx - 28, cy - 16, cx + 28, cy + 22, 8, fill='', outline=SURF2, dash=(4, 4))
        c.create_text(cx, y1 - 18, text=text, fill=outline if zs['state'] != 'EMPTY' else SUBTEXT,
                      font=self.f_bodyb)

    def _arm_look(self, s, arm):
        """(ring colour, short status) for an arm."""
        if s is None:
            return SURF1, "offline"
        a = s['arms'][arm]
        if s.get('stopping') == 'estop':
            return RED, "E-STOPPED"
        if s.get('stopping') == 'home':
            if a.get('moving'):
                return PEACH, ("GOING HOME" if 'shutdown' in (a.get('activity') or '')
                               else "FINISHING MOVE")
            return SURF2, "SHUTTING DOWN"
        if a.get('recovery'):
            return RED, "NEEDS HELP"
        if a.get('held'):
            return YELLOW, "PAUSED"
        if arm == 'arm1' and s.get('prompt'):
            return PEACH, "WAITING FOR YOU"
        if (arm == 'arm1' and not a.get('moving')
                and 'bin empty' in (a.get('activity') or '').lower()):
            return PEACH, "BIN EMPTY"
        if a.get('state') is None:
            return SURF1, "stopped" if s.get('started') else "starting"
        if a.get('moving'):
            return ARM_COLOR[arm], "PAUSING" if s['paused'] else "MOVING"
        return SURF2, "PAUSING" if s['paused'] else "IDLE"

    def _hub(self, c, arm, centre, r, s, t):
        cx, cy = centre
        ring, short = self._arm_look(s, arm)
        a = s['arms'][arm] if s else {}
        c.create_oval(cx - r - 10, cy - r - 10, cx + r + 10, cy + r + 10, fill=BASE, outline='')
        c.create_oval(cx - r, cy - r, cx + r, cy + r, fill=MANTLE, outline=ring, width=5)
        if a.get('moving'):
            c.create_arc(cx - r - 7, cy - r - 7, cx + r + 7, cy + r + 7, start=-(t * 300) % 360,
                         extent=70, style='arc', outline=ARM_COLOR[arm], width=4)
        c.create_text(cx, cy - 12, text=ARM_TITLE[arm], fill=ARM_COLOR[arm], font=self.f_h2)
        c.create_text(cx, cy + 12, text=short, fill=ring if ring not in (SURF1, SURF2) else SUBTEXT,
                      font=self.f_smallb)
        if a.get('held'):   # pause glyph
            for dx in (-9, 3):
                c.create_rectangle(cx + dx, cy - r - 26, cx + dx + 6, cy - r - 10, fill=YELLOW, outline='')
        if a.get('cup'):    # what it is holding
            if arm == 'arm1':
                box(c, cx + r * 0.75, cy + r * 0.75, 13)
            else:
                sticker(c, cx + r * 0.75, cy + r * 0.75, 14)

    # ── arm cards ────────────────────────────────────────────────────────────
    def _draw_card(self, arm, s, t):
        c = self.cards[arm]
        c.delete('all')
        W, H = max(c.winfo_width(), 200), max(c.winfo_height(), 150)
        col = ARM_COLOR[arm]
        rrect(c, 0, 0, W, H, 16, fill=MANTLE, outline='')
        c.create_rectangle(0, 16, 5, H - 16, fill=col, outline='')
        c.create_text(20, 24, text=ARM_TITLE[arm], anchor='w', fill=col, font=self.f_h2)
        c.create_text(20 + self.f_h2.measure(ARM_TITLE[arm]) + 10, 25, text=ARM_ROLE[arm],
                      anchor='w', fill=SUBTEXT, font=self.f_body)
        ring, short = self._arm_look(s, arm)
        pill(c, W - 14, 24, short, CRUST if ring not in (SURF1, SURF2) else TEXT, ring, self.f_smallb)

        a = s['arms'][arm] if s else {}
        state = a.get('state')
        # pipeline chips
        steps = ARM_STEPS[arm]
        gap, x, y0, y1 = 6, 20, 48, 78
        cw = (W - 34 - gap * (len(steps) - 1)) / len(steps)
        for key, name in steps:
            on = state == key
            rrect(c, x, y0, x + cw, y1, 10, fill=col if on else SURF0, outline='')
            c.create_text(x + cw / 2, (y0 + y1) / 2, text=shorten(name, self.f_smallb, cw - 8),
                          fill=CRUST if on else SUBTEXT, font=self.f_smallb)
            x += cw + gap

        # what it is doing now
        if s is None:
            now = "No signal"
        elif a.get('recovery'):
            now = "In the recovery window"
        else:
            now = a.get('activity') or STATE_NAME.get(state, '')
            if state == 'DECIDE_HUB' and not a.get('moving'):
                now = "Choosing the next task"
        c.create_text(20, 100, text="NOW", anchor='w', fill=OVERLAY, font=self.f_smallb)
        c.create_text(20, 120, text=shorten(now[:1].upper() + now[1:] if now else "", self.f_bodyb, W - 40),
                      anchor='w', fill=TEXT, font=self.f_bodyb)
        c.create_text(20, 140, text=STATE_NAME.get(state, '') if state else '', anchor='w',
                      fill=SUBTEXT, font=self.f_small)

        # suction
        what = 'item' if arm == 'arm1' else 'label'
        cup = a.get('cup')
        c.create_text(20, 170, text="SUCTION", anchor='w', fill=OVERLAY, font=self.f_smallb)
        c.create_oval(20, 184, 34, 198, fill=GREEN if cup else SURF1, outline='')
        c.create_text(42, 191, anchor='w', fill=TEXT, font=self.f_body,
                      text="On, holding the %s" % what if cup else "Off")
        if s is not None and not s.get('vacuum', True):
            sensor = "pneumatics off (dry run)"
        elif a.get('raw') is not None:
            sensor = "sensor: %s (raw %s)" % ("grip" if a.get('has_item') else "no grip", a['raw'])
        elif 'has_item' in a:
            sensor = "sensor: %s" % ("grip" if a.get('has_item') else "no grip")
        else:
            sensor = ""
        c.create_text(20, 208, anchor='w', fill=SUBTEXT, font=self.f_small, text=sensor)

        # brakes: what the last LOCK / UNLOCK & PRIME press did; the buttons sit below
        self._draw_brakes(c, arm, a, W, t)

    def _draw_brakes(self, c, arm, a, W, t):
        """The arm's live brake state and errors (Desk, via desk_helper.py), and which
        buttons make sense: UNLOCK & PRIME only while its brakes are locked, LOCK only
        while they are open and the arm is not moving."""
        with self.desk_lock:
            st = dict(self.desk_state[arm])
        rs = self.desk_robots.get(ARM_ROBOT[arm]) or {}
        known = 'brakes_open' in rs and t - rs.get('t', 0) < DESK_STALE_S
        locked = known and not rs['brakes_open']
        errors = rs.get('errors') or [] if known else []
        c.create_text(20, 236, text="BRAKES", anchor='w', fill=OVERLAY, font=self.f_smallb)
        if st['busy']:
            dot = YELLOW if int(t * 2) % 2 else SURF2
            fg = YELLOW
            text = ("Unlocking… up to ~3 min if the robot restarts" if st['busy'] == 'unlock'
                    else "Locking…")
        elif st['ok'] is False and t - st['t'] < 30:
            dot, fg = RED, RED
            text = "%s  %s" % (time.strftime('%H:%M', time.localtime(st['t'])), st['msg'])
        elif self.helper_up is False:
            dot, fg = SURF1, OVERLAY
            text = "Unavailable: desk_helper.py is not running on the host"
        elif errors:
            dot, fg = RED, RED
            text = "Error %s: clear it in Desk" % ", ".join(errors)
        elif locked:
            dot, fg = YELLOW, YELLOW
            text = "Locked: press UNLOCK & PRIME"
        elif known:
            dot, fg = GREEN, TEXT
            text = "Unlocked"
        else:
            dot, fg = SURF1, OVERLAY
            text = "Checking Desk…" if not rs.get('error') else "Unknown: %s" % rs['error']
        c.create_oval(20, 250, 34, 264, fill=dot, outline='')
        c.create_text(42, 257, anchor='w', fill=fg, font=self.f_small,
                      text=shorten(text, self.f_small, W - 56))

        free = not st['busy'] and self.helper_up is not False
        can_unlock = free and locked
        can_lock = free and known and rs['brakes_open'] and not a.get('moving')
        brake = self.desk_btns[arm]
        if known and rs['brakes_open']:
            brake.configure(text="LOCK", fg=TEXT, activebackground=SURF2, activeforeground=TEXT,
                            state='normal' if can_lock else 'disabled',
                            bg=SURF1 if can_lock else SURF0)
        else:
            brake.configure(text="UNLOCK & PRIME", fg=CRUST, activebackground='#c3f0bd',
                            activeforeground=CRUST, state='normal' if can_unlock else 'disabled',
                            bg=GREEN if can_unlock else SURF0)
        self._draw_intervene(arm, a)

    def _draw_intervene(self, arm, a):
        """INTERVENE: usable while the run is live and this arm is not already in (or on
        its way to) its recovery window."""
        s = self.status if self.connected() else None
        btn = self.intervene_btns[arm]
        if a.get('intervene'):
            btn.configure(text="STOPPING…", state='disabled', bg=SURF0)
            return
        usable = (s is not None and not s.get('stopping') and not s.get('shutdown')
                  and not a.get('recovery') and a.get('state') is not None)
        btn.configure(text="INTERVENE", state='normal' if usable else 'disabled',
                      bg=PEACH if usable else SURF0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--parent-pid', type=int, default=None,
                    help="Close when this process ends (full_sequence.py passes its pid).")
    ap.add_argument('--grasp-dir', default=GRASP_DIR,
                    help="Where the grasp planner writes its panels (default: %(default)s).")
    ap.add_argument('--label-dir', default=LABEL_DIR,
                    help="Where perception writes label_*.png (default: %(default)s).")
    args, _ = ap.parse_known_args(rospy.myargv()[1:])

    rospy.init_node('orio_dashboard', anonymous=True, disable_signals=True)
    root = tk.Tk()
    Dashboard(root, parent_pid=args.parent_pid, grasp_dir=args.grasp_dir, label_dir=args.label_dir)
    signal.signal(signal.SIGINT, lambda *_: root.destroy())
    signal.signal(signal.SIGTERM, lambda *_: root.destroy())
    root.mainloop()
    rospy.signal_shutdown('dashboard closed')


if __name__ == '__main__':
    main()
