#!/usr/bin/env python3
"""Inventory window: abuela's master inventory and the labels applied on this PC.

run_full_demo.sh opens it on the host when it starts the RFID node (--no-inventory to
skip). By hand, from any folder:
    python3 src/devel_packages/orio_rfid/inventory_dashboard.py

Every PULL_S seconds it refreshes this PC's copy of abuela's inventory (master_sync.py
pull, over ssh), and once a second it shows:
  - the newest label large, and every label applied here since the window opened,
    each with its item number and whether it has been pushed to abuela;
  - totals: labelled this run, tagged items, on the shelf, expired on the shelf;
  - stock per item in abuela's inventory, on the shelf or everything tagged.
PUSH TO MASTER opens the list of labels not pushed yet: tick some and push them to
abuela (master.db, item_names.csv, tag_registry.csv), or discard wrong ones; the
labels already pushed are listed below. Click the expired tile to list the expired
items. Needs Python 3 with tkinter; Pillow adds abuela's item photos.

  --since TIME   count labels from TIME ("YYYY-MM-DD HH:MM:SS") instead of from now
  --hide-new     stock only, no label panels
  --no-pull      do not refresh the copy from abuela (show it as it is)
"""

import argparse
import os
import sqlite3
import threading
import time
import tkinter as tk
from datetime import date, datetime
from tkinter import font as tkfont, messagebox

import master_sync

try:
    from PIL import Image, ImageTk
except ImportError:  # no pictures then, a drawn box instead
    Image = ImageTk = None

POLL_MS = 1000
PULL_S = 15           # refresh the copy of abuela's inventory this often
STALE_S = 60          # a copy older than this is shown as out of date
NO_NAME = 'No name yet'
SHELF_LIVE_S = 10     # a shelf scan this recent = shelf reader live
NEW_FLASH_S = 4.0     # how long a new label stays highlighted
SOON_DAYS = 30        # expiring within this many days = yellow
TS = '%Y-%m-%d %H:%M:%S'

# Catppuccin Mocha, as in the demo dashboard and the item setup window
CRUST, MANTLE, BASE = '#11111b', '#181825', '#1e1e2e'
SURF0, SURF1, SURF2 = '#313244', '#45475a', '#585b70'
OVERLAY, SUBTEXT, TEXT = '#6c7086', '#a6adc8', '#cdd6f4'
BLUE, MAUVE, GREEN, YELLOW = '#89b4fa', '#cba6f7', '#a6e3a1', '#f9e2af'
PEACH, RED, TEAL = '#fab387', '#f38ba8', '#94e2d5'
CARDBOARD, CARDBOARD_DARK = '#c8a27a', '#9c7a56'


# ── Helpers ──────────────────────────────────────────────────────────────────

def rrect(c, x0, y0, x1, y1, r, **kw):
    """Rounded rectangle on canvas c."""
    r = max(0, min(r, (x1 - x0) / 2, (y1 - y0) / 2))
    pts = [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1,
           x1 - r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0]
    return c.create_polygon(pts, smooth=True, **kw)


def pill(c, x, y, text, fg, bg, font, anchor='w', padx=10, h=24, tags=()):
    """A rounded label; (x, y) is its left (anchor 'w') or right ('e') middle. Returns its width."""
    w = font.measure(text) + 2 * padx
    x0 = x - w if anchor == 'e' else x
    rrect(c, x0, y - h / 2, x0 + w, y + h / 2, h / 2, fill=bg, outline='', tags=tags)
    c.create_text(x0 + w / 2, y, text=text, fill=fg, font=font, tags=tags)
    return w


def box_icon(c, cx, cy, s):
    """A small cardboard box with a label on it (for items without a picture)."""
    rrect(c, cx - s, cy - s * 0.7, cx + s, cy + s * 0.7, s * 0.2,
          fill=CARDBOARD, outline=CARDBOARD_DARK, width=2)
    c.create_line(cx - s, cy - s * 0.25, cx + s, cy - s * 0.25, fill=CARDBOARD_DARK, width=2)
    rrect(c, cx - s * 0.55, cy - s * 0.05, cx + s * 0.55, cy + s * 0.5, 4, fill='#f5f5f5', outline='')


def shorten(text, font, width):
    if font.measure(text) <= width:
        return text
    while text and font.measure(text + '…') > width:
        text = text[:-1]
    return text + '…'


def ago(seconds):
    seconds = max(0, int(seconds))
    if seconds < 60:
        return 'just now' if seconds < 5 else '%d s ago' % seconds
    if seconds < 3600:
        return '%d min ago' % (seconds // 60)
    if seconds < 86400:
        return '%d h ago' % (seconds // 3600)
    return '%d days ago' % (seconds // 86400)


def parse_ts(text):
    try:
        return datetime.strptime(text, TS).timestamp()
    except (TypeError, ValueError):
        return None


def tag_groups(uid):
    return ' '.join(uid[i:i + 4] for i in range(0, len(uid), 4))


def expiry_look(text):
    """(words, colour) for an expiration date."""
    try:
        d = datetime.strptime(text, '%Y-%m-%d').date()
    except (TypeError, ValueError):
        return ('Expires %s' % text if text else 'No expiry', SUBTEXT)
    days = (d - date.today()).days
    nice = '%d %s %d' % (d.day, d.strftime('%b'), d.year)
    if days < 0:
        return ('Expired %s' % nice, RED)
    if days == 0:
        return ('Expires today', RED)
    if days <= SOON_DAYS:
        return ('Expires in %d day%s' % (days, '' if days == 1 else 's'), YELLOW)
    return ('Expires %s' % nice, SUBTEXT)


def read_data(since):
    """A snapshot of the copy of abuela's inventory and of the labels applied here."""
    have_copy = os.path.exists(master_sync.COPY_DB)
    items = master_sync.copy_items() if have_copy else []
    today = date.today().isoformat()
    stock = {}
    for i in items:
        total, on = stock.get(i['name'] or NO_NAME, (0, 0))
        stock[i['name'] or NO_NAME] = (total + 1, on + i['on_shelf'])
    session = [(r['rfid_uid'], r['item_name'], r['expiration_date'], r['date_labeled'], r['item_no'], r['status'])
               for r in master_sync.labelled_since(since)]
    return {'have_copy': have_copy, 'session': session,
            'stock': sorted(((n, t, o) for n, (t, o) in stock.items()), key=lambda r: (-r[1], r[0])),
            'expired': sorted(((i['rfid_uid'], i['name'] or 'Item %d' % i['no'], i['expiry']) for i in items
                               if i['on_shelf'] and i['expiry'] and i['expiry'] < today), key=lambda r: r[2]),
            'last_scan': max((i['last_seen'] for i in items if i['last_seen']), default=None),
            'pending': len(master_sync.pending())}


# ── Window ───────────────────────────────────────────────────────────────────

class InventoryWindow:
    def __init__(self, root, since, show_new=True, pull=True):
        self.root, self.since, self.show_new = root, since, show_new
        root.title('ORIO Inventory')
        root.configure(bg=CRUST)
        root.geometry('1180x720' if show_new else '640x720')
        root.minsize(900 if show_new else 520, 600)

        family = next((f for f in ('Ubuntu', 'Cantarell', 'DejaVu Sans', 'Helvetica')
                       if f in tkfont.families(root)), 'Helvetica')
        mono = next((f for f in ('Ubuntu Mono', 'DejaVu Sans Mono', 'Courier')
                     if f in tkfont.families(root)), 'Courier')
        F = lambda size, weight='normal', fam=family: tkfont.Font(root=root, family=fam, size=size, weight=weight)
        self.f_title, self.f_h2, self.f_big = F(20, 'bold'), F(13, 'bold'), F(28, 'bold')
        self.f_name, self.f_body, self.f_small = F(17, 'bold'), F(11), F(9, 'bold')
        self.f_row, self.f_mono, self.f_mono_big = F(11, 'bold'), F(10, fam=mono), F(13, fam=mono)

        self.canvas = tk.Canvas(root, bg=CRUST, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)

        self.thumbs = {}
        self.data, self.error, self.updated = None, None, None
        self.known = None           # rfid_uids seen in an earlier poll
        self.flash_uid, self.flash_until = None, 0.0
        self.stock_tab = None       # 'shelf' or 'all'; picked on the first data
        self.scroll = {'list': 0, 'stock': 0}
        self.regions = {}           # scroll region name -> (x0, y0, x1, y1)
        self.popup = None
        self.push_win = None
        self.sync_lock = threading.Lock()   # one ssh exchange with abuela at a time
        self.pull_ok, self.pull_msg = None, ''  # the last pull: True / False / None (none yet)

        c = self.canvas
        c.bind('<Configure>', lambda e: self.draw())
        for seq, step in (('<MouseWheel>', None), ('<Button-4>', -1), ('<Button-5>', 1)):
            c.bind(seq, lambda e, step=step: self.on_wheel(e, step))
        self.clickable('expired', self.show_expired)
        self.clickable('tab_shelf', lambda: self.set_tab('shelf'))
        self.clickable('tab_all', lambda: self.set_tab('all'))
        self.clickable('push', self.show_push)
        if pull:
            threading.Thread(target=self.pull_loop, daemon=True).start()
        else:
            self.pull_msg = 'not refreshed (--no-pull)'
        self.poll()

    def pull_loop(self):
        while True:
            self.pull_now()
            time.sleep(PULL_S)

    def pull_now(self):
        with self.sync_lock:
            self.pull_ok, self.pull_msg = master_sync.pull()

    def clickable(self, tag, action):
        c = self.canvas
        c.tag_bind(tag, '<Button-1>', lambda e: action())
        c.tag_bind(tag, '<Enter>', lambda e: c.configure(cursor='hand2'))
        c.tag_bind(tag, '<Leave>', lambda e: c.configure(cursor=''))

    def set_tab(self, tab):
        self.stock_tab, self.scroll['stock'] = tab, 0
        self.draw()

    def on_wheel(self, event, step):
        if step is None:
            step = -1 if event.delta > 0 else 1
        for name, (x0, y0, x1, y1) in self.regions.items():
            if x0 <= event.x <= x1 and y0 <= event.y <= y1:
                self.scroll[name] = max(0, self.scroll[name] + step)
                self.draw()
                return

    # ── Data ──

    def poll(self):
        try:
            self.data = read_data(self.since)
            self.error = None
            if self.data is not None:
                self.updated = time.time()
                uids = [row[0] for row in self.data['session']]
                if self.known is not None:
                    fresh = [u for u in uids if u not in self.known]
                    if fresh:
                        self.flash_uid, self.flash_until = fresh[0], time.time() + NEW_FLASH_S
                        self.scroll['list'] = 0
                self.known = set(uids)
                if self.stock_tab is None:
                    self.stock_tab = 'shelf' if self.shelf_live() else 'all'
        except (sqlite3.Error, OSError) as e:
            self.error = str(e)
        self.draw()
        self.root.after(POLL_MS, self.poll)

    def shelf_live(self):
        t = parse_ts(self.data and self.data['last_scan'])
        return t is not None and time.time() - t <= SHELF_LIVE_S

    def thumb(self, name, size):
        """A PhotoImage of the item's picture fitted in size x size, or None."""
        path = master_sync.photo_for(name)
        if Image is None or not path:
            return None
        key = (path, size)
        if key not in self.thumbs:
            try:
                im = Image.open(path).convert('RGB')
                im.thumbnail((size, size), Image.LANCZOS)
                self.thumbs[key] = ImageTk.PhotoImage(im, master=self.root)
            except OSError:
                self.thumbs[key] = None
        return self.thumbs[key]

    def picture(self, name, cx, cy, size):
        """The item's picture on a rounded tile, or a box icon."""
        c = self.canvas
        rrect(c, cx - size / 2, cy - size / 2, cx + size / 2, cy + size / 2, 10, fill=SURF0, outline='')
        img = self.thumb(name, int(size - 8))
        if img is not None:
            c.create_image(cx, cy, image=img)
        else:
            box_icon(c, cx, cy, size * 0.28)

    # ── Drawing ──

    def card(self, x0, y0, x1, y1, outline=SURF0, width=1, tags=()):
        rrect(self.canvas, x0, y0, x1, y1, 16, fill=BASE, outline=outline, width=width, tags=tags)

    def draw(self):
        c = self.canvas
        c.delete('all')
        self.regions = {}
        W, H = c.winfo_width(), c.winfo_height()
        if W < 50 or H < 50:
            return
        pad = 20
        self.draw_header(pad, W)
        tiles_y = 84
        self.draw_tiles(pad, tiles_y, W - pad, tiles_y + 96)
        top = tiles_y + 96 + 16
        if self.data is None:
            self.draw_waiting(pad, top, W - pad, H - pad)
            return
        if self.show_new:
            split = pad + int((W - 2 * pad) * 0.58)
            hero_h = 158
            self.draw_hero(pad, top, split - 8, top + hero_h)
            self.draw_list(pad, top + hero_h + 16, split - 8, H - pad)
            self.draw_stock(split + 8, top, W - pad, H - pad)
        else:
            self.draw_stock(pad, top, W - pad, H - pad)

    def draw_header(self, pad, W):
        c = self.canvas
        c.create_text(pad, 32, text='Inventory', anchor='w', fill=TEXT, font=self.f_title)
        c.create_text(pad + self.f_title.measure('Inventory') + 14, 35, anchor='w',
                      text="abuela's master inventory and the labels applied here", fill=OVERLAY, font=self.f_body)
        pulled = master_sync.pulled_at()
        fresh = pulled is not None and time.time() - pulled <= STALE_S
        if self.error:
            look = ('●  Error', RED, '#3a2530')
        elif self.pull_ok is False:
            look = ('●  abuela unreachable', YELLOW, '#3a3426')
        elif pulled is None:
            look = ('●  Waiting for abuela', YELLOW, '#3a3426')
        else:
            look = ('●  Live' if fresh else '●  Copy out of date', GREEN if fresh else YELLOW,
                    '#253a2c' if fresh else '#3a3426')
        w = pill(c, W - pad, 30, look[0], look[1], look[2], self.f_row, anchor='e', h=28, padx=14)
        n = self.data['pending'] if self.data else 0
        pill(c, W - pad - w - 12, 30, 'PUSH TO MASTER · %d' % n, CRUST if n else SUBTEXT, PEACH if n else SURF1,
             self.f_row, anchor='e', h=28, padx=14, tags=('push',))
        if self.error:
            sub, color = shorten(self.error, self.f_body, 420), RED
        elif self.pull_ok is False:
            sub, color = shorten(self.pull_msg, self.f_body, 420), YELLOW
        elif pulled is not None:
            sub, color = 'copy of abuela from %s' % datetime.fromtimestamp(pulled).strftime('%H:%M:%S'), OVERLAY
        else:
            sub, color = self.pull_msg or 'fetching the copy…', OVERLAY
        c.create_text(W - pad, 58, text=sub, anchor='e', fill=color, font=self.f_body)

    def draw_tiles(self, x0, y0, x1, y1):
        d = self.data
        since_txt = datetime.strptime(self.since, TS).strftime('%H:%M') if parse_ts(self.since) else self.since
        if d is None:
            tiles = [('LABELLED THIS RUN', '–', 'since %s' % since_txt, TEAL, None),
                     ('TAGGED ITEMS', '–', '', BLUE, None),
                     ('ON SHELF', '–', '', GREEN, None),
                     ('EXPIRED ON SHELF', '–', '', OVERLAY, None)]
        else:
            total = sum(n for _, n, _ in d['stock'])
            on_shelf = sum(s for _, _, s in d['stock'])
            t = parse_ts(d['last_scan'])
            if self.shelf_live():
                shelf_txt = 'robot scanning'
            elif t is None:
                shelf_txt = 'no shelf scans yet'
            else:
                shelf_txt = 'last scan %s' % ago(time.time() - t)
            n_exp = len(d['expired'])
            tiles = [('LABELLED THIS RUN', str(len(d['session'])),
                      ('%d not pushed' % d['pending']) if d['pending'] else 'since %s' % since_txt, TEAL, None),
                     ('TAGGED ITEMS', str(total), '%d kinds' % len(d['stock']), BLUE, None),
                     ('ON SHELF', str(on_shelf), shelf_txt, GREEN, None),
                     ('EXPIRED ON SHELF', str(n_exp),
                      'click to see them' if n_exp else 'none',
                      RED if n_exp else OVERLAY, 'expired' if n_exp else None)]
        c, gap = self.canvas, 14
        w = (x1 - x0 - gap * (len(tiles) - 1)) / len(tiles)
        for i, (label, value, caption, color, tag) in enumerate(tiles):
            tx = x0 + i * (w + gap)
            tags = (tag,) if tag else ()
            self.card(tx, y0, tx + w, y1, outline=color if tag else SURF0, width=2 if tag else 1, tags=tags)
            c.create_rectangle(tx + 18, y0 + 18, tx + 22, y0 + 32, fill=color, outline='', tags=tags)
            c.create_text(tx + 30, y0 + 25, text=label, anchor='w', fill=SUBTEXT, font=self.f_small, tags=tags)
            c.create_text(tx + 18, y0 + 56, text=value, anchor='w', fill=color, font=self.f_big, tags=tags)
            c.create_text(tx + 18, y0 + 82, text=shorten(caption, self.f_body, w - 36), anchor='w',
                          fill=OVERLAY, font=self.f_body, tags=tags)

    def draw_waiting(self, x0, y0, x1, y1):
        c = self.canvas
        self.card(x0, y0, x1, y1)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        box_icon(c, cx, cy - 50, 34)
        c.create_text(cx, cy + 12, text='Could not read the inventory', fill=TEXT, font=self.f_h2)
        c.create_text(cx, cy + 40, fill=OVERLAY, font=self.f_body, justify='center',
                      text=self.error or '')

    def draw_hero(self, x0, y0, x1, y1):
        c = self.canvas
        session = self.data['session']
        flashing = session and session[0][0] == self.flash_uid and time.time() < self.flash_until
        self.card(x0, y0, x1, y1, outline=GREEN if flashing else SURF0, width=3 if flashing else 1)
        c.create_text(x0 + 20, y0 + 22, text='LATEST LABEL', anchor='w', fill=SUBTEXT, font=self.f_small)
        if not session:
            cy = (y0 + y1) / 2 + 8
            box_icon(c, x0 + 70, cy, 26)
            c.create_text(x0 + 120, cy - 12, anchor='w', fill=TEXT, font=self.f_h2,
                          text='Waiting for the first label')
            c.create_text(x0 + 120, cy + 14, anchor='w', fill=OVERLAY, font=self.f_body,
                          text='Labels show up here as arm 2 applies and saves them.')
            return
        uid, name, expiry, labelled, no, status = session[0]
        size = y1 - y0 - 58
        self.picture(name, x0 + 20 + size / 2, y0 + 40 + size / 2, size)
        tx, right = x0 + 40 + size, x1 - 20
        t = parse_ts(labelled)
        when = ago(time.time() - t) if t else labelled
        if flashing:
            pill(c, right, y0 + 22, 'NEW', CRUST, GREEN, self.f_small, anchor='e', h=22)
        else:
            c.create_text(right, y0 + 22, text=when, anchor='e', fill=OVERLAY, font=self.f_body)
        c.create_text(tx, y0 + 56, text=shorten(name, self.f_name, right - tx), anchor='w',
                      fill=TEXT, font=self.f_name)
        c.create_text(tx, y0 + 88, text='Item %d · %s' % (no, tag_groups(uid)), anchor='w', fill=SUBTEXT,
                      font=self.f_mono_big)
        self.push_pill(right, y0 + 122, status)
        words, color = expiry_look(expiry)
        w = pill(c, tx, y0 + 122, words, color, SURF0, self.f_row, h=26)
        if flashing:
            c.create_text(tx + w + 12, y0 + 122, text=when, anchor='w', fill=OVERLAY, font=self.f_body)

    def draw_list(self, x0, y0, x1, y1):
        c = self.canvas
        session = self.data['session']
        self.card(x0, y0, x1, y1)
        c.create_text(x0 + 20, y0 + 26, text='This run', anchor='w', fill=TEXT, font=self.f_h2)
        c.create_text(x1 - 20, y0 + 26, anchor='e', fill=OVERLAY, font=self.f_body,
                      text='%d label%s' % (len(session), '' if len(session) == 1 else 's'))
        top, row_h = y0 + 50, 54
        if not session:
            c.create_text((x0 + x1) / 2, (top + y1) / 2, text='No labels yet this run',
                          fill=OVERLAY, font=self.f_body)
            return
        fits = max(1, int((y1 - 12 - top) // row_h))
        self.scroll['list'] = start = min(self.scroll['list'], max(0, len(session) - fits))
        self.regions['list'] = (x0, top, x1, y1)
        now = time.time()
        for i, (uid, name, expiry, labelled, no, status) in enumerate(session[start:start + fits]):
            ry = top + i * row_h
            cy = ry + row_h / 2
            if session[start + i][0] == self.flash_uid and now < self.flash_until:
                rrect(c, x0 + 8, ry + 3, x1 - 8, ry + row_h - 3, 10, fill='#253a2c', outline='')
            elif i:
                c.create_line(x0 + 20, ry, x1 - 20, ry, fill=SURF0)
            self.picture(name, x0 + 40, cy, 40)
            t = parse_ts(labelled)
            when = ('pushed · ' if status == 'pushed' else 'not pushed · ') + (ago(now - t) if t else '')
            right = x1 - 20
            c.create_text(right, cy - 9, text=when, anchor='e', fill=GREEN if status == 'pushed' else PEACH,
                          font=self.f_body)
            words, color = expiry_look(expiry)
            c.create_text(right, cy + 10, text=words, anchor='e', fill=color, font=self.f_body)
            text_w = right - (x0 + 72) - max(self.f_body.measure(when), self.f_body.measure(words)) - 16
            c.create_text(x0 + 72, cy - 9, text=shorten(name, self.f_row, text_w), anchor='w',
                          fill=TEXT, font=self.f_row)
            c.create_text(x0 + 72, cy + 10, text='Item %d · tag …%s' % (no, tag_groups(uid[-8:])), anchor='w',
                          fill=OVERLAY, font=self.f_mono)
        self.more_note(x0, x1, y1, start, len(session) - start - fits)

    def push_pill(self, right, y, status):
        """'Pushed to abuela' / 'Not pushed yet' at the right end of a line."""
        if status == 'pushed':
            pill(self.canvas, right, y, 'Pushed to abuela', GREEN, '#253a2c', self.f_row, anchor='e', h=26)
        else:
            pill(self.canvas, right, y, 'Not pushed yet', PEACH, '#3a2f26', self.f_row, anchor='e', h=26)

    def more_note(self, x0, x1, y1, above, below):
        parts = (['%d above' % above] if above > 0 else []) + (['%d more below' % below] if below > 0 else [])
        if parts:
            self.canvas.create_text((x0 + x1) / 2, y1 - 10, text='scroll · ' + ' · '.join(parts),
                                    fill=OVERLAY, font=self.f_small)

    def draw_stock(self, x0, y0, x1, y1):
        c = self.canvas
        self.card(x0, y0, x1, y1)
        c.create_text(x0 + 20, y0 + 26, text='Stock', anchor='w', fill=TEXT, font=self.f_h2)
        tab_x = x1 - 20
        for tab, label, tag in (('all', 'All tagged', 'tab_all'), ('shelf', 'On shelf', 'tab_shelf')):
            on = self.stock_tab == tab
            w = pill(c, tab_x, y0 + 26, label, CRUST if on else SUBTEXT, (GREEN if tab == 'shelf' else BLUE)
                     if on else SURF0, self.f_row, anchor='e', h=26, tags=(tag,))
            tab_x -= w + 8
        shelf = self.stock_tab == 'shelf'
        rows = [(n, s if shelf else total) for n, total, s in self.data['stock'] if (s if shelf else total) > 0]
        rows.sort(key=lambda r: (-r[1], r[0]))
        top, row_h = y0 + 56, 50
        if not rows:
            cy = (top + y1) / 2
            if not self.data['have_copy']:
                head, sub = "No copy of abuela's inventory yet", self.pull_msg or 'fetching it…'
            elif shelf:
                head, sub = 'Nothing on the shelf', 'The robot did not see any tags in its last scan.'
            else:
                head, sub = 'No tagged items', ''
            c.create_text((x0 + x1) / 2, cy - 12, fill=TEXT, font=self.f_row, text=head)
            c.create_text((x0 + x1) / 2, cy + 12, fill=OVERLAY, font=self.f_body, text=shorten(sub, self.f_body, x1 - x0 - 40))
            return
        fits = max(1, int((y1 - 12 - top) // row_h))
        self.scroll['stock'] = start = min(self.scroll['stock'], max(0, len(rows) - fits))
        self.regions['stock'] = (x0, top, x1, y1)
        biggest = max(n for _, n in rows)
        color = GREEN if shelf else BLUE
        for i, (name, n) in enumerate(rows[start:start + fits]):
            ry = top + i * row_h
            count = str(n)
            c.create_text(x1 - 20, ry + 18, text=count, anchor='e', fill=TEXT, font=self.f_h2)
            name_w = x1 - 20 - self.f_h2.measure(count) - 16 - (x0 + 20)
            c.create_text(x0 + 20, ry + 18, text=shorten(name, self.f_body, name_w), anchor='w',
                          fill=SUBTEXT, font=self.f_body)
            bar_x1 = x1 - 20
            rrect(c, x0 + 20, ry + 34, bar_x1, ry + 40, 3, fill=SURF0, outline='')
            rrect(c, x0 + 20, ry + 34, x0 + 20 + max(6, (bar_x1 - x0 - 20) * n / biggest), ry + 40, 3,
                  fill=color, outline='')
        self.more_note(x0, x1, y1, start, len(rows) - start - fits)

    def show_push(self):
        if self.push_win is not None and self.push_win.alive():
            self.push_win.top.lift()
            return
        self.push_win = PushWindow(self)

    # ── Expired list ──

    def show_expired(self):
        expired = self.data and self.data['expired']
        if not expired:
            return
        if self.popup is not None and self.popup.winfo_exists():
            self.popup.destroy()
        top = self.popup = tk.Toplevel(self.root, bg=CRUST)
        top.title('Expired items on the shelf')
        top.transient(self.root)
        tk.Label(top, text='Remove these from the shelf', bg=CRUST, fg=RED, font=self.f_h2,
                 anchor='w').pack(fill='x', padx=20, pady=(18, 4))
        tk.Label(top, text='%d expired item%s on the shelf in the last scan' % (len(expired), '' if len(expired) == 1 else 's'),
                 bg=CRUST, fg=OVERLAY, font=self.f_body, anchor='w').pack(fill='x', padx=20, pady=(0, 10))
        body = tk.Frame(top, bg=BASE, padx=16, pady=10)
        body.pack(fill='both', expand=True, padx=20)
        for i, (uid, name, expiry) in enumerate(expired[:14]):
            row = tk.Frame(body, bg=BASE)
            row.pack(fill='x', pady=4)
            tk.Label(row, text=name, bg=BASE, fg=TEXT, font=self.f_row, anchor='w').pack(side='left')
            tk.Label(row, text=expiry_look(expiry)[0], bg=BASE, fg=RED, font=self.f_body).pack(side='right')
            tk.Label(row, text='…' + uid[-8:], bg=BASE, fg=OVERLAY, font=self.f_mono).pack(side='right', padx=14)
        if len(expired) > 14:
            tk.Label(body, text='and %d more' % (len(expired) - 14), bg=BASE, fg=OVERLAY,
                     font=self.f_body).pack(anchor='w', pady=4)
        tk.Button(top, text='Close', command=top.destroy, bg=SURF1, fg=TEXT, activebackground=SURF2,
                  activeforeground=TEXT, relief='flat', font=self.f_row, padx=18, pady=6,
                  highlightthickness=0, bd=0).pack(pady=14)


# ── Push to abuela ───────────────────────────────────────────────────────────

class Scrolled(tk.Frame):
    """A frame with a vertical scrollbar; put rows in .inner."""

    def __init__(self, parent, height):
        super().__init__(parent, bg=BASE)
        self.canvas = tk.Canvas(self, bg=BASE, highlightthickness=0, height=height)
        bar = tk.Scrollbar(self, orient='vertical', command=self.canvas.yview)
        self.inner = tk.Frame(self.canvas, bg=BASE)
        win = self.canvas.create_window(0, 0, window=self.inner, anchor='nw')
        self.inner.bind('<Configure>', lambda e: self.canvas.configure(scrollregion=self.canvas.bbox('all')))
        self.canvas.bind('<Configure>', lambda e: self.canvas.itemconfigure(win, width=e.width))
        self.canvas.configure(yscrollcommand=bar.set)
        self.canvas.pack(side='left', fill='both', expand=True)
        bar.pack(side='right', fill='y')
        for seq, step in (('<MouseWheel>', None), ('<Button-4>', -1), ('<Button-5>', 1)):
            self.bind_all_rows(seq, step)

    def bind_all_rows(self, seq, step):
        def wheel(e):
            if str(e.widget).startswith(str(self)):
                self.canvas.yview_scroll(step if step is not None else (-1 if e.delta > 0 else 1), 'units')
        self.bind_all(seq, wheel, add='+')

    def clear(self):
        for w in self.inner.winfo_children():
            w.destroy()


class PushWindow:
    """The labels not pushed yet, with a tick box each; the ticked ones are pushed to abuela or
    discarded. Below them, the labels already pushed."""

    def __init__(self, inv):
        self.inv = inv
        self.top = top = tk.Toplevel(inv.root, bg=CRUST)
        top.title('[ORIO] Push labels to abuela')
        top.geometry('900x640')
        top.minsize(700, 480)
        f = inv
        self.checks = {}          # rfid_uid -> BooleanVar
        self.shown = None         # (pending rows, history rows) last drawn
        self.busy = False

        tk.Label(top, text='Labels not pushed yet', bg=CRUST, fg=TEXT, font=f.f_h2,
                 anchor='w').pack(fill='x', padx=20, pady=(16, 0))
        tk.Label(top, bg=CRUST, fg=OVERLAY, font=f.f_body, anchor='w', justify='left',
                 text="Pushing adds each ticked label to abuela: master.db, item_names.csv and "
                      "tag_registry.csv.\nLabels you leave unticked stay here for later.").pack(fill='x', padx=20)
        self.pending_box = Scrolled(top, 230)
        self.pending_box.pack(fill='both', expand=True, padx=20, pady=(10, 6))

        bar = tk.Frame(top, bg=CRUST)
        bar.pack(fill='x', padx=20)
        btn = dict(relief='flat', bd=0, highlightthickness=0, font=f.f_row, padx=14, pady=6)
        tk.Button(bar, text='Select all', command=lambda: self.tick(True), bg=SURF1, fg=TEXT,
                  activebackground=SURF2, activeforeground=TEXT, **btn).pack(side='left', padx=(0, 8))
        tk.Button(bar, text='Select none', command=lambda: self.tick(False), bg=SURF1, fg=TEXT,
                  activebackground=SURF2, activeforeground=TEXT, **btn).pack(side='left')
        self.push_btn = tk.Button(bar, text='Push selected', command=self.push, bg=GREEN, fg=CRUST,
                                  activebackground='#c3ecbe', activeforeground=CRUST, **btn)
        self.push_btn.pack(side='right')
        self.discard_btn = tk.Button(bar, text='Discard selected', command=self.discard, bg=RED, fg=CRUST,
                                     activebackground='#f6b3c4', activeforeground=CRUST, **btn)
        self.discard_btn.pack(side='right', padx=8)
        self.status = tk.Label(top, text='', bg=CRUST, fg=OVERLAY, font=f.f_body, anchor='w')
        self.status.pack(fill='x', padx=20, pady=(6, 0))

        tk.Label(top, text='Pushed', bg=CRUST, fg=TEXT, font=f.f_h2, anchor='w').pack(fill='x', padx=20, pady=(12, 0))
        self.history_box = Scrolled(top, 150)
        self.history_box.pack(fill='both', expand=True, padx=20, pady=(6, 16))
        self.refresh()

    def alive(self):
        return self.top.winfo_exists()

    def refresh(self):
        """Redraw when the lists changed (a new label, a push, a discard); ticks are kept."""
        if not self.alive():
            return
        rows, done = master_sync.pending(), master_sync.history()
        key = ([tuple(r.values()) for r in rows], [tuple(r.values()) for r in done])
        if key != self.shown:
            self.shown = key
            self.draw_pending(rows)
            self.draw_history(done)
        self.top.after(2000, self.refresh)

    def draw_pending(self, rows):
        f, box = self.inv, self.pending_box
        box.clear()
        self.checks = {r['rfid_uid']: self.checks.get(r['rfid_uid']) or tk.BooleanVar(master=self.top, value=False)
                       for r in rows}
        if not rows:
            tk.Label(box.inner, text='Every label applied here has been pushed.', bg=BASE, fg=OVERLAY,
                     font=f.f_body).pack(pady=20)
            return
        for r in rows:
            row = tk.Frame(box.inner, bg=BASE)
            row.pack(fill='x', padx=8, pady=3)
            # On X11 the tick is drawn in the foreground colour (black by default).
            tk.Checkbutton(row, variable=self.checks[r['rfid_uid']], bg=BASE, activebackground=BASE,
                           fg=GREEN, activeforeground=GREEN,
                           selectcolor=SURF0, highlightthickness=0, bd=0).pack(side='left')
            tk.Label(row, text='Item %d' % r['item_no'], bg=BASE, fg=SUBTEXT, font=f.f_mono, width=10,
                     anchor='w').pack(side='left')
            words, color = expiry_look(r['expiration_date'])
            tk.Label(row, text=(r['date_labeled'] or '')[5:16], bg=BASE, fg=OVERLAY, font=f.f_body).pack(side='right')
            tk.Label(row, text='…' + r['rfid_uid'][-8:], bg=BASE, fg=OVERLAY, font=f.f_mono).pack(side='right', padx=12)
            tk.Label(row, text=words, bg=BASE, fg=color, font=f.f_body).pack(side='right', padx=6)
            tk.Label(row, text=r['item_name'], bg=BASE, fg=TEXT, font=f.f_row, anchor='w').pack(side='left', padx=6)
            if r['note']:
                tk.Label(box.inner, text='      last push failed: %s' % r['note'], bg=BASE, fg=RED, font=f.f_body,
                         anchor='w').pack(fill='x', padx=8)

    def draw_history(self, rows):
        f, box = self.inv, self.history_box
        box.clear()
        if not rows:
            tk.Label(box.inner, text='Nothing pushed yet.', bg=BASE, fg=OVERLAY, font=f.f_body).pack(pady=12)
            return
        for r in rows:
            row = tk.Frame(box.inner, bg=BASE)
            row.pack(fill='x', padx=8, pady=2)
            tk.Label(row, text='Item %d' % r['item_no'], bg=BASE, fg=SUBTEXT, font=f.f_mono, width=10,
                     anchor='w').pack(side='left')
            tk.Label(row, text='pushed %s' % (r['pushed_at'] or '')[5:16], bg=BASE, fg=GREEN,
                     font=f.f_body).pack(side='right')
            tk.Label(row, text='…' + r['rfid_uid'][-8:], bg=BASE, fg=OVERLAY, font=f.f_mono).pack(side='right', padx=12)
            tk.Label(row, text=r['expiration_date'] or '', bg=BASE, fg=OVERLAY, font=f.f_body).pack(side='right', padx=6)
            tk.Label(row, text=r['item_name'], bg=BASE, fg=TEXT, font=f.f_body, anchor='w').pack(side='left', padx=6)

    def tick(self, value):
        for var in self.checks.values():
            var.set(value)

    def selected(self):
        return [uid for uid, var in self.checks.items() if var.get()]

    def set_busy(self, busy, text, color=OVERLAY):
        self.busy = busy
        state = 'disabled' if busy else 'normal'
        self.push_btn.configure(state=state)
        self.discard_btn.configure(state=state)
        self.status.configure(text=text, fg=color)

    def push(self):
        uids = self.selected()
        if self.busy:
            return
        if not uids:
            self.status.configure(text='Tick the labels to push first.', fg=YELLOW)
            return
        self.set_busy(True, 'Pushing %d label%s to abuela…' % (len(uids), '' if len(uids) == 1 else 's'))

        self.result = None

        def work():   # no Tk calls in this thread: wait_push() picks the result up
            with self.inv.sync_lock:
                try:
                    self.result = master_sync.push(uids)
                except Exception as e:  # noqa: BLE001 — shown in the window
                    self.result = (False, str(e), {})
        threading.Thread(target=work, daemon=True).start()
        self.top.after(200, self.wait_push)

    def wait_push(self):
        if not self.alive():
            return
        if self.result is None:
            self.top.after(200, self.wait_push)
            return
        self.pushed(*self.result)

    def pushed(self, ok, msg, results):
        failed = [m for good, m in results.values() if not good]
        if not ok:
            self.set_busy(False, 'Not pushed: %s' % msg, RED)
        elif failed:
            self.set_busy(False, '%s. The others stay in the list; the reason is under each.' % msg, YELLOW)
        else:
            self.set_busy(False, msg[:1].upper() + msg[1:] + '.', GREEN)
        for uid, (good, _) in results.items():
            if good and uid in self.checks:
                self.checks[uid].set(False)
        self.shown = None

    def discard(self):
        uids = self.selected()
        if self.busy or not uids:
            if not uids:
                self.status.configure(text='Tick the labels to discard first.', fg=YELLOW)
            return
        if not messagebox.askyesno('Discard labels', 'Discard %d label%s? They will not be pushed to abuela.'
                                   % (len(uids), '' if len(uids) == 1 else 's'), parent=self.top):
            return
        n = master_sync.discard(uids)
        self.status.configure(text='%d label%s discarded.' % (n, '' if n == 1 else 's'), fg=OVERLAY)
        self.shown = None


def main():
    ap = argparse.ArgumentParser(description="Live view of abuela's inventory and the labels applied here.")
    ap.add_argument('--since', default=None,
                    help='count labels from this time, "YYYY-MM-DD HH:MM:SS" (default: now)')
    ap.add_argument('--hide-new', action='store_true', help='stock only, no label panels')
    ap.add_argument('--no-pull', action='store_true', help='do not refresh the copy from abuela')
    args = ap.parse_args()
    since = args.since or datetime.now().strftime(TS)
    if parse_ts(since) is None:
        ap.error('--since must look like "2026-11-10 14:30:00"')
    root = tk.Tk()
    InventoryWindow(root, since, show_new=not args.hide_new, pull=not args.no_pull)
    root.mainloop()


if __name__ == '__main__':
    main()
