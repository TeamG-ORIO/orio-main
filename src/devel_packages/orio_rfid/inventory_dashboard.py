#!/usr/bin/env python3
"""Inventory window: what the RFID node has put in the database, live.

run_full_demo.sh opens it on the host when it starts the RFID node (--no-inventory to
skip). By hand, from any folder:
    python3 src/devel_packages/orio_rfid/inventory_dashboard.py

Reads medical_inventory.db next to this file (read-only) once a second and shows:
  - the newest label large, and every label registered since the window opened;
  - totals: labelled this run, tagged items, on the shelf, expired on the shelf;
  - stock per item, either what the shelf reader sees or everything tagged.
Click the expired tile to list the expired items. Needs Python 3 with tkinter;
Pillow adds the item pictures from item_catalog/.

  --db PATH      another database file
  --since TIME   count labels from TIME ("YYYY-MM-DD HH:MM:SS") instead of from now
  --hide-new     stock only, no label panels
"""

import argparse
import json
import os
import sqlite3
import time
import tkinter as tk
from datetime import date, datetime
from tkinter import font as tkfont
from urllib.request import pathname2url

try:
    from PIL import Image, ImageTk
except ImportError:  # no pictures then, a drawn box instead
    Image = ImageTk = None

HERE = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(HERE, 'medical_inventory.db')
CATALOG_DIR = os.path.join(HERE, 'item_catalog')
POLL_MS = 1000
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


def load_catalog():
    """item name -> picture path, from item_catalog/items.json."""
    pictures = {}
    try:
        with open(os.path.join(CATALOG_DIR, 'items.json')) as f:
            for item in json.load(f).get('items', []):
                if item.get('image'):
                    pictures[item['name']] = os.path.join(CATALOG_DIR, 'images', item['image'])
    except (OSError, ValueError) as e:
        print('item catalog not read: %s' % e, flush=True)
    return pictures


def read_db(path, since):
    """A snapshot of the database, or None when the file does not exist yet."""
    if not os.path.exists(path):
        return None
    conn = sqlite3.connect('file:%s?mode=ro' % pathname2url(path), uri=True, timeout=1.0)
    try:
        c = conn.cursor()
        session = c.execute(
            'SELECT rfid_uid, item_name, expiration_date, date_labeled FROM medical_supplies '
            'WHERE date_labeled >= ? ORDER BY date_labeled DESC, rowid DESC', (since,)).fetchall()
        stock = c.execute(
            'SELECT item_name, COUNT(*), COALESCE(SUM(is_on_shelf), 0) FROM medical_supplies '
            'GROUP BY item_name ORDER BY 2 DESC, 1').fetchall()
        expired = c.execute(
            'SELECT rfid_uid, item_name, expiration_date FROM medical_supplies '
            'WHERE is_on_shelf = 1 AND expiration_date < ? ORDER BY expiration_date',
            (date.today().isoformat(),)).fetchall()
        last_scan = c.execute('SELECT MAX(last_seen) FROM medical_supplies').fetchone()[0]
    finally:
        conn.close()
    return {'session': session, 'stock': stock, 'expired': expired, 'last_scan': last_scan}


# ── Window ───────────────────────────────────────────────────────────────────

class InventoryWindow:
    def __init__(self, root, db_path, since, show_new=True):
        self.root, self.db_path, self.since, self.show_new = root, db_path, since, show_new
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

        self.pictures = load_catalog()
        self.thumbs = {}
        self.data, self.error, self.updated = None, None, None
        self.known = None           # rfid_uids seen in an earlier poll
        self.flash_uid, self.flash_until = None, 0.0
        self.stock_tab = None       # 'shelf' or 'all'; picked on the first data
        self.scroll = {'list': 0, 'stock': 0}
        self.regions = {}           # scroll region name -> (x0, y0, x1, y1)
        self.popup = None

        c = self.canvas
        c.bind('<Configure>', lambda e: self.draw())
        for seq, step in (('<MouseWheel>', None), ('<Button-4>', -1), ('<Button-5>', 1)):
            c.bind(seq, lambda e, step=step: self.on_wheel(e, step))
        self.clickable('expired', self.show_expired)
        self.clickable('tab_shelf', lambda: self.set_tab('shelf'))
        self.clickable('tab_all', lambda: self.set_tab('all'))
        self.poll()

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
            self.data = read_db(self.db_path, self.since)
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
        except sqlite3.Error as e:
            self.error = str(e)
        self.draw()
        self.root.after(POLL_MS, self.poll)

    def shelf_live(self):
        t = parse_ts(self.data and self.data['last_scan'])
        return t is not None and time.time() - t <= SHELF_LIVE_S

    def thumb(self, name, size):
        """A PhotoImage of the item's picture fitted in size x size, or None."""
        path = self.pictures.get(name)
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
                      text='RFID labels and shelf stock', fill=OVERLAY, font=self.f_body)
        if self.error:
            look = ('●  Database error', RED, '#3a2530')
        elif self.data is None:
            look = ('●  Waiting for database', YELLOW, '#3a3426')
        else:
            look = ('●  Live', GREEN, '#253a2c')
        pill(c, W - pad, 30, look[0], look[1], look[2], self.f_row, anchor='e', h=28, padx=14)
        if self.error:
            sub = shorten(self.error, self.f_body, 420)
        elif self.updated:
            sub = 'updated %s' % datetime.fromtimestamp(self.updated).strftime('%H:%M:%S')
        else:
            sub = os.path.basename(self.db_path)
        c.create_text(W - pad, 58, text=sub, anchor='e', fill=RED if self.error else OVERLAY,
                      font=self.f_body)

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
                shelf_txt = 'shelf reader live'
            elif t is None:
                shelf_txt = 'no shelf scans yet'
            else:
                shelf_txt = 'last scan %s' % ago(time.time() - t)
            n_exp = len(d['expired'])
            tiles = [('LABELLED THIS RUN', str(len(d['session'])), 'since %s' % since_txt, TEAL, None),
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
        c.create_text(cx, cy + 12, text='Waiting for the inventory database', fill=TEXT, font=self.f_h2)
        c.create_text(cx, cy + 40, fill=OVERLAY, font=self.f_body, justify='center',
                      text='The RFID node creates it when it first starts:\n%s' % self.db_path)

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
                          text='Labels show up here as arm 2 applies and registers them.')
            return
        uid, name, expiry, labelled = session[0]
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
        c.create_text(tx, y0 + 88, text=tag_groups(uid), anchor='w', fill=SUBTEXT, font=self.f_mono_big)
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
        for i, (uid, name, expiry, labelled) in enumerate(session[start:start + fits]):
            ry = top + i * row_h
            cy = ry + row_h / 2
            if session[start + i][0] == self.flash_uid and now < self.flash_until:
                rrect(c, x0 + 8, ry + 3, x1 - 8, ry + row_h - 3, 10, fill='#253a2c', outline='')
            elif i:
                c.create_line(x0 + 20, ry, x1 - 20, ry, fill=SURF0)
            self.picture(name, x0 + 40, cy, 40)
            t = parse_ts(labelled)
            when = ago(now - t) if t else ''
            right = x1 - 20
            c.create_text(right, cy - 9, text=when, anchor='e', fill=OVERLAY, font=self.f_body)
            words, color = expiry_look(expiry)
            c.create_text(right, cy + 10, text=words, anchor='e', fill=color, font=self.f_body)
            text_w = right - (x0 + 72) - max(self.f_body.measure(when), self.f_body.measure(words)) - 16
            c.create_text(x0 + 72, cy - 9, text=shorten(name, self.f_row, text_w), anchor='w',
                          fill=TEXT, font=self.f_row)
            c.create_text(x0 + 72, cy + 10, text='tag …' + tag_groups(uid[-8:]), anchor='w',
                          fill=OVERLAY, font=self.f_mono)
        self.more_note(x0, x1, y1, start, len(session) - start - fits)

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
            c.create_text((x0 + x1) / 2, cy - 12, fill=TEXT, font=self.f_row,
                          text='Nothing on the shelf' if shelf else 'No tagged items')
            c.create_text((x0 + x1) / 2, cy + 12, fill=OVERLAY, font=self.f_body,
                          text='The shelf reader has not seen any tags recently.' if shelf else '')
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
        tk.Label(top, text='%d expired item%s the shelf reader can see' % (len(expired), '' if len(expired) == 1 else 's'),
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


def main():
    ap = argparse.ArgumentParser(description='Live view of the RFID inventory database.')
    ap.add_argument('--db', default=DB_PATH, help='database file (default: %(default)s)')
    ap.add_argument('--since', default=None,
                    help='count labels from this time, "YYYY-MM-DD HH:MM:SS" (default: now)')
    ap.add_argument('--hide-new', action='store_true', help='stock only, no label panels')
    args = ap.parse_args()
    since = args.since or datetime.now().strftime(TS)
    if parse_ts(since) is None:
        ap.error('--since must look like "2026-11-10 14:30:00"')
    root = tk.Tk()
    InventoryWindow(root, os.path.abspath(args.db), since, show_new=not args.hide_new)
    root.mainloop()


if __name__ == '__main__':
    main()
