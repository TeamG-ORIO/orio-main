"""Item setup window shown before a run: pick the item being labelled and its expiry.

The item is chosen from a dropdown of known items, each with a picture. Choosing
"Other…" opens a field for a new item name. Known items come from:

  * orio_rfid/item_catalog/items.json   names and pictures (tracked)
  * the RFID database                   every item name already labelled
  * orio_rfid/item_catalog/local_items.json
                                        names typed in through "Other…" (git-ignored)

Items without a picture get item_catalog/images/default.png.
"""

import datetime
import json
import os
import re
import sqlite3
import tkinter as tk
from tkinter import font as tkfont, messagebox

_HERE = os.path.dirname(os.path.abspath(__file__))
RFID_DIR = os.path.normpath(os.path.join(_HERE, '..', 'orio_rfid'))
CATALOG_DIR = os.path.join(RFID_DIR, 'item_catalog')
CATALOG_FILE = os.path.join(CATALOG_DIR, 'items.json')
LOCAL_ITEMS_FILE = os.path.join(CATALOG_DIR, 'local_items.json')
IMAGE_DIR = os.path.join(CATALOG_DIR, 'images')
DEFAULT_ICON = os.path.join(IMAGE_DIR, 'default.png')
DB_FILES = [os.path.join(RFID_DIR, 'medical_inventory.db'),        # runtime copy
            os.path.join(RFID_DIR, 'seed_medical_inventory.db')]   # tracked seed

ICON_SIZE = 40
MENU_ROWS = 12  # entries per dropdown column; longer lists wrap into more columns
OTHER = "Other…"
DEFAULT_EXPIRY = "2027-12-31"  # prefilled in the expiry field; the operator can change it

BG, FIELD_BG, FG, ACCENT, ERR, OK = '#1e1e2e', '#313244', '#cdd6f4', '#89b4fa', '#f38ba8', '#a6e3a1'


def _read_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def _db_item_names():
    """Item names in the RFID database (runtime copy if present, else the seed)."""
    for path in DB_FILES:
        if not os.path.exists(path):
            continue
        try:
            conn = sqlite3.connect('file:%s?mode=ro' % path, uri=True)
            try:
                rows = conn.execute("SELECT DISTINCT item_name FROM medical_supplies").fetchall()
            finally:
                conn.close()
            return [r[0] for r in rows if r[0] and r[0].strip()]
        except sqlite3.Error:
            continue
    return []


def load_items():
    """Known items as a sorted list of (name, image path or None), one per name."""
    items = {}  # lower-case name -> (name, image)

    def add(name, image=None):
        name = (name or '').strip()
        if not name:
            return
        key = name.lower()
        if key in items:
            if image and not items[key][1]:
                items[key] = (items[key][0], image)
            return
        items[key] = (name, image)

    for entry in _read_json(CATALOG_FILE, {}).get('items', []):
        image = entry.get('image')
        add(entry.get('name'), os.path.join(IMAGE_DIR, image) if image else None)
    for name in _db_item_names():
        add(name)
    for name in _read_json(LOCAL_ITEMS_FILE, []):
        add(name)
    return sorted(items.values(), key=lambda it: it[0].lower())


def remember_item(name):
    """Keep a name typed in through "Other…" so it is offered next time."""
    names = _read_json(LOCAL_ITEMS_FILE, [])
    if any(n.lower() == name.lower() for n in names):
        return
    names.append(name)
    try:
        with open(LOCAL_ITEMS_FILE, 'w') as f:
            json.dump(names, f, indent=2)
    except OSError:
        pass  # not worth failing the run over; the RFID database records it too


def _load_icon(master, path, default):
    """A Tk image of `path` fitted into an ICON_SIZE square, or `default`."""
    if not path:
        return default
    try:
        from PIL import Image, ImageTk
        im = Image.open(path).convert('RGBA')
        im.thumbnail((ICON_SIZE, ICON_SIZE), Image.LANCZOS)
        square = Image.new('RGBA', (ICON_SIZE, ICON_SIZE), (0, 0, 0, 0))
        square.paste(im, ((ICON_SIZE - im.width) // 2, (ICON_SIZE - im.height) // 2))
        return ImageTk.PhotoImage(square, master=master)
    except Exception:
        return default


def show_item_setup_gui():
    """Blocking window that collects the item name and expiry date before a run.

    Returns (item_name, expiry). Closing the window or cancelling returns
    (None, None) so a run cannot start without them.
    """
    result = {'name': None, 'expiry': None}
    items = load_items()

    root = tk.Tk()
    root.title("[ORIO] Label Registration Setup")
    root.configure(bg=BG)
    root.resizable(False, False)

    try:
        default_icon = tk.PhotoImage(file=DEFAULT_ICON, master=root)
    except tk.TclError:
        default_icon = tk.PhotoImage(width=ICON_SIZE, height=ICON_SIZE, master=root)
    icons = {name: _load_icon(root, image, default_icon) for name, image in items}

    # ── Header ──────────────────────────────────────────────────────────────
    header = tk.Frame(root, bg=ACCENT, pady=8)
    header.pack(fill='x')
    tk.Label(header, text="  ORIO — Label Registration Setup",
             font=('Helvetica', 14, 'bold'), fg=BG, bg=ACCENT).pack()

    # ── Fields ──────────────────────────────────────────────────────────────
    fields = tk.Frame(root, bg=BG, padx=20, pady=16)
    fields.pack(fill='x')

    def label(text, row):
        # Fixed width so the column does not shift when "New item name:" appears.
        w = tk.Label(fields, text=text, font=('Courier', 11), fg=FG, bg=BG,
                     anchor='w', justify='left', width=15)
        w.grid(row=row, column=0, sticky='w', pady=(0, 8))
        return w

    # Item dropdown: a menubutton whose menu entries carry the item pictures.
    label("Item:", 0)
    choice = {'name': None}
    # Wide enough for the longest name, so the window does not resize on each pick.
    btn_font = tkfont.Font(root=root, family='Courier', size=12)
    longest = max([btn_font.measure(n) for n, _ in items] + [btn_font.measure("Select an item…")])
    item_btn = tk.Menubutton(fields, text="Select an item…", image=default_icon,
                             compound='left', anchor='w', width=min(longest, 900) + ICON_SIZE + 40,
                             font=btn_font,
                             bg=FIELD_BG, fg=FG, activebackground='#45475a', activeforeground=FG,
                             relief='flat', indicatoron=True, padx=6, pady=4)
    item_btn.grid(row=0, column=1, sticky='w', padx=(12, 0), pady=(0, 8))
    menu = tk.Menu(item_btn, tearoff=False, font=('Courier', 11),
                   bg=FIELD_BG, fg=FG, activebackground=ACCENT, activeforeground=BG)
    item_btn['menu'] = menu

    # "New item name" row, shown only when "Other…" is chosen.
    new_label = label("New item name:", 1)
    new_label.grid_remove()
    new_var = tk.StringVar()
    new_entry = tk.Entry(fields, textvariable=new_var, font=('Courier', 12), bg=FIELD_BG, fg=FG,
                         insertbackground='white', relief='flat', width=30)

    def select(name):
        choice['name'] = name
        is_other = name == OTHER
        item_btn.config(text=name, image=default_icon if is_other else icons[name])
        if is_other:
            new_label.grid()
            new_entry.grid(row=1, column=1, sticky='ew', padx=(12, 0), pady=(0, 8))
            new_entry.focus_set()
        else:
            new_label.grid_remove()
            new_entry.grid_remove()
            expiry_entry.focus_set()
        err_var.set("")

    entries = [(name, icons[name]) for name, _ in items] + [(OTHER, default_icon)]
    for i, (name, icon) in enumerate(entries):
        if name == OTHER and i % MENU_ROWS:
            menu.add_separator()
        menu.add_command(label="  " + name, image=icon, compound='left',
                         columnbreak=1 if i and i % MENU_ROWS == 0 else 0,
                         command=lambda n=name: select(n))

    label("Expiry date\n(YYYY-MM-DD):", 2)
    expiry_var = tk.StringVar(value=DEFAULT_EXPIRY)
    expiry_entry = tk.Entry(fields, textvariable=expiry_var, font=('Courier', 12), bg=FIELD_BG,
                            fg=FG, insertbackground='white', relief='flat', width=30)
    expiry_entry.grid(row=2, column=1, sticky='ew', padx=(12, 0))

    # ── Error line (empty until validation fails) ───────────────────────────
    err_var = tk.StringVar()
    tk.Label(root, textvariable=err_var, font=('Courier', 10), fg=ERR, bg=BG).pack(padx=20, anchor='w')

    # ── Buttons ─────────────────────────────────────────────────────────────
    btn_frame = tk.Frame(root, bg=BG, pady=12)
    btn_frame.pack(fill='x', padx=20)

    def on_submit():
        name = choice['name']
        if name is None:
            err_var.set("Choose an item.")
            return
        new_item = name == OTHER
        if new_item:
            name = new_var.get().strip()
            if not name:
                err_var.set("Enter the new item's name.")
                return
            # Typing a name that is already listed picks that item, not a duplicate.
            known = {n.lower(): n for n, _ in items}
            new_item = name.lower() not in known
            name = known.get(name.lower(), name)
        expiry = expiry_var.get().strip()
        if not expiry:
            err_var.set("Expiry date cannot be empty.")
            return
        if not re.match(r'^\d{4}-\d{2}-\d{2}$', expiry):
            err_var.set("Expiry must be YYYY-MM-DD format.")
            return
        try:
            datetime.date.fromisoformat(expiry)
        except ValueError:
            err_var.set("Expiry is not a real date.")
            return
        if new_item:
            remember_item(name)
        result['name'] = name
        result['expiry'] = expiry
        root.destroy()

    def on_cancel():
        if messagebox.askyesno("Cancel", "Abort the run?", parent=root):
            root.destroy()

    tk.Button(btn_frame, text="Start Run →", command=on_submit, bg=OK, fg=BG,
              font=('Helvetica', 12, 'bold'), relief='flat', padx=16, pady=6).pack(side='left', padx=(0, 12))
    tk.Button(btn_frame, text="Cancel", command=on_cancel, bg=ERR, fg=BG,
              font=('Helvetica', 12, 'bold'), relief='flat', padx=16, pady=6).pack(side='left')

    root.protocol("WM_DELETE_WINDOW", on_cancel)
    root.bind('<Return>', lambda _: on_submit())
    item_btn.focus_set()
    root.lift()
    root.mainloop()

    return result['name'], result['expiry']  # both None if cancelled


if __name__ == '__main__':
    print(show_item_setup_gui())
