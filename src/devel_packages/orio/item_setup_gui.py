"""Item setup window shown before a run: pick the item being labelled and its expiry.

The item is chosen from a dropdown of known items, each with a picture. Choosing
"Other…" opens a field for a new item name. Known items come from:

  * abuela's item_names.csv            the master inventory's names (this PC's copy,
                                       orio_rfid/master_sync/copy/, see master_sync.py)
  * orio_rfid/item_catalog/local_items.json
                                       names typed in through "Other…" (git-ignored)

Pictures are abuela's item_photos (the same copy); items without one get
item_catalog/images/default.png.
"""

import datetime
import json
import os
import re
import sys
import tkinter as tk
from tkinter import font as tkfont, messagebox

_HERE = os.path.dirname(os.path.abspath(__file__))
RFID_DIR = os.path.normpath(os.path.join(_HERE, '..', 'orio_rfid'))
CATALOG_DIR = os.path.join(RFID_DIR, 'item_catalog')
LOCAL_ITEMS_FILE = os.path.join(CATALOG_DIR, 'local_items.json')
DEFAULT_ICON = os.path.join(CATALOG_DIR, 'images', 'default.png')

sys.path.insert(0, RFID_DIR)
import master_sync  # noqa: E402

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


def load_items():
    """Known items as a sorted list of (name, image path or None), one per name."""
    items = {}  # lower-case name -> name
    for name in master_sync.name_list() + _read_json(LOCAL_ITEMS_FILE, []):
        name = (name or '').strip()
        if name:
            items.setdefault(name.lower(), name)
    return [(name, master_sync.photo_for(name)) for name in sorted(items.values(), key=str.lower)]


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
        pass  # not worth failing the run over; the label records the name too


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


def show_item_setup_gui(initial=None, title="Label Registration Setup",
                        submit_text="Start Run →", cancel_text="Cancel",
                        cancel_question="Abort the run?", should_close=None):
    """Blocking window that collects the item name and expiry date before a run.

    Returns (item_name, expiry). Closing the window or cancelling returns
    (None, None) so a run cannot start without them.

    full_sequence.py opens it again mid-run (the dashboard's CHANGE ITEM and ITEMS
    ADDED) with `initial` = the current (name, expiry) prefilled, its own title and
    button texts, cancel_question=None (cancel without asking), and `should_close`, a
    function polled every 300 ms that closes the window (as a cancel) when it is true.
    """
    result = {'name': None, 'expiry': None}
    items = load_items()

    root = tk.Tk()
    root.title("[ORIO] %s" % title)
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
    tk.Label(header, text="  ORIO — %s" % title,
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
        if cancel_question is None or messagebox.askyesno("Cancel", cancel_question, parent=root):
            root.destroy()

    tk.Button(btn_frame, text=submit_text, command=on_submit, bg=OK, fg=BG,
              font=('Helvetica', 12, 'bold'), relief='flat', padx=16, pady=6).pack(side='left', padx=(0, 12))
    tk.Button(btn_frame, text=cancel_text, command=on_cancel, bg=ERR, fg=BG,
              font=('Helvetica', 12, 'bold'), relief='flat', padx=16, pady=6).pack(side='left')

    root.protocol("WM_DELETE_WINDOW", on_cancel)
    root.bind('<Return>', lambda _: on_submit())
    item_btn.focus_set()

    if initial and initial[0]:   # the current item, prefilled
        if initial[0] in icons:
            select(initial[0])
        else:
            select(OTHER)
            new_var.set(initial[0])
        if initial[1]:
            expiry_var.set(initial[1])
        err_var.set("")

    if should_close is not None:
        def poll():
            if should_close():
                root.destroy()
                return
            root.after(300, poll)
        root.after(300, poll)
    root.lift()
    root.mainloop()

    return result['name'], result['expiry']  # both None if cancelled


if __name__ == '__main__':
    print(show_item_setup_gui())
