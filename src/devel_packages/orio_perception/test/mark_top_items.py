#!/usr/bin/env python3
"""Click the item that is on top, zone by zone, to make the ground truth for the top-item rule.

Shows each label zone where the segmentation found two or more items: the clean picture on
the left, the same with the items outlined and lettered on the right. Click the item that
lies on top (in either picture) and the next zone comes up. Every mark is saved at once to
top_items.csv in the folder, so the window can be closed and reopened at any time; it
starts at the first zone without a mark.

    Click an item, or press A / B / C     that item is on top
    N  or the "Neither" button            neither lies on the other (side by side)
    ?  or the "Can't tell" button         cannot tell
    Left / Right arrows, Back / Skip      move without marking
    Q or Esc                              close

Run on the host, from the repo root (needs a display, tkinter, Pillow, numpy and OpenCV):

    src/devel_packages/orio_perception/venv/bin/python \\
        src/devel_packages/orio_perception/test/mark_top_items.py [folder]

folder: default logging/label_zones_seg/top_truth, as written with the zones' masks in
masks/ (index.json + one .npz per zone: rgb, masks, items).
"""
import json
import os
import sys
import tkinter as tk

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
PANEL = 640                                   # px, each of the two pictures
COLOURS = [(255, 70, 70), (70, 220, 70), (60, 150, 255)]   # A, B, C
LETTERS = "ABC"
HEADER = ("# Which item is on top in each zone? Fill the last column with A, B (or C), as lettered in the\n"
          "# mark_*.png pictures. Use - when neither lies on the other (side by side, not touching),\n"
          "# and ? when you cannot tell. Leave the other columns as they are.\n"
          "number,setup,zone,choices,top\n")


def outline(mask, width=2):
    """The pixels of a mask within `width` of its edge."""
    inner = mask.copy()
    for _ in range(width):
        m = inner
        inner = m.copy()
        inner[1:] &= m[:-1]
        inner[:-1] &= m[1:]
        inner[:, 1:] &= m[:, :-1]
        inner[:, :-1] &= m[:, 1:]
    return mask & ~inner


class Zone:
    """One zone's pictures (PIL, PANEL x PANEL) and what is needed to map a click to an item."""

    def __init__(self, folder, entry):
        self.entry = entry
        z = np.load(os.path.join(folder, "masks", entry["key"] + ".npz"))
        rgb, self.masks = z["rgb"], z["masks"].astype(bool)
        # How far inside each mask a pixel is: the letter goes on the deepest point, and a
        # click where two masks overlap a little counts for the one it is deeper inside.
        self.depth = np.stack([cv2.distanceTransform(m.astype(np.uint8), cv2.DIST_L2, 3) for m in self.masks])
        ys, xs = np.nonzero(self.masks.any(axis=0))
        cx, cy = (xs.min() + xs.max()) // 2, (ys.min() + ys.max()) // 2
        half = int(max(xs.max() - xs.min(), ys.max() - ys.min()) / 2 * 1.15) + 10
        self.x0, self.y0 = max(cx - half, 0), max(cy - half, 0)
        x1, y1 = min(cx + half, rgb.shape[1]), min(cy + half, rgb.shape[0])
        crop = rgb[self.y0:y1, self.x0:x1]
        self.scale = PANEL / float(max(crop.shape[:2]))
        size = (int(round(crop.shape[1] * self.scale)), int(round(crop.shape[0] * self.scale)))
        self.clean = Image.new("RGB", (PANEL, PANEL))
        self.clean.paste(Image.fromarray(crop).resize(size, Image.BICUBIC))
        marked = crop.copy()
        for k, m in enumerate(self.masks):
            marked[outline(m[self.y0:y1, self.x0:x1])] = COLOURS[k % 3]
        self.marked = Image.new("RGB", (PANEL, PANEL))
        self.marked.paste(Image.fromarray(marked).resize(size, Image.BICUBIC))
        draw = ImageDraw.Draw(self.marked)
        try:
            font = ImageFont.truetype("DejaVuSans-Bold.ttf", 26)
        except OSError:
            font = ImageFont.load_default()
        self.letter_px = []   # (x, y) of each item's letter, zone pixels
        for k in range(len(self.masks)):
            y, x = np.unravel_index(int(np.argmax(self.depth[k])), self.depth[k].shape)
            self.letter_px.append((int(x), int(y)))
            u, v = (x - self.x0) * self.scale, (y - self.y0) * self.scale
            draw.ellipse([u - 22, v - 22, u + 22, v + 22], fill=(0, 0, 0), outline=COLOURS[k % 3], width=3)
            draw.text((u, v), LETTERS[k], fill=(255, 255, 255), font=font, anchor="mm")
        self.n = len(self.masks)

    def item_at(self, px, py):
        """Index of the item under a click at panel pixel (px, py), or None."""
        x, y = int(px / self.scale) + self.x0, int(py / self.scale) + self.y0
        if not (0 <= y < self.masks.shape[1] and 0 <= x < self.masks.shape[2]):
            return None
        hit = [k for k in range(self.n) if self.masks[k, y, x]]
        return max(hit, key=lambda k: self.depth[k, y, x]) if hit else None


class App:
    def __init__(self, folder, root):
        self.folder, self.root = folder, root
        self.csv = os.path.join(folder, "top_items.csv")
        with open(os.path.join(folder, "masks", "index.json")) as fh:
            self.index = json.load(fh)
        self.marks = self._read_marks()
        self.cache = {}
        self.i = next((i for i, e in enumerate(self.index) if not self.marks.get(e["number"])), 0)

        root.title("Which item is on top?")
        root.configure(bg="#1e1e2e")
        self.title = tk.Label(root, font=("DejaVu Sans", 15, "bold"), fg="#cdd6f4", bg="#1e1e2e")
        self.title.pack(pady=(10, 4))
        self.canvas = tk.Canvas(root, width=2 * PANEL + 12, height=PANEL, bg="black", highlightthickness=0)
        self.canvas.pack(padx=10)
        self.canvas.bind("<Button-1>", self._on_click)
        self.status = tk.Label(root, font=("DejaVu Sans", 13), fg="#f9e2af", bg="#1e1e2e")
        self.status.pack(pady=6)
        row = tk.Frame(root, bg="#1e1e2e")
        row.pack(pady=(0, 10))
        for text, cmd in (("< Back", lambda: self._go(-1)), ("Neither is on top (N)", lambda: self._mark("-")),
                          ("Can't tell (?)", lambda: self._mark("?")), ("Skip >", lambda: self._go(1))):
            tk.Button(row, text=text, command=cmd, font=("DejaVu Sans", 12), padx=10, pady=4).pack(side="left", padx=6)
        tk.Label(root, text="Click the item that lies on top (either picture), or press A / B / C.   "
                            "Left / Right arrows move.   Q closes; marks are saved as you go.",
                 font=("DejaVu Sans", 10), fg="#a6adc8", bg="#1e1e2e").pack(pady=(0, 10))
        root.bind("<Key>", self._on_key)
        self._show()

    # ── marks file ────────────────────────────────────────────────────────────
    def _read_marks(self):
        marks = {}
        if os.path.exists(self.csv):
            for line in open(self.csv):
                parts = line.strip().split(",")
                if len(parts) == 5 and parts[0].isdigit():
                    marks[int(parts[0])] = parts[4].strip().upper()
        return marks

    def _write_marks(self):
        tmp = self.csv + ".tmp"
        with open(tmp, "w") as fh:
            fh.write(HEADER)
            for e in self.index:
                n = self._zone(e).n if e["key"] in self.cache else self._n_items(e)
                fh.write("%02d,%s,%s,%s,%s\n" % (e["number"], e["setup"], e["zone"],
                                               "/".join(LETTERS[:n]), self.marks.get(e["number"], "")))
        os.replace(tmp, self.csv)

    def _n_items(self, e):
        return int(np.load(os.path.join(self.folder, "masks", e["key"] + ".npz"))["masks"].shape[0])

    # ── view ──────────────────────────────────────────────────────────────────
    def _zone(self, e):
        if e["key"] not in self.cache:
            self.cache[e["key"]] = Zone(self.folder, e)
        return self.cache[e["key"]]

    def _show(self):
        e = self.index[self.i]
        z = self._zone(e)
        self.photos = (ImageTk.PhotoImage(z.clean), ImageTk.PhotoImage(z.marked))   # keep references
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, image=self.photos[0], anchor="nw")
        self.canvas.create_image(PANEL + 12, 0, image=self.photos[1], anchor="nw")
        done = sum(1 for x in self.index if self.marks.get(x["number"]))
        self.title.config(text="#%02d of %d    %s  zone %s        (%d marked)"
                               % (e["number"], len(self.index), e["setup"], e["zone"], done))
        mark = self.marks.get(e["number"], "")
        text = {"": "not marked yet", "-": "marked: neither is on top", "?": "marked: can't tell"}.get(
            mark, "marked: %s is on top" % mark)
        if done == len(self.index):
            text += "        ALL %d ZONES MARKED - you can close the window" % done
        self.status.config(text=text)

    def _go(self, step):
        self.i = min(max(self.i + step, 0), len(self.index) - 1)
        self._show()

    def _mark(self, value):
        e = self.index[self.i]
        self.marks[e["number"]] = value
        self._write_marks()
        if self.i < len(self.index) - 1:
            self.i += 1
        self._show()

    def _on_click(self, ev):
        x = ev.x - (PANEL + 12) if ev.x >= PANEL + 12 else ev.x
        k = self._zone(self.index[self.i]).item_at(x, ev.y)
        if k is None:
            self.status.config(text="that click was not on an item - click inside one of the outlines")
            return
        self._mark(LETTERS[k])

    def _on_key(self, ev):
        key = ev.keysym.lower()
        n = self._zone(self.index[self.i]).n
        if key in ("a", "b", "c") and LETTERS.lower().index(key) < n:
            self._mark(key.upper())
        elif key in ("n", "minus"):
            self._mark("-")
        elif key in ("question", "slash"):
            self._mark("?")
        elif key in ("left", "backspace"):
            self._go(-1)
        elif key in ("right", "space"):
            self._go(1)
        elif key in ("q", "escape"):
            self.root.destroy()


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, "logging", "label_zones_seg", "top_truth")
    if not os.path.exists(os.path.join(folder, "masks", "index.json")):
        sys.exit("no masks/index.json in %s" % folder)
    root = tk.Tk()
    App(folder, root)
    root.mainloop()


if __name__ == "__main__":
    main()
