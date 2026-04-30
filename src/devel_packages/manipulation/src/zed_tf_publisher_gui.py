#!/usr/bin/env python3
"""
Interactive calibration GUI for the ZED Mini static TF.

Three linked sections – all stay in sync with each other:
  • Translation sliders + text boxes  (x, y, z  in metres)
  • Quaternion sliders + text boxes   (qx, qy, qz, qw  – auto-normalised)
  • RPY sliders + text boxes          (roll, pitch, yaw  in degrees)

Editing any widget updates the other two rotation representations live.

Buttons:
  Save to YAML      – overwrites zed_tf.yaml
  Reset from YAML   – reloads file and restores all widgets
  Undo (Ctrl+Z)     – steps back through last 50 changes
"""

import collections
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rospy
import rospkg
import yaml
import tf2_ros
import geometry_msgs.msg
import numpy as np
import scipy.spatial.transform as spt
from geometry_msgs.msg import Pose


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def pose_to_transformation_matrix(pose):
    T = np.eye(4)
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    r = spt.Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w])
    T[0:3, 0:3] = r.as_matrix()
    return T


def transformation_matrix_to_pose(T):
    p = geometry_msgs.msg.Pose()
    p.position.x, p.position.y, p.position.z = T[0, 3], T[1, 3], T[2, 3]
    q = spt.Rotation.from_matrix(T[0:3, 0:3]).as_quat()
    p.orientation.x, p.orientation.y = q[0], q[1]
    p.orientation.z, p.orientation.w = q[2], q[3]
    return p


def compute_zed_base_pose(panda_to_optical_pose, tf_buffer):
    """Chain panda→zedm_left_camera_optical_frame with optical→zedm_base_link."""
    try:
        t = tf_buffer.lookup_transform(
            'zedm_left_camera_optical_frame', 'zedm_base_link',
            rospy.Time(0), rospy.Duration(5.0))
        optical_to_base = Pose()
        optical_to_base.position.x = t.transform.translation.x
        optical_to_base.position.y = t.transform.translation.y
        optical_to_base.position.z = t.transform.translation.z
        optical_to_base.orientation.x = t.transform.rotation.x
        optical_to_base.orientation.y = t.transform.rotation.y
        optical_to_base.orientation.z = t.transform.rotation.z
        optical_to_base.orientation.w = t.transform.rotation.w
        M = np.matmul(pose_to_transformation_matrix(panda_to_optical_pose),
                      pose_to_transformation_matrix(optical_to_base))
        return transformation_matrix_to_pose(M)
    except Exception as e:
        rospy.logwarn_throttle(5.0, f"ZED TF lookup failed: {e}")
        return None


def broadcast_tf(pose, broadcaster):
    ts = geometry_msgs.msg.TransformStamped()
    ts.header.stamp = rospy.Time.now()
    ts.header.frame_id = "panda_link0"
    ts.child_frame_id = "zedm_base_link"
    ts.transform.translation.x = pose.position.x
    ts.transform.translation.y = pose.position.y
    ts.transform.translation.z = pose.position.z
    ts.transform.rotation.x = pose.orientation.x
    ts.transform.rotation.y = pose.orientation.y
    ts.transform.rotation.z = pose.orientation.z
    ts.transform.rotation.w = pose.orientation.w
    broadcaster.sendTransform(ts)


# ---------------------------------------------------------------------------
# GUI constants
# ---------------------------------------------------------------------------

T_RANGE   = 2.0
T_STEPS   = 4000
Q_RANGE   = 1.0
Q_STEPS   = 2000
RPY_RANGE = 180.0
RPY_STEPS = 3600


def _make_row(parent, row, label, slider_from, slider_to, slider_steps,
              slider_var, text_var, text_width=12, on_press=None):
    ttk.Label(parent, text=label, width=5, anchor="e").grid(
        row=row, column=0, padx=(4, 2))
    sl = ttk.Scale(parent, from_=slider_from, to=slider_to,
                   orient="horizontal", length=320, variable=slider_var)
    sl.grid(row=row, column=1, padx=4, pady=2)
    if on_press:
        sl.bind("<ButtonPress-1>", on_press)
    ent = ttk.Entry(parent, textvariable=text_var, width=text_width)
    ent.grid(row=row, column=2, padx=4)
    return sl, ent


# ---------------------------------------------------------------------------
# GUI class
# ---------------------------------------------------------------------------

class ZedCalibrationGUI:
    def __init__(self, root, yaml_path, tf_buffer):
        self.root = root
        self.yaml_path = yaml_path
        self.tf_buffer = tf_buffer
        self.dirty = threading.Event()
        self._updating = False
        self._history = collections.deque(maxlen=50)

        root.title("ZED Mini TF Calibration")
        root.resizable(False, False)

        init = self._load_yaml()
        tx, ty, tz = init["t"]
        qx, qy, qz, qw = init["q"]
        ro, pi_, ya = spt.Rotation.from_quat([qx, qy, qz, qw]).as_euler("xyz", degrees=True)

        # ---- Translation ------------------------------------------------
        tf_frame = ttk.LabelFrame(root, text="Translation (metres)", padding=8)
        tf_frame.grid(row=0, column=0, padx=10, pady=6, sticky="ew")

        self.t_sl, self.t_txt = {}, {}
        for i, (axis, val) in enumerate(zip(["x", "y", "z"], [tx, ty, tz])):
            sv = tk.DoubleVar(value=val)
            tv = tk.StringVar(value=f"{val:.5f}")
            self.t_sl[axis] = sv
            self.t_txt[axis] = tv
            _make_row(tf_frame, i, axis, -T_RANGE, T_RANGE, T_STEPS, sv, tv,
                      on_press=lambda _: self._push_snapshot())
            sv.trace_add("write", self._make_slider_cb_t(axis, sv, tv))
            tv.trace_add("write", self._make_text_cb_t(axis, tv, sv))

        # ---- Quaternion -------------------------------------------------
        qf = ttk.LabelFrame(root, text="Rotation – Quaternion (auto-normalised)", padding=8)
        qf.grid(row=1, column=0, padx=10, pady=6, sticky="ew")

        self.q_sl, self.q_txt = {}, {}
        for i, (axis, val) in enumerate(zip(["qx", "qy", "qz", "qw"], [qx, qy, qz, qw])):
            sv = tk.DoubleVar(value=val)
            tv = tk.StringVar(value=f"{val:.8f}")
            self.q_sl[axis] = sv
            self.q_txt[axis] = tv
            _make_row(qf, i, axis, -Q_RANGE, Q_RANGE, Q_STEPS, sv, tv,
                      on_press=lambda _: self._push_snapshot())
            sv.trace_add("write", self._make_slider_cb_q(axis, sv, tv))
            tv.trace_add("write", self._make_text_cb_q(axis, tv, sv))

        # ---- RPY --------------------------------------------------------
        rf = ttk.LabelFrame(root, text="Rotation – RPY (degrees, XYZ extrinsic)", padding=8)
        rf.grid(row=2, column=0, padx=10, pady=6, sticky="ew")

        self.rpy_sl, self.rpy_txt = {}, {}
        for i, (axis, val) in enumerate(zip(["roll", "pitch", "yaw"], [ro, pi_, ya])):
            sv = tk.DoubleVar(value=val)
            tv = tk.StringVar(value=f"{val:.4f}")
            self.rpy_sl[axis] = sv
            self.rpy_txt[axis] = tv
            _make_row(rf, i, axis, -RPY_RANGE, RPY_RANGE, RPY_STEPS, sv, tv,
                      on_press=lambda _: self._push_snapshot())
            sv.trace_add("write", self._make_slider_cb_rpy(axis, sv, tv))
            tv.trace_add("write", self._make_text_cb_rpy(axis, tv, sv))

        # ---- Buttons / status -------------------------------------------
        btn_frame = ttk.Frame(root, padding=6)
        btn_frame.grid(row=3, column=0, pady=4)
        ttk.Button(btn_frame, text="Save to YAML",
                   command=self._save).pack(side="left", padx=6)
        ttk.Button(btn_frame, text="Reset from YAML",
                   command=self._reset).pack(side="left", padx=6)
        self._undo_btn = ttk.Button(btn_frame, text="Undo",
                                    command=self._undo, state="disabled")
        self._undo_btn.pack(side="left", padx=6)
        root.bind("<Control-z>", lambda _: self._undo())

        self.status = ttk.Label(root, text="Ready", anchor="w", foreground="green")
        self.status.grid(row=4, column=0, padx=10, pady=(0, 6), sticky="ew")

        self.dirty.set()

    # ------------------------------------------------------------------
    # YAML
    # ------------------------------------------------------------------

    def _load_yaml(self):
        with open(self.yaml_path, "r") as f:
            d = yaml.safe_load(f)
        p = d["pose"]
        t = [p["translation"]["x"], p["translation"]["y"], p["translation"]["z"]]
        q = [p["rotation"]["x"], p["rotation"]["y"],
             p["rotation"]["z"], p["rotation"]["w"]]
        return {"t": t, "q": q}

    # ------------------------------------------------------------------
    # Undo
    # ------------------------------------------------------------------

    def _current_snapshot(self):
        return {
            "t":   {k: v.get() for k, v in self.t_sl.items()},
            "q":   {k: v.get() for k, v in self.q_sl.items()},
            "rpy": {k: v.get() for k, v in self.rpy_sl.items()},
        }

    def _push_snapshot(self):
        self._history.append(self._current_snapshot())
        self._undo_btn.config(state="normal")

    def _apply_snapshot(self, snap):
        self._updating = True
        try:
            for axis, val in snap["t"].items():
                self.t_sl[axis].set(val);  self.t_txt[axis].set(f"{val:.5f}")
            for axis, val in snap["q"].items():
                self.q_sl[axis].set(val);  self.q_txt[axis].set(f"{val:.8f}")
            for axis, val in snap["rpy"].items():
                self.rpy_sl[axis].set(val); self.rpy_txt[axis].set(f"{val:.4f}")
        finally:
            self._updating = False
        self.dirty.set()

    def _undo(self):
        if not self._history:
            return
        self._apply_snapshot(self._history.pop())
        if not self._history:
            self._undo_btn.config(state="disabled")
        self._set_status(f"Undo – {len(self._history)} step(s) remaining", "blue")

    # ------------------------------------------------------------------
    # Callbacks – translation
    # ------------------------------------------------------------------

    def _make_slider_cb_t(self, axis, sv, tv):
        def cb(*_):
            if self._updating:
                return
            self._updating = True
            try:
                tv.set(f"{sv.get():.5f}")
            finally:
                self._updating = False
            self.dirty.set()
        return cb

    def _make_text_cb_t(self, axis, tv, sv):
        def cb(*_):
            if self._updating:
                return
            try:
                val = float(tv.get())
            except ValueError:
                return
            self._push_snapshot()
            self._updating = True
            try:
                sv.set(val)
            finally:
                self._updating = False
            self.dirty.set()
        return cb

    # ------------------------------------------------------------------
    # Callbacks – quaternion (also syncs RPY)
    # ------------------------------------------------------------------

    def _make_slider_cb_q(self, axis, sv, tv):
        def cb(*_):
            if self._updating:
                return
            self._updating = True
            try:
                tv.set(f"{sv.get():.8f}")
                self._sync_rpy_from_quat()
            finally:
                self._updating = False
            self.dirty.set()
        return cb

    def _make_text_cb_q(self, axis, tv, sv):
        def cb(*_):
            if self._updating:
                return
            try:
                val = float(tv.get())
            except ValueError:
                return
            self._push_snapshot()
            self._updating = True
            try:
                sv.set(np.clip(val, -Q_RANGE, Q_RANGE))
                self._sync_rpy_from_quat()
            finally:
                self._updating = False
            self.dirty.set()
        return cb

    # ------------------------------------------------------------------
    # Callbacks – RPY (also syncs quaternion)
    # ------------------------------------------------------------------

    def _make_slider_cb_rpy(self, axis, sv, tv):
        def cb(*_):
            if self._updating:
                return
            self._updating = True
            try:
                tv.set(f"{sv.get():.4f}")
                self._sync_quat_from_rpy()
            finally:
                self._updating = False
            self.dirty.set()
        return cb

    def _make_text_cb_rpy(self, axis, tv, sv):
        def cb(*_):
            if self._updating:
                return
            try:
                val = float(tv.get())
            except ValueError:
                return
            self._push_snapshot()
            self._updating = True
            try:
                sv.set(np.clip(val, -RPY_RANGE, RPY_RANGE))
                self._sync_quat_from_rpy()
            finally:
                self._updating = False
            self.dirty.set()
        return cb

    # ------------------------------------------------------------------
    # Cross-sync helpers
    # ------------------------------------------------------------------

    def _get_quat_raw(self):
        return [self.q_sl[k].get() for k in ["qx", "qy", "qz", "qw"]]

    def _sync_rpy_from_quat(self):
        q = self._get_quat_raw()
        norm = np.linalg.norm(q)
        if norm < 1e-6:
            return
        q = [v / norm for v in q]
        rpy = spt.Rotation.from_quat(q).as_euler("xyz", degrees=True)
        for axis, val in zip(["roll", "pitch", "yaw"], rpy):
            self.rpy_sl[axis].set(np.clip(val, -RPY_RANGE, RPY_RANGE))
            self.rpy_txt[axis].set(f"{val:.4f}")

    def _sync_quat_from_rpy(self):
        rpy = [self.rpy_sl[k].get() for k in ["roll", "pitch", "yaw"]]
        q = spt.Rotation.from_euler("xyz", rpy, degrees=True).as_quat()
        for axis, val in zip(["qx", "qy", "qz", "qw"], q):
            self.q_sl[axis].set(np.clip(val, -Q_RANGE, Q_RANGE))
            self.q_txt[axis].set(f"{val:.8f}")

    # ------------------------------------------------------------------
    # get_pose
    # ------------------------------------------------------------------

    def get_pose(self):
        try:
            tx = self.t_sl["x"].get()
            ty = self.t_sl["y"].get()
            tz = self.t_sl["z"].get()
            q = self._get_quat_raw()
        except tk.TclError:
            return None
        norm = np.linalg.norm(q)
        if norm < 1e-6:
            return None
        q = [v / norm for v in q]
        p = Pose()
        p.position.x, p.position.y, p.position.z = tx, ty, tz
        p.orientation.x, p.orientation.y = q[0], q[1]
        p.orientation.z, p.orientation.w = q[2], q[3]
        return p

    # ------------------------------------------------------------------
    # Save / Reset
    # ------------------------------------------------------------------

    def _save(self):
        pose = self.get_pose()
        if pose is None:
            messagebox.showerror("Invalid values", "Quaternion is zero or non-numeric.")
            return
        q = pose.orientation
        data = {"pose": {
            "translation": {"x": float(pose.position.x),
                            "y": float(pose.position.y),
                            "z": float(pose.position.z)},
            "rotation":    {"x": float(q.x), "y": float(q.y),
                            "z": float(q.z), "w": float(q.w)},
        }}
        with open(self.yaml_path, "w") as f:
            yaml.dump(data, f, default_flow_style=False)
        self._set_status("Saved to YAML", "green")

    def _reset(self):
        try:
            init = self._load_yaml()
        except Exception as e:
            messagebox.showerror("Load error", str(e))
            return
        tx, ty, tz = init["t"]
        qx, qy, qz, qw = init["q"]
        rpy = spt.Rotation.from_quat([qx, qy, qz, qw]).as_euler("xyz", degrees=True)
        self._updating = True
        try:
            for axis, val in zip(["x", "y", "z"], [tx, ty, tz]):
                self.t_sl[axis].set(val);  self.t_txt[axis].set(f"{val:.5f}")
            for axis, val in zip(["qx", "qy", "qz", "qw"], [qx, qy, qz, qw]):
                self.q_sl[axis].set(val);  self.q_txt[axis].set(f"{val:.8f}")
            for axis, val in zip(["roll", "pitch", "yaw"], rpy):
                self.rpy_sl[axis].set(val); self.rpy_txt[axis].set(f"{val:.4f}")
        finally:
            self._updating = False
        self.dirty.set()
        self._set_status("Reset from YAML", "blue")

    def _set_status(self, msg, color="green"):
        self.root.after(0, lambda: self.status.config(text=msg, foreground=color))


# ---------------------------------------------------------------------------
# Publisher thread
# ---------------------------------------------------------------------------

def publisher_thread(gui, tf_buffer, rate_hz=10):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if gui.dirty.is_set():
            gui.dirty.clear()
            pose = gui.get_pose()
            if pose is not None:
                corrected = compute_zed_base_pose(pose, tf_buffer)
                if corrected is not None:
                    broadcast_tf(corrected, broadcaster)
                    gui._set_status("TF published", "green")
                else:
                    broadcast_tf(pose, broadcaster)
                    gui._set_status("TF published (no optical→base chain found)", "orange")
            else:
                gui._set_status("Invalid quaternion – not publishing", "red")
        rate.sleep()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node("zed_tf_calibration_gui", anonymous=False)

    rospack = rospkg.RosPack()
    yaml_path = rospack.get_path("manipulation") + "/config/zed_to_label_tf.yaml"

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    root = tk.Tk()
    gui = ZedCalibrationGUI(root, yaml_path, tf_buffer)

    pub_thread = threading.Thread(target=publisher_thread,
                                  args=(gui, tf_buffer), daemon=True)
    pub_thread.start()

    root.mainloop()
