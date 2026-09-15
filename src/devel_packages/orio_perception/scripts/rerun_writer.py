"""Perception-side rerun writer: <run_dir>/perception.rrd, all encoding on a worker thread.

Service handlers only enqueue (bounded queue, drop-on-full); nothing here can block them.
Disabled when ORIO_LOGGING=0 or no run dir is given.
"""
import os
import queue
import threading
import time

import numpy as np

ARM1_FRAME = 'arm1/panda_link0'


class PerceptionLog:
    def __init__(self, run_dir=None, run_id=None, enabled=None, full_frames=None, jpeg_quality=80, maxsize=32):
        run_dir = run_dir or os.environ.get('ORIO_RUN_DIR')
        if enabled is None:
            enabled = bool(run_dir) and os.environ.get('ORIO_LOGGING', '1') != '0'
        self.enabled = enabled
        self.full_frames = os.environ.get('ORIO_LOG_FULL_FRAMES') == '1' if full_frames is None else full_frames
        self.jpeg_quality = jpeg_quality
        self.dropped = 0
        self.calls = {'pnp': 0, 'lbl': 0}
        self.path = None
        if not self.enabled:
            return
        import rerun as rr
        self.rr = rr
        os.makedirs(run_dir, exist_ok=True)
        self.path = os.path.join(run_dir, 'perception.rrd')
        rr.init('orio', recording_id=run_id or os.environ.get('ORIO_RUN_ID') or os.path.basename(run_dir.rstrip('/')))
        rr.save(self.path)
        self._q = queue.Queue(maxsize=maxsize)
        self._thread = threading.Thread(target=self._worker, name='rerun-writer', daemon=True)
        self._thread.start()

    # ── public API (cheap: copies references, returns immediately) ────────
    def pnp(self, rgb, depth, depth_scale, boxes=None, phrases=None, logits=None, masks=None,
            grasps=(), backend='classical', plan_time=0.0, reason=''):
        """One /compute_grasps call. grasps: dicts with center, normal (optional), score, tilt_deg."""
        self.calls['pnp'] += 1
        n, t = self.calls['pnp'], time.time()
        args = (t, n, rgb, depth, depth_scale, boxes, phrases, logits, masks, list(grasps), backend, plan_time, reason)
        self._put(lambda: self._log_pnp(*args))

    def lbl(self, rgb, masks=None, boxes=None, phrases=None, logits=None, zones=None, ok=False, reason=''):
        """One /compute_grasps_labelling call. zones: {1: {x, y, angle_deg, depth} | None, 2: ...}."""
        self.calls['lbl'] += 1
        n, t = self.calls['lbl'], time.time()
        args = (t, n, rgb, masks, boxes, phrases, logits, zones or {}, ok, reason)
        self._put(lambda: self._log_lbl(*args))

    def close(self):
        if not self.enabled:
            return
        self._q.put(None)
        self._thread.join(timeout=5.0)
        self.rr.disconnect()

    # ── worker side ───────────────────────────────────────────────────────
    def _put(self, fn):
        if not self.enabled:
            return
        try:
            self._q.put_nowait(fn)
        except queue.Full:
            self.dropped += 1

    def _worker(self):
        while True:
            fn = self._q.get()
            if fn is None:
                return
            try:
                fn()
            except Exception as exc:  # never let a logging bug reach the node
                self.dropped += 1
                print(f'[rerun_writer] dropped item: {exc!r}')

    def _image(self, path, rgb, boxes, phrases, logits, masks):
        rr = self.rr
        rgb = np.ascontiguousarray(rgb[:, :, :3])
        rr.log(path, rr.Image(rgb, color_model='RGB').compress(jpeg_quality=self.jpeg_quality))
        if boxes is not None and len(boxes):
            labels = [f'{p} {float(l):.2f}' for p, l in zip(phrases or [''] * len(boxes), logits or [0] * len(boxes))]
            rr.log(f'{path}/boxes', rr.Boxes2D(array=np.asarray(boxes, dtype=np.float32),
                                                array_format=rr.Box2DFormat.XYXY, labels=labels))
        if masks:
            label_map = np.zeros(rgb.shape[:2], dtype=np.uint8)
            for i, m in enumerate(masks):
                label_map[np.asarray(m, dtype=bool)] = i + 1
            rr.log(f'{path}/masks', rr.SegmentationImage(label_map))

    def _log_pnp(self, t, n, rgb, depth, depth_scale, boxes, phrases, logits, masks, grasps, backend, plan_time, reason):
        rr = self.rr
        rr.set_time('ros_time', timestamp=t)
        rr.set_time('pick', sequence=n)
        self._image('perception/pnp/image', rgb, boxes, phrases, logits, masks)
        rr.log('perception/pnp/depth', rr.DepthImage(np.ascontiguousarray(depth), meter=float(depth_scale)))
        rr.log('perception/pnp/plan_time', rr.Scalars(float(plan_time)))
        rr.log('perception/pnp/n_grasps', rr.Scalars(float(len(grasps))))
        if grasps:
            centers = np.asarray([g['center'] for g in grasps], dtype=np.float64)
            normals = np.asarray([g.get('normal', [0, 0, 1]) for g in grasps], dtype=np.float64)
            scores = [float(g.get('score', np.nan)) for g in grasps]
            rr.log('perception/pnp/grasp/points', rr.Points3D(centers, radii=0.01, labels=[f'q={s:.3g}' for s in scores]),
                   rr.CoordinateFrame(frame=ARM1_FRAME))
            rr.log('perception/pnp/grasp/normals', rr.Arrows3D(origins=centers, vectors=normals * 0.05),
                   rr.CoordinateFrame(frame=ARM1_FRAME))
            rr.log('perception/pnp/q_value', rr.Scalars(scores[0]))
            rr.log('perception/pnp/tilt_deg', rr.Scalars(float(grasps[0].get('tilt_deg', np.nan))))
            rr.log('perception/pnp/center', rr.Scalars(centers[0]))
        rr.log('perception/pnp/result', rr.TextLog(f'{backend}: {len(grasps)} grasp(s) {reason}'.strip(),
                                                   level='INFO' if grasps else 'WARN'))

    def _log_lbl(self, t, n, rgb, masks, boxes, phrases, logits, zones, ok, reason):
        rr = self.rr
        rr.set_time('ros_time', timestamp=t)
        rr.set_time('label', sequence=n)
        self._image('perception/lbl/image', rgb, boxes, phrases, logits, masks)
        pts, labels = [], []
        for z, v in zones.items():
            if v:
                pts.append([v['x'], v['y']])
                labels.append(f'Z{z} {v.get("angle_deg", 0):.0f}deg')
                for k in ('x', 'y', 'angle_deg', 'depth'):
                    if k in v:
                        rr.log(f'perception/lbl/z{z}/{k}', rr.Scalars(float(v[k])))
        if pts:
            rr.log('perception/lbl/image/targets', rr.Points2D(pts, radii=6, labels=labels))
        rr.log('perception/lbl/result', rr.TextLog(reason, level='INFO' if ok else 'WARN'))
