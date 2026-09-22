"""Pure helpers of recorder.py. Run: <perception venv>/bin/python -m pytest src/devel_packages/orio_logging/test"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import recorder  # noqa: E402


def test_pose_from_colmajor_matches_franka_layout():
    c, s = np.cos(0.3), np.sin(0.3)
    # libfranka O_T_EE: column-major 4x4, translation in elements 12..14
    m = [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0.5, -0.2, 0.3, 1]
    t, R = recorder.pose_from_colmajor(m)
    assert np.allclose(t, [0.5, -0.2, 0.3])
    assert np.allclose(R[:, 0], [c, s, 0]) and np.allclose(R @ R.T, np.eye(3))


def test_default_run_dir_prefers_env(monkeypatch, tmp_path):
    monkeypatch.setenv('ORIO_RUN_DIR', str(tmp_path / 'r1'))
    assert recorder.default_run_dir() == tmp_path / 'r1'
    monkeypatch.delenv('ORIO_RUN_DIR')
    monkeypatch.setenv('ORIO_RUN_ID', 'abc')
    assert recorder.default_run_dir().name == 'abc'


def test_check_disk_refuses_when_low(tmp_path, capsys):
    (tmp_path / 'old_run').mkdir()
    try:
        recorder.check_disk(tmp_path, min_free_gb=1e9)
    except SystemExit as e:
        assert e.code == 3
    assert 'old_run' in capsys.readouterr().err
    assert recorder.check_disk(tmp_path, min_free_gb=0) > 0
