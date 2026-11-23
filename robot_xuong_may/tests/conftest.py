# -*- coding: utf-8 -*-
import os
import subprocess
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot import config as _config  # noqa: E402


def make_cfg(**overrides):
    """Bản sao cấu hình (không đụng vào robot/config.py) + ghi đè cho từng test."""
    d = {k: getattr(_config, k) for k in dir(_config) if k.isupper()}
    d["STATIONS"] = {k: dict(v) for k, v in _config.STATIONS.items()}
    d.update(overrides)
    return types.SimpleNamespace(**d)


@pytest.fixture
def cfg():
    return make_cfg()


def run_script(args, cwd=None, timeout=300):
    """Chạy 1 script Python như trên Windows khi kết quả bị thu lại (bảng mã cp1252) –
    script phải tự in được tiếng Việt; kết quả đọc bằng UTF-8."""
    env = dict(os.environ, PYTHONIOENCODING="cp1252")
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, encoding="utf-8",
                          errors="replace", cwd=cwd or ROOT, env=env, timeout=timeout)
