# -*- coding: utf-8 -*-
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
