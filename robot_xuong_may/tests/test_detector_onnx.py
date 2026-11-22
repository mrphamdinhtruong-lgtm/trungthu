# -*- coding: utf-8 -*-
"""Bộ nhận diện ONNX: dựng một mô hình ONNX giả có đầu ra YOLO11 (1, 4+nc, N) đã biết trước
để kiểm tra letterbox, giải mã, NMS, đọc metadata và kiểm tra lớp – không cần ultralytics."""
import json

import numpy as np
import pytest

onnx = pytest.importorskip("onnx")
pytest.importorskip("onnxruntime")
from onnx import TensorProto, helper, numpy_helper  # noqa: E402

from robot.detector import OnnxDetector, check_classes, decode_yolo, letterbox, load_metadata  # noqa: E402

from conftest import make_cfg  # noqa: E402

N = 52 * 52 + 26 * 26 + 13 * 13      # 3549 ô dự đoán ở ảnh 416


def fake_output(nc=3):
    out = np.zeros((1, 4 + nc, N), np.float32)
    def put(k, cx, cy, w, h, cls, conf):
        out[0, :4, k] = (cx, cy, w, h)
        out[0, 4 + cls, k] = conf
    put(10, 208, 208, 65, 65, 1, 0.90)        # tram_2 giữa ảnh, 100x100 px ảnh gốc
    put(11, 210, 209, 66, 64, 1, 0.80)        # trùng -> NMS phải bỏ
    put(12, 100, 150, 39, 39, 2, 0.70)        # tram_3 bên trái
    put(13, 300, 300, 20, 20, 0, 0.20)        # dưới ngưỡng tin cậy
    return out


def build_model(path, nc=3, meta=None):
    const = fake_output(nc)
    inp = helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 416, 416])
    out = helper.make_tensor_value_info("output0", TensorProto.FLOAT, list(const.shape))
    nodes = [
        helper.make_node("ReduceMean", ["images"], ["m"], axes=[1, 2, 3], keepdims=0),
        helper.make_node("Mul", ["m", "zero"], ["z"]),
        helper.make_node("Add", ["z", "pred"], ["output0"]),
    ]
    inits = [numpy_helper.from_array(np.zeros(1, np.float32), "zero"), numpy_helper.from_array(const, "pred")]
    g = helper.make_graph(nodes, "fake_yolo", [inp], [out], inits)
    m = helper.make_model(g, opset_imports=[helper.make_opsetid("", 12)])
    m.ir_version = 7
    if meta:
        for k, v in meta.items():
            e = m.metadata_props.add()
            e.key, e.value = k, v
    onnx.save(m, str(path))


@pytest.fixture
def model(tmp_path):
    p = tmp_path / "best.onnx"
    build_model(p)
    (tmp_path / "best.metadata.json").write_text(json.dumps({
        "model_version": "tram3_v1", "names": {"0": "tram_1", "1": "tram_2", "2": "tram_3"},
        "nc": 3, "imgsz": [416, 416]}), encoding="utf-8")
    return p


def test_letterbox_geometry():
    img, r, pad = letterbox(np.zeros((480, 640, 3), np.uint8), (416, 416))
    assert img.shape == (416, 416, 3) and r == pytest.approx(0.65) and pad == (0, 52)


@pytest.mark.parametrize("backend", ["onnxruntime", "opencv"])
def test_decode_boxes_nms_and_names(model, backend):
    det = OnnxDetector(str(model), 416, conf=0.45, iou=0.5, backend=backend)
    assert det.names == {0: "tram_1", 1: "tram_2", 2: "tram_3"} and det.version == "tram3_v1"
    out = sorted(det.detect(np.zeros((480, 640, 3), np.uint8)), key=lambda d: -d.conf)
    assert [d.cls for d in out] == ["tram_2", "tram_3"]          # trùng bị NMS bỏ, dưới ngưỡng bị bỏ
    a = out[0]
    assert a.conf == pytest.approx(0.90, abs=1e-6)
    # 208/0.65 = 320, (208-52)/0.65 = 240, cạnh 65/0.65 = 100 px
    assert (a.x1, a.y1, a.x2, a.y2) == pytest.approx((270 / 640, 190 / 480, 370 / 640, 290 / 480), abs=1e-4)


def test_metadata_fallback_from_onnx_props(tmp_path):
    p = tmp_path / "x.onnx"
    build_model(p, meta={"names": "{0: 'tram_1', 1: 'tram_2', 2: 'tram_3'}", "imgsz": "[416, 416]"})
    meta = load_metadata(p)
    assert meta["names"] == {0: "tram_1", 1: "tram_2", 2: "tram_3"} and meta["imgsz"] == [416, 416]


def test_missing_names_is_an_error(tmp_path):
    p = tmp_path / "y.onnx"
    build_model(p)
    with pytest.raises(RuntimeError):
        OnnxDetector(str(p))


def test_check_classes():
    cfg = make_cfg()
    ok, notes = check_classes({0: "tram_1", 1: "tram_2", 2: "tram_3"}, cfg)
    assert ok == [] and any("vật cản" in n for n in notes)
    bad, _ = check_classes({0: "tram_2", 1: "tram_1", 2: "tram_3"}, cfg)
    assert any("Thứ tự" in e for e in bad)
    miss, _ = check_classes({0: "tram_1", 1: "tram_2"}, cfg)
    assert any("tram_3" in e for e in miss)


def test_decode_rejects_wrong_class_count():
    with pytest.raises(ValueError):
        decode_yolo(fake_output(3), 5, 0.4, 0.5, 1.0, (0, 0), (416, 416))
