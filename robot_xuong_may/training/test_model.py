#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
THỬ / ĐÁNH GIÁ MÔ HÌNH bằng ĐÚNG CODE NHẬN DIỆN CỦA ROBOT (best.onnx hoặc best.pt)

  python training/test_model.py                                    # camera của robot, hiện FPS
  python training/test_model.py --source video_nhom_gui.mp4        # video
  python training/test_model.py --source anh_moi/                  # thư mục ảnh (xem từng ảnh)
  python training/test_model.py --eval dataset --split test        # ĐÁNH GIÁ có nhãn: đúng / nhầm / bỏ sót
                                                                   # theo từng trạm + logs/eval_*.csv

Đánh giá (IoU ≥ 0.5): mỗi nhãn thật được ghép với 1 khung dự đoán cùng vị trí.
  ĐÚNG      : cùng vị trí, cùng lớp
  NHẦM TRẠM : cùng vị trí nhưng khác lớp (nguy hiểm nhất: robot dừng sai trạm)
  BỎ SÓT    : có trạm nhưng không nhận ra
  BÁO GIẢ   : nhận ra trạm ở chỗ không có
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from collections import Counter
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402
from robot.common import focal_norm  # noqa: E402
from robot.detector import check_classes, create_detector  # noqa: E402

IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp"}


def draw(frame, dets, fps, ms):
    import cv2
    h, w = frame.shape[:2]
    fn = focal_norm(cfg.CAMERA_VFOV_DEG)
    for d in dets:
        x1, y1, x2, y2 = int(d.x1 * w), int(d.y1 * h), int(d.x2 * w), int(d.y2 * h)
        col = (0, 0, 255) if d.cls in cfg.OBSTACLE_CLASSES else (0, 220, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
        txt = f"{d.cls} {d.conf:.2f}"
        station = next((s for s in cfg.STATIONS.values() if s["class"] == d.cls), None)
        if station:
            txt += f" ~{fn * station['marker_h_m'] / max(d.h, 1e-3):.2f}m"
        cv2.putText(frame, txt, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    cv2.putText(frame, f"{fps:.1f} FPS  {ms:.0f} ms", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return frame


def iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def read_labels(path: Path, names: dict):
    out = []
    if path.exists():
        for line in path.read_text().splitlines():
            f = line.split()
            if len(f) == 5:
                c, cx, cy, w, h = int(f[0]), *map(float, f[1:])
                out.append((names.get(c, str(c)), (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)))
    return out


def evaluate(det, root: Path, split: str, iou_thr: float = 0.5):
    import cv2
    img_dir = root / "images" / split if (root / "images" / split).exists() else root
    lab_dir = root / "labels" / split if (root / "labels" / split).exists() else root
    files = sorted(p for p in img_dir.rglob("*") if p.suffix.lower() in IMG_EXT)
    if not files:
        raise SystemExit(f"Không có ảnh trong {img_dir}")
    names = {i: n for i, n in enumerate(cfg.MODEL_CLASSES)} if getattr(cfg, "MODEL_CLASSES", None) else det.names
    stat = {n: Counter() for n in names.values()}
    rows, times = [], []
    for p in files:
        frame = cv2.imread(str(p))
        if frame is None:
            continue
        dets = det.detect(frame)
        times.append(det.last_ms)
        gts = read_labels(lab_dir / (p.stem + ".txt"), names)
        used = set()
        for gcls, gbox in gts:
            best, bi = 0.0, -1
            for i, d in enumerate(dets):
                if i in used:
                    continue
                v = iou(gbox, (d.x1, d.y1, d.x2, d.y2))
                if v > best:
                    best, bi = v, i
            st = stat.setdefault(gcls, Counter())
            if bi >= 0 and best >= iou_thr:
                used.add(bi)
                if dets[bi].cls == gcls:
                    st["dung"] += 1
                else:
                    st["nham"] += 1
                    rows.append((p.name, "NHẦM TRẠM", gcls, dets[bi].cls, f"{dets[bi].conf:.2f}"))
            else:
                st["bo_sot"] += 1
                rows.append((p.name, "BỎ SÓT", gcls, "", ""))
        for i, d in enumerate(dets):
            if i not in used:
                stat.setdefault(d.cls, Counter())["bao_gia"] += 1
                rows.append((p.name, "BÁO GIẢ", "", d.cls, f"{d.conf:.2f}"))
    print(f"\nĐánh giá {len(files)} ảnh ({img_dir}) – mô hình {getattr(det, 'version', '?')}, "
          f"trung bình {sum(times) / max(1, len(times)):.0f} ms/ảnh trên máy này")
    print(f"{'lớp':8s} {'đúng':>6s} {'nhầm':>6s} {'bỏ sót':>7s} {'báo giả':>8s} {'recall':>7s} {'precision':>9s}")
    for n, st in stat.items():
        gt = st["dung"] + st["nham"] + st["bo_sot"]
        pr = st["dung"] + sum(1 for r in rows if r[1] in ("NHẦM TRẠM", "BÁO GIẢ") and r[3] == n)
        rec = st["dung"] / gt if gt else float("nan")
        prec = st["dung"] / pr if pr else float("nan")
        print(f"{n:8s} {st['dung']:6d} {st['nham']:6d} {st['bo_sot']:7d} {st['bao_gia']:8d} {rec:7.2f} {prec:9.2f}")
    out = PROJECT_DIR / cfg.LOG_DIR / time.strftime(f"eval_{split}_%Y%m%d_%H%M%S.csv")
    out.parent.mkdir(exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["anh", "loi", "lop_that", "lop_du_doan", "do_tin_cay"])
        w.writerows(rows)
    print(f"Danh sách lỗi ({len(rows)} dòng): {out}")
    return stat, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=cfg.MODEL_PATH)
    ap.add_argument("--source", default="camera", help="camera | thư mục ảnh | file video")
    ap.add_argument("--eval", default=None, help="thư mục dataset (images/<split>, labels/<split>)")
    ap.add_argument("--split", default="test")
    ap.add_argument("--imgsz", type=int, default=cfg.YOLO_IMGSZ)
    ap.add_argument("--conf", type=float, default=cfg.YOLO_CONF)
    args = ap.parse_args()

    import cv2
    det = create_detector(args.model, args.imgsz, args.conf, cfg.YOLO_IOU)
    errors, notes = check_classes(det.names, cfg)
    for e in errors:
        print("LỖI LỚP:", e)
    for n in notes:
        print("Lưu ý:", n)
    if args.eval:
        evaluate(det, Path(args.eval), args.split)
        return
    gui = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or os.name == "nt")
    src = Path(args.source)
    if args.source != "camera" and src.is_dir():
        files = sorted(p for p in src.iterdir() if p.suffix.lower() in IMG_EXT)
        for p in files:
            frame = cv2.imread(str(p))
            dets = det.detect(frame)
            print(p.name, [(d.cls, round(d.conf, 2)) for d in dets])
            if gui:
                cv2.imshow("test", draw(frame, dets, 0, det.last_ms))
                if cv2.waitKey(0) & 0xFF == 27:
                    break
        return
    from robot.camera import Camera
    cam = Camera(cfg.CAMERA_BACKEND, cfg.CAMERA_INDEX, cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT, cfg.CAMERA_FLIP,
                 video_path=None if args.source == "camera" else args.source).start()
    fps, last = 0.0, time.monotonic()
    try:
        while True:
            frame, _ = cam.read()
            if frame is None:
                time.sleep(0.05)
                continue
            dets = det.detect(frame)
            now = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(1e-3, now - last) if fps else 1.0 / max(1e-3, now - last)
            last = now
            if gui:
                cv2.imshow("test", draw(frame.copy(), dets, fps, det.last_ms))
                if cv2.waitKey(1) & 0xFF == 27:
                    break
            else:
                print(f"{fps:5.1f} FPS {det.last_ms:5.0f} ms", [(d.cls, round(d.conf, 2)) for d in dets])
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()


if __name__ == "__main__":
    main()
