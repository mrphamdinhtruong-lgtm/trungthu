#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
KIỂM TRA BỘ DỮ LIỆU NHÓM TRAIN BÀN GIAO (việc của NGƯỜI VIẾT CODE – chỉ đọc, không sửa, không chia lại)

  python training/check_dataset.py nhan/dataset            # thư mục đã giải nén
  python training/check_dataset.py nhan/dataset_cloud.zip  # file zip tải từ cloud (Roboflow / Ultralytics...)

Đọc được 2 kiểu thư mục:
  * Ultralytics : images/{train,val,test}/  + labels/{train,val,test}/  + data.yaml
  * Roboflow    : {train,valid,test}/images/ + {train,valid,test}/labels/ + data.yaml

Kiểm tra (đúng bước 5 của kế hoạch: "tên/thứ tự lớp, định dạng file và dữ liệu đầu vào code cần nhận"):
  [LỖI]  data.yaml thiếu / tên-thứ tự lớp khác robot/config.py MODEL_CLASSES
  [LỖI]  dòng nhãn sai: không đủ 5 số, mã lớp ngoài 0..nc-1, tọa độ ngoài 0..1, khung rỗng
  [LỖI]  thiếu tập train hoặc val; tập nào đó KHÔNG có đối tượng của 1 lớp
  [LỖI]  ảnh không đọc được
  [CẢNH BÁO] ảnh val/test gần trùng ảnh train (dHash), cùng một buổi chụp nằm ở nhiều tập,
             ảnh không phải tỉ lệ 4:3 (camera robot là 640x480), ảnh nhỏ hơn 416 px, nhãn không có ảnh
Kết quả: bảng tóm tắt + logs/check_dataset_*.txt (gửi lại nhóm train). Mã thoát 0 = ĐẠT, 1 = KHÔNG ĐẠT.
"""
from __future__ import annotations

import argparse
import ast
import re
import sys
import tempfile
import time
import zipfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

PROJECT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_DIR))

from robot import config as cfg  # noqa: E402

IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp"}
SPLIT_ALIASES = {"train": "train", "val": "val", "valid": "val", "validation": "val", "test": "test"}


# ---------------------------------------------------------------------------
def read_yaml_names(path: Path) -> Tuple[Optional[List[str]], Optional[int]]:
    """Đọc names / nc trong data.yaml (dạng danh sách hoặc từ điển) – không cần thư viện yaml."""
    text = path.read_text(encoding="utf-8", errors="ignore")
    try:
        import yaml
        d = yaml.safe_load(text) or {}
        names, nc = d.get("names"), d.get("nc")
    except Exception:
        names, nc = None, None
        m = re.search(r"^names:\s*(\[.*?\])", text, re.M | re.S)
        if m:
            names = ast.literal_eval(m.group(1))
        else:
            block = re.search(r"^names:\s*\n((?:[ \t]+\S.*\n?)+)", text, re.M)
            if block:
                names = {}
                for line in block.group(1).splitlines():
                    kv = re.match(r"\s*(\d+)\s*:\s*['\"]?([^'\"#]+?)['\"]?\s*(#.*)?$", line)
                    if kv:
                        names[int(kv.group(1))] = kv.group(2).strip()
        m = re.search(r"^nc:\s*(\d+)", text, re.M)
        nc = int(m.group(1)) if m else None
    if isinstance(names, dict):
        names = [str(names[k]) for k in sorted(names, key=int)]
    elif isinstance(names, list):
        names = [str(n) for n in names]
    return names, (int(nc) if nc is not None else None)


def find_splits(root: Path) -> Dict[str, Tuple[Path, Path]]:
    """-> {"train": (thư_mục_ảnh, thư_mục_nhãn), ...}"""
    out = {}
    for name, split in SPLIT_ALIASES.items():
        a = (root / "images" / name, root / "labels" / name)          # kiểu Ultralytics
        b = (root / name / "images", root / name / "labels")          # kiểu Roboflow
        for img_dir, lab_dir in (a, b):
            if img_dir.is_dir() and split not in out:
                out[split] = (img_dir, lab_dir)
    return out


def session_key(name: str) -> str:
    """Đoán "buổi chụp" từ tên file: phần trước '__' (split_dataset) hoặc trước '_' đầu tiên."""
    base = name.split(".rf.")[0]                                       # Roboflow thêm .rf.<mã>
    if "__" in base:
        return base.split("__")[0]
    return re.split(r"[_\-]", base, maxsplit=1)[0]


def check_label_file(path: Path, nc: int) -> Tuple[Counter, List[str]]:
    counts, bad = Counter(), []
    for i, line in enumerate(path.read_text(errors="ignore").splitlines(), 1):
        if not line.strip():
            continue
        f = line.split()
        try:
            if len(f) != 5:
                raise ValueError("cần 5 số")
            c = int(float(f[0]))
            if float(f[0]) != c:
                raise ValueError("mã lớp không nguyên")
            cx, cy, w, h = map(float, f[1:])
            if not 0 <= c < nc:
                raise ValueError(f"mã lớp {c} ngoài 0..{nc - 1}")
            if not all(0.0 <= v <= 1.0 for v in (cx, cy, w, h)):
                raise ValueError("tọa độ ngoài 0..1")
            if w <= 0 or h <= 0:
                raise ValueError("khung rỗng")
            counts[c] += 1
        except ValueError as exc:
            bad.append(f"{path.name}:{i}: '{line.strip()}' ({exc})")
    return counts, bad


def check(root: Path, expected: List[str], dup_dist: int = 5) -> Tuple[bool, List[str]]:
    import cv2
    from training.split_dataset import dhash
    rep: List[str] = []
    errors = 0

    def err(msg):
        nonlocal errors
        errors += 1
        rep.append("[LỖI] " + msg)

    def warn(msg):
        rep.append("[CẢNH BÁO] " + msg)

    # ---- data.yaml ----
    yamls = sorted(root.rglob("data.yaml"), key=lambda p: len(p.parts))
    names, nc = (None, None)
    if not yamls:
        err("không có data.yaml")
    else:
        names, nc = read_yaml_names(yamls[0])
        rep.append(f"data.yaml: {yamls[0].relative_to(root)} -> names = {names}, nc = {nc}")
        if names is None:
            err("data.yaml không có 'names'")
        elif expected and names != expected:
            err(f"tên/thứ tự lớp {names} KHÁC MODEL_CLASSES {expected} (yêu cầu 0=tram_1, 1=tram_2, 2=tram_3)")
        if names and nc is not None and nc != len(names):
            err(f"nc = {nc} nhưng names có {len(names)} lớp")
    nclass = len(expected or names or [])

    # ---- các tập ----
    splits = find_splits(root)
    rep.append("Các tập tìm thấy: " + (", ".join(f"{s} ({p[0].relative_to(root)})" for s, p in splits.items()) or "KHÔNG CÓ"))
    for need in ("train", "val"):
        if need not in splits:
            err(f"thiếu tập {need}")
    if "test" not in splits:
        warn("không có tập test giữ riêng (kế hoạch yêu cầu train/val/test)")

    table, hashes, sessions = [], {}, defaultdict(set)
    bad_all: List[str] = []
    ratio_off, small, unread = 0, 0, []
    for split, (img_dir, lab_dir) in splits.items():
        imgs = sorted(p for p in img_dir.rglob("*") if p.suffix.lower() in IMG_EXT)
        labs = {p.stem: p for p in lab_dir.rglob("*.txt")} if lab_dir.is_dir() else {}
        counts, nolab = Counter(), 0
        hashes[split] = []
        for p in imgs:
            lab = labs.pop(p.stem, None)
            if lab is None:
                nolab += 1                                      # không có file nhãn = ảnh nền
            else:
                c, bad = check_label_file(lab, nclass or 999)
                counts.update(c)
                bad_all += bad
                if not c and not bad:
                    nolab += 1                                  # file nhãn rỗng = ảnh nền
            im = cv2.imread(str(p))
            if im is None:
                unread.append(f"{split}/{p.name}")
                continue
            h, w = im.shape[:2]
            if abs(w / h - 4 / 3) > 0.05:
                ratio_off += 1
            if min(w, h) < 416:
                small += 1
            hashes[split].append((p.name, dhash(p)))
            sessions[session_key(p.name)].add(split)
        if labs:
            warn(f"{split}: {len(labs)} file nhãn không có ảnh tương ứng (vd {next(iter(labs.values())).name})")
        table.append((split, len(imgs), nolab, counts))
        for i in range(nclass):
            if imgs and counts[i] == 0:
                err(f"tập {split} KHÔNG có đối tượng lớp {i} ({(expected or names or ['?'] * nclass)[i]})")

    # ---- bảng tóm tắt ----
    cls_names = expected or names or [str(i) for i in range(nclass)]
    rep.append("")
    rep.append(f"{'tập':6s} {'ảnh':>6s} {'ảnh nền':>8s}  " + "  ".join(f"{n:>8s}" for n in cls_names))
    for split, n_img, nolab, counts in table:
        rep.append(f"{split:6s} {n_img:6d} {nolab:8d}  " + "  ".join(f"{counts[i]:8d}" for i in range(nclass)))
    rep.append("")

    if bad_all:
        err(f"{len(bad_all)} dòng nhãn sai định dạng, ví dụ:")
        rep += ["       " + b for b in bad_all[:15]]
    if unread:
        err(f"{len(unread)} ảnh không đọc được: {unread[:5]}")
    total = sum(len(v) for v in hashes.values())
    if total and ratio_off / total > 0.3:
        warn(f"{ratio_off}/{total} ảnh không phải tỉ lệ 4:3 (camera robot 640x480) – mô hình có thể kém khi chạy thật")
    if small:
        warn(f"{small} ảnh có cạnh < 416 px (nhỏ hơn kích thước train)")

    # ---- rò rỉ giữa các tập ----
    leak = {s: sp for s, sp in sessions.items() if len(sp) > 1}
    if leak and len(sessions) > 1:
        ex = ", ".join(f"{s}->{sorted(v)}" for s, v in list(leak.items())[:5])
        warn(f"{len(leak)} buổi chụp (đoán theo tên file) nằm ở nhiều tập: {ex} – kế hoạch yêu cầu chia THEO BUỔI")
    dups = []
    for s in ("val", "test"):
        for name, h in hashes.get(s, []):
            if h is None:
                continue
            for tname, g in hashes.get("train", []):
                if g is not None and bin(h ^ g).count("1") <= dup_dist:
                    dups.append(f"{s}/{name} ~ train/{tname}")
                    break
    if dups:
        warn(f"{len(dups)} ảnh val/test GẦN TRÙNG ảnh train, ví dụ: {dups[:5]}")

    ok = errors == 0
    rep.append("KẾT LUẬN: " + ("ĐẠT – dùng được để train / đánh giá" if ok else f"KHÔNG ĐẠT – {errors} lỗi, gửi lại nhóm train sửa"))
    return ok, rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="thư mục dataset hoặc file .zip nhóm train gửi")
    ap.add_argument("--classes", default=",".join(getattr(cfg, "MODEL_CLASSES", ())),
                    help="tên lớp mong đợi theo thứ tự (mặc định lấy từ robot/config.py)")
    args = ap.parse_args()
    expected = [c for c in args.classes.split(",") if c]
    p = Path(args.path)
    tmp = None
    if p.suffix.lower() == ".zip":
        tmp = tempfile.TemporaryDirectory()
        with zipfile.ZipFile(p) as z:
            for m in z.infolist():                          # chống đường dẫn "../" trong zip
                target = (Path(tmp.name) / m.filename).resolve()
                if Path(tmp.name).resolve() in target.parents or target == Path(tmp.name).resolve():
                    z.extract(m, tmp.name)
        root = Path(tmp.name)
        subs = [d for d in root.iterdir() if d.is_dir()]
        if not find_splits(root) and len(subs) == 1:
            root = subs[0]
    else:
        root = p
    ok, rep = check(root, expected)
    text = "\n".join(rep)
    print(text)
    out = PROJECT_DIR / cfg.LOG_DIR / time.strftime("check_dataset_%Y%m%d_%H%M%S.txt")
    out.parent.mkdir(exist_ok=True)
    out.write_text(f"Kiểm tra: {p}\n{text}\n", encoding="utf-8")
    print(f"Báo cáo: {out}")
    if tmp:
        tmp.cleanup()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
