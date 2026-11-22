#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CHIA DỮ LIỆU ĐÃ GÁN NHÃN thành train / val theo định dạng YOLO.

Đầu vào: một thư mục chứa ảnh (.jpg/.png) và file nhãn YOLO cùng tên (.txt)
         – ví dụ xuất từ LabelImg / CVAT / Label Studio (định dạng "YOLO").
  python training/split_dataset.py --src dataset/labeled --val 0.2

Đầu ra: dataset/images/{train,val}, dataset/labels/{train,val}
Ảnh không có file nhãn được coi là ẢNH NỀN (không có đối tượng) – vẫn hữu ích.
"""
from __future__ import annotations

import argparse
import random
import shutil
from collections import Counter
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="thư mục ảnh + nhãn .txt")
    ap.add_argument("--dst", default=str(PROJECT_DIR / "dataset"))
    ap.add_argument("--val", type=float, default=0.2, help="tỉ lệ ảnh kiểm định")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    src, dst = Path(args.src), Path(args.dst)
    images = sorted(p for p in src.rglob("*") if p.suffix.lower() in IMG_EXT)
    if not images:
        raise SystemExit(f"Không có ảnh trong {src}")
    random.Random(args.seed).shuffle(images)
    n_val = max(1, int(len(images) * args.val))
    counts = Counter()
    for split, items in (("val", images[:n_val]), ("train", images[n_val:])):
        (dst / "images" / split).mkdir(parents=True, exist_ok=True)
        (dst / "labels" / split).mkdir(parents=True, exist_ok=True)
        for img in items:
            shutil.copy2(img, dst / "images" / split / img.name)
            lab = img.with_suffix(".txt")
            target = dst / "labels" / split / (img.stem + ".txt")
            if lab.exists():
                shutil.copy2(lab, target)
                for line in lab.read_text().splitlines():
                    if line.strip():
                        counts[int(line.split()[0])] += 1
            else:
                target.write_text("")
    print(f"train: {len(images) - n_val} ảnh, val: {n_val} ảnh")
    print("Số đối tượng theo lớp:", dict(sorted(counts.items())))


if __name__ == "__main__":
    main()
