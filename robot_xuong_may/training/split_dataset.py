#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GỘP & CHIA DỮ LIỆU ĐÃ GÁN NHÃN thành train / val / test THEO BUỔI CHỤP (định dạng YOLO).

Vì sao chia theo buổi? Ảnh cùng một buổi (cùng chỗ, cùng ánh sáng, chụp liên tiếp) gần như trùng nhau.
Nếu ảnh gần trùng nằm ở cả tập học và tập kiểm tra, mAP sẽ đẹp giả tạo nhưng robot chạy thật lại sai.

Đầu vào: thư mục ảnh (.jpg/.png) + nhãn YOLO cùng tên (.txt). Có thể gồm nhiều thư mục con
(phần A/B/C của 3 người). "Buổi" được xác định:
  * --group folder  : theo tên thư mục con chứa ảnh (vd dataset/labeled/buoi_sang_A/...)  [mặc định]
  * --group prefix  : theo phần đầu tên file trước dấu "_" (vd sang1_0001.jpg -> "sang1")
  * --group regex --pattern "(buoi\\d+)" : theo biểu thức chính quy

  python training/split_dataset.py --src dataset/labeled --val 0.2 --test 0.1
Đầu ra: dataset/images/{train,val,test}, dataset/labels/{train,val,test} + báo cáo số đối tượng mỗi lớp,
cảnh báo lớp thiếu và ẢNH GẦN TRÙNG giữa các tập (so dHash 64 bit).
"""
from __future__ import annotations

import argparse
import random
import re
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List

PROJECT_DIR = Path(__file__).resolve().parent.parent
IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp"}
SPLITS = ("train", "val", "test")


def session_of(img: Path, src: Path, mode: str, pattern: str = "") -> str:
    if mode == "folder":
        rel = img.parent.relative_to(src)
        return str(rel) if str(rel) != "." else img.stem.split("_")[0]
    if mode == "regex":
        m = re.search(pattern, img.name)
        return m.group(1) if m else "khac"
    return img.stem.split("_")[0]


def assign_groups(groups: Dict[str, List[Path]], val: float, test: float, seed: int) -> Dict[str, str]:
    """Chia nguyên cả buổi vào 1 tập, cố gắng sát tỉ lệ yêu cầu theo SỐ ẢNH."""
    names = sorted(groups)
    random.Random(seed).shuffle(names)
    total = sum(len(v) for v in groups.values())
    want = {"test": test * total, "val": val * total}
    have = {"train": 0, "val": 0, "test": 0}
    out = {}
    for g in sorted(names, key=lambda n: -len(groups[n])):
        n = len(groups[g])
        # tập nào còn thiếu nhiều nhất (so với mục tiêu) thì nhận buổi này
        need = {s: want[s] - have[s] for s in ("test", "val") if want[s] - have[s] >= 0.5 * n}
        s = max(need, key=need.get) if need else "train"
        out[g] = s
        have[s] += n
    if "train" not in out.values() and out:
        out[max(out, key=lambda k: len(groups[k]))] = "train"
    return out


def dhash(path: Path):
    import cv2
    im = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if im is None:
        return None
    im = cv2.resize(im, (9, 8), interpolation=cv2.INTER_AREA)
    bits = (im[:, 1:] > im[:, :-1]).flatten()
    return int("".join("1" if b else "0" for b in bits), 2)


def near_duplicates(split_imgs: Dict[str, List[Path]], max_dist: int = 5):
    hashes = {s: [(p, dhash(p)) for p in imgs] for s, imgs in split_imgs.items()}
    pairs = []
    for s in ("val", "test"):
        for p, h in hashes.get(s, []):
            if h is None:
                continue
            for q, g in hashes.get("train", []):
                if g is not None and bin(h ^ g).count("1") <= max_dist:
                    pairs.append((s, p.name, q.name))
                    break
    return pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="thư mục ảnh + nhãn .txt (có thể có thư mục con A/B/C)")
    ap.add_argument("--dst", default=str(PROJECT_DIR / "dataset"))
    ap.add_argument("--val", type=float, default=0.2)
    ap.add_argument("--test", type=float, default=0.1)
    ap.add_argument("--group", choices=("folder", "prefix", "regex"), default="folder")
    ap.add_argument("--pattern", default="", help="regex có 1 nhóm (...) khi --group regex")
    ap.add_argument("--classes", default="tram_1,tram_2,tram_3", help="tên lớp theo thứ tự 0,1,2...")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    src, dst = Path(args.src), Path(args.dst)
    names = args.classes.split(",")
    images = sorted(p for p in src.rglob("*") if p.suffix.lower() in IMG_EXT)
    if not images:
        raise SystemExit(f"Không có ảnh trong {src}")
    groups: Dict[str, List[Path]] = defaultdict(list)
    for p in images:
        groups[session_of(p, src, args.group, args.pattern)].append(p)
    if len(groups) < 3:
        print(f"!! Chỉ có {len(groups)} buổi chụp -> không chia đủ train/val/test theo buổi. "
              "Nên chụp ít nhất 4–6 buổi (khác giờ, khác góc, khác chỗ).")
    plan = assign_groups(groups, args.val, args.test, args.seed)

    split_imgs: Dict[str, List[Path]] = {s: [] for s in SPLITS}
    counts = {s: Counter() for s in SPLITS}
    bad = []
    for s in SPLITS:
        (dst / "images" / s).mkdir(parents=True, exist_ok=True)
        (dst / "labels" / s).mkdir(parents=True, exist_ok=True)
    for g, imgs in groups.items():
        s = plan[g]
        for img in imgs:
            name = f"{re.sub(r'[^A-Za-z0-9_-]+', '_', g)}__{img.name}"      # tránh trùng tên giữa A/B/C
            shutil.copy2(img, dst / "images" / s / name)
            lab = img.with_suffix(".txt")
            target = dst / "labels" / s / (Path(name).stem + ".txt")
            if lab.exists():
                shutil.copy2(lab, target)
                for line in lab.read_text().splitlines():
                    if not line.strip():
                        continue
                    f = line.split()
                    try:
                        c = int(f[0])
                        coords = [float(x) for x in f[1:5]]
                        if c >= len(names) or len(f) != 5 or not all(0.0 <= v <= 1.0 for v in coords):
                            raise ValueError
                        counts[s][c] += 1
                    except (ValueError, IndexError):
                        bad.append(f"{lab.name}: '{line.strip()}'")
            else:
                target.write_text("")                                   # ảnh nền (không có trạm)
            split_imgs[s].append(dst / "images" / s / name)

    print(f"{'tập':6s} {'buổi':>5s} {'ảnh':>6s}  " + "  ".join(f"{n:>8s}" for n in names))
    for s in SPLITS:
        nb = sum(1 for g in plan if plan[g] == s)
        print(f"{s:6s} {nb:5d} {len(split_imgs[s]):6d}  " + "  ".join(f"{counts[s][i]:8d}" for i in range(len(names))))
        for i, n in enumerate(names):
            if split_imgs[s] and counts[s][i] == 0:
                print(f"  !! tập {s} KHÔNG có đối tượng lớp {n} – đổi --seed hoặc chụp thêm buổi có {n}")
    print("Buổi -> tập:", {g: plan[g] for g in sorted(plan)})
    if bad:
        print(f"!! {len(bad)} dòng nhãn SAI định dạng / mã lớp (≥ {len(names)} hoặc tọa độ ngoài 0..1), ví dụ:")
        for b in bad[:10]:
            print("   ", b)
    dups = near_duplicates(split_imgs)
    if dups:
        print(f"!! {len(dups)} ảnh ở val/test GẦN TRÙNG ảnh train (nên chuyển cả buổi đó sang train / chụp lại):")
        for s, a, b in dups[:10]:
            print(f"    {s}: {a}  ~  train: {b}")
    else:
        print("Không thấy ảnh gần trùng giữa train và val/test.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
