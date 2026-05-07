#!/usr/bin/env python3
"""测试柱子模板匹配：dump图 x2 放大 vs 4个模板"""

import cv2
import os
import numpy as np

base = os.path.expanduser("~/pcl_develop_ws/src/pcl_detection2")
temp_dir = os.path.join(base, "temp")
tpl_dir = os.path.join(base, "templates")

dumps = sorted([f for f in os.listdir(temp_dir) if f.endswith(".png")])
tpl_files = sorted([f for f in os.listdir(tpl_dir) if f.startswith("pillar")])

# 加载原始模板
tpl_imgs = {}
for f in tpl_files:
    tpl_imgs[f] = cv2.imread(os.path.join(tpl_dir, f), cv2.IMREAD_GRAYSCALE)

for dump_name in dumps:
    dump_raw = cv2.imread(os.path.join(temp_dir, dump_name), cv2.IMREAD_GRAYSCALE)
    print(f"\n{'='*80}")
    print(f"  {dump_name} (原{dump_raw.shape[1]}x{dump_raw.shape[0]})")
    print(f"{'='*80}")

    # 方案 A: 原始尺寸 + pad 1px (当前做法)
    dump_pad = cv2.copyMakeBorder(dump_raw, 0, 0, 0, 1, cv2.BORDER_CONSTANT, value=0)
    print(f"\n--- 原始尺寸 (pad后{dump_pad.shape[1]}x{dump_pad.shape[0]}) ---")
    for name in tpl_files:
        tpl = tpl_imgs[name]
        r = cv2.matchTemplate(dump_pad, tpl, cv2.TM_CCOEFF_NORMED)
        _, s, _, loc = cv2.minMaxLoc(r)
        print(f"  {name}: {s:+.4f} @ {loc}")

    # 方案 B: x2 放大 (最近邻保持二值)
    new_w, new_h = dump_raw.shape[1] * 2, dump_raw.shape[0] * 2
    dump_2x = cv2.resize(dump_raw, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
    print(f"\n--- x2放大 (最近邻, {dump_2x.shape[1]}x{dump_2x.shape[0]}) ---")

    best_name = None
    best_score = -999
    best_loc = None
    tpl_2x_list = []
    
    for name in tpl_files:
        tpl_raw = tpl_imgs[name]
        tpl_2x = cv2.resize(tpl_raw, (tpl_raw.shape[1]*2, tpl_raw.shape[0]*2),
                            interpolation=cv2.INTER_NEAREST)
        tpl_2x_list.append((name, tpl_2x))

        r = cv2.matchTemplate(dump_2x, tpl_2x, cv2.TM_CCOEFF_NORMED)
        _, s, _, loc = cv2.minMaxLoc(r)
        print(f"  {name}: {s:+.4f} @ {loc}")
        if s > best_score:
            best_score = s
            best_name = name
            best_loc = loc

    # 方案 C: x2 放大 dump + x2 缩小模板 (模板变小 → 有滑动空间)
    # 模板 x2 放大但 dump x4 放大 → 模板 < 图 → 有滑动
    dump_4x = cv2.resize(dump_raw, (dump_raw.shape[1]*4, dump_raw.shape[0]*4),
                         interpolation=cv2.INTER_NEAREST)
    print(f"\n--- dump x4({dump_4x.shape[1]}x{dump_4x.shape[0]}) vs 模板 x2({tpl_2x_list[0][1].shape[1]}x{tpl_2x_list[0][1].shape[0]}) ---")
    for name, tpl_2x in tpl_2x_list:
        r = cv2.matchTemplate(dump_4x, tpl_2x, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(r)
        print(f"  {name}: max={max_val:+.4f} @ {max_loc}  min={min_val:+.4f}")

    # 可视化最佳匹配 (方案B)
    if best_name:
        vis = cv2.cvtColor(dump_2x, cv2.COLOR_GRAY2BGR)
        best_tpl = [t for n, t in tpl_2x_list if n == best_name][0]
        cv2.rectangle(vis, best_loc, 
                     (best_loc[0]+best_tpl.shape[1], best_loc[1]+best_tpl.shape[0]),
                     (0, 255, 0), 1)
        cv2.putText(vis, f"{best_name} {best_score:.3f}", (2, 12),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
        out_path = os.path.join(temp_dir, "match_debug",
                                dump_name.replace(".png", "_x2_match.png"))
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        cv2.imwrite(out_path, vis)
        print(f"\n  最佳: {best_name} score={best_score:.4f}, 可视化 → {out_path}")

    # 方案 D: dump x3, tpl x2 (有滑动空间的折中)
    dump_3x = cv2.resize(dump_raw, (dump_raw.shape[1]*3, dump_raw.shape[0]*3),
                         interpolation=cv2.INTER_NEAREST)
    print(f"\n--- dump x3({dump_3x.shape[1]}x{dump_3x.shape[0]}) vs tpl x2 ---")
    for name, tpl_2x in tpl_2x_list:
        r = cv2.matchTemplate(dump_3x, tpl_2x, cv2.TM_CCOEFF_NORMED)
        _, s, _, loc = cv2.minMaxLoc(r)
        print(f"  {name}: {s:+.4f} @ {loc}")
