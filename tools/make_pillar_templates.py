#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# 文件: tools/make_pillar_templates.py
# 作者: 肚腩特大哥 (Claude 辅助)
# 日期: 2026-07
# 功能: 生成柱子模板匹配用的 4 张模板图 pillar_case_00~03.png
#   （pcl_detection2/core/pillar_detect.hpp 启动时从 templates/ 加载）
#
# 【case 命名规则（全队统一，与 raicom_vision_laser/config/traverse_map.yaml 一致）】
#   柱1候选：A左(2.7,1.55) / A右(3.3,1.55)；柱2候选：B左(2.7,2.8) / B右(3.3,2.8)
#     case0 = A左+B左（两柱都在 x=2.7）
#     case1 = A左+B右（2.7,1.55）+（3.3,2.8）
#     case2 = A右+B左（3.3,1.55）+（2.7,2.8）
#     case3 = A右+B右（两柱都在 x=3.3）
#
# 【像素几何 —— 与 pillar_detect.hpp 的投影完全一致】
#   模板 26 列 x 53 行，0.05 m/px；
#   模板坐标原点对应 odom 系 ROI 角 (x_min=-3.0, y_min=-2.65)：
#     col = (x_odom - (-3.0)) / 0.05,  row = (y_odom - (-2.65)) / 0.05
#   odom 系柱位 = 场地系绕出生点(0.65,0.75)转 180°：
#     A左 (2.7,1.55)->(-2.05,-0.80)->(col 19,row 37)
#     A右 (3.3,1.55)->(-2.65,-0.80)->(col  7,row 37)
#     B左 (2.7,2.80)->(-2.05,-2.05)->(col 19,row 12)
#     B右 (3.3,2.80)->(-2.65,-2.05)->(col  7,row 12)
#   白点半径 2px：柱直径 0.2m=4px，叠加检测端 3x3 膨胀后的匹配尺度。
#   运行时投影图比模板大（yaml pillar_roi 更宽），靠 matchTemplate
#   滑动对准，所以模板内点的"相对位置"必须精确，绝对边距不重要。
#
# 用法: python3 tools/make_pillar_templates.py
#   直接覆盖写入 ../templates/pillar_case_0X.png（旧文件先备份为 *_legacy.png）
# ============================================================

import os
import shutil
import sys

from PIL import Image, ImageDraw

# Windows 控制台默认 GBK，强制 UTF-8 输出
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# ---------------- 模板几何参数（改 pillar_roi/resolution 必须同步改这里重生成） ----------------
TPL_COLS = 26          # 1.3m / 0.05
TPL_ROWS = 53          # 2.65m / 0.05
RES = 0.05             # m/px
ORIGIN_X = -3.0        # 模板 (col=0,row=0) 对应的 odom 坐标
ORIGIN_Y = -2.65
DOT_R = 2              # 白点半径 px

# odom 系 4 个候选柱位（场系 -> odom: p_odom = (0.65-x_field, 0.75-y_field)）
A_LEFT = (-2.05, -0.80)    # 场系 (2.7, 1.55)
A_RIGHT = (-2.65, -0.80)   # 场系 (3.3, 1.55)
B_LEFT = (-2.05, -2.05)    # 场系 (2.7, 2.80)
B_RIGHT = (-2.65, -2.05)   # 场系 (3.3, 2.80)

# case -> 两个柱位（命名规则见文件头）
CASES = {
    0: (A_LEFT, B_LEFT),
    1: (A_LEFT, B_RIGHT),
    2: (A_RIGHT, B_LEFT),
    3: (A_RIGHT, B_RIGHT),
}


def to_pixel(pt):
    col = (pt[0] - ORIGIN_X) / RES
    row = (pt[1] - ORIGIN_Y) / RES
    return col, row


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    tpl_dir = os.path.join(here, "..", "templates")
    os.makedirs(tpl_dir, exist_ok=True)

    for cid, (p1, p2) in sorted(CASES.items()):
        img = Image.new("L", (TPL_COLS, TPL_ROWS), 0)
        draw = ImageDraw.Draw(img)
        for pt in (p1, p2):
            col, row = to_pixel(pt)
            draw.ellipse([col - DOT_R, row - DOT_R, col + DOT_R, row + DOT_R], fill=255)
            print("  case%d 柱位 odom(%.2f, %.2f) -> 像素(col %.1f, row %.1f)"
                  % (cid, pt[0], pt[1], col, row))

        path = os.path.join(tpl_dir, "pillar_case_%02d.png" % cid)
        # 旧模板备份一次（已备份过不重复覆盖）
        if os.path.exists(path):
            legacy = os.path.join(tpl_dir, "pillar_case_%02d_legacy.png" % cid)
            if not os.path.exists(legacy):
                shutil.copy2(path, legacy)
                print("  旧模板已备份: %s" % os.path.basename(legacy))
        img.save(path)
        print("  已生成 %s (%dx%d)" % (path, TPL_COLS, TPL_ROWS))


if __name__ == "__main__":
    main()
