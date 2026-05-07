#!/usr/bin/env python3
import cv2, os
base = os.path.expanduser("~/pcl_develop_ws/src/pcl_detection2")

d0 = cv2.imread(f"{base}/temp/pillar_dump_0_f5_case-1_s50.png", cv2.IMREAD_GRAYSCALE)
d1 = cv2.imread(f"{base}/temp/pillar_dump_1_f10_case-1_s50.png", cv2.IMREAD_GRAYSCALE)
d2 = cv2.imread(f"{base}/temp/pillar_dump_2_f15_case-1_s50.png", cv2.IMREAD_GRAYSCALE)

d01 = cv2.countNonZero(cv2.absdiff(d0, d1))
d02 = cv2.countNonZero(cv2.absdiff(d0, d2))
print(f"dump_0 == dump_1: {d01} diff pixels → {'IDENTICAL' if d01 == 0 else 'DIFFERENT'}")
print(f"dump_0 == dump_2: {d02} diff pixels")
print()

# Show each dump and template
for label, img in [("dump_0",d0), ("dump_2",d2)]:
    print(f"=== {label} ({img.shape[1]}x{img.shape[0]}) white={cv2.countNonZero(img)} ===")
    for y in range(0, 53, 2):
        row = ""
        for x in range(img.shape[1]):
            row += "#" if img[y, x] > 127 else "."
        print(f"{y:2d}|{row}|")
    print()

for name in sorted(os.listdir(f"{base}/templates")):
    if not name.startswith("pillar"): continue
    t = cv2.imread(f"{base}/templates/{name}", cv2.IMREAD_GRAYSCALE)
    print(f"=== {name} ({t.shape[1]}x{t.shape[0]}) white={cv2.countNonZero(t)} ===")
    for y in range(0, 53, 2):
        row = ""
        for x in range(26):
            row += "#" if t[y, x] > 127 else "."
        print(f"{y:2d}|{row}|")
    print()
