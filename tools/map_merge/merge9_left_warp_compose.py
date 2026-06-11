"""Recompose first_floor with second_half_building's left-end drift corrected.

The user identified a ground-truth correspondence: a corner of a 20 ft wall
visible in BOTH maps at the left end (some_map's lidar saw it over the
stairwell). A local brute-force alignment around that corner shows
second_half_building must move (up 20 px, right 23 px) ~= 1.5 m to match —
gmapping odometry drift accumulated from the map's starting point in the
middle (where alignment is perfect) out to the ends.

The correction is applied as a smooth ramp: full shift left of canvas
x=700, fading to zero by x=845 (the middle overlap starts ~847 and is
verified perfect, so it must not move). Composition rules are merge8's:
B wins in the end zones, ghost suppression, A wins elsewhere.
"""
import cv2
import numpy as np

from PIL import Image
from scipy import ndimage

M = '/home/ros/Documents/matrix_turtlebot/maps/'
RES = 0.05
ANG, DY, DX = 196.0, -474, 392
A_origin = (-63.234, -11.410)

# left-end drift correction for B, from the 20ft-wall corner match:
# content moves up 20 px and right 23 px at full strength
SHIFT_Y, SHIFT_X = -20.0, +23.0
RAMP_FULL, RAMP_ZERO = 700.0, 845.0   # canvas x: full correction .. none

A = np.array(Image.open(M + 'some_map.pgm'))
B = np.array(Image.open(M + 'second_half_building.pgm'))


def rotate(im, ang, fill):
    h, w = im.shape
    diag = int(np.ceil(np.hypot(h, w)))
    Mr = cv2.getRotationMatrix2D((w / 2, h / 2), ang, 1.0)
    Mr[0, 2] += (diag - w) / 2
    Mr[1, 2] += (diag - h) / 2
    return cv2.warpAffine(im, Mr, (diag, diag), flags=cv2.INTER_NEAREST,
                          borderValue=fill)


Brot = rotate(B, -ANG, 205)
H = max(A.shape[0], DY + Brot.shape[0]) - min(0, DY)
W = max(A.shape[1], DX + Brot.shape[1]) - min(0, DX)
oy, ox = -min(0, DY), -min(0, DX)

cA = np.full((H, W), 205, np.uint8)
cA[oy:oy + A.shape[0], ox:ox + A.shape[1]] = A
cB = np.full((H, W), 205, np.uint8)
by, bx = DY + oy, DX + ox
ys, xs = max(0, by), max(0, bx)
ye, xe = min(H, by + Brot.shape[0]), min(W, bx + Brot.shape[1])
cB[ys:ye, xs:xe] = Brot[ys - by:ye - by, xs - bx:xe - bx]

# ---- drift-correcting warp of B (smoothstep ramp on destination x) ----
xx = np.arange(W, dtype=np.float32)
t = np.clip((RAMP_ZERO - xx) / (RAMP_ZERO - RAMP_FULL), 0.0, 1.0)
wgt = t * t * (3 - 2 * t)                      # smoothstep
map_x = (xx[None, :] - SHIFT_X * wgt[None, :]).repeat(H, axis=0)
map_y = (np.arange(H, dtype=np.float32)[:, None]
         - SHIFT_Y * wgt[None, :])  # broadcasts to (H, W)
cB = cv2.remap(cB, map_x, map_y, cv2.INTER_NEAREST,
               borderMode=cv2.BORDER_CONSTANT, borderValue=205)
print(f'warp applied: B moves ({-SHIFT_Y:.0f} up, {SHIFT_X:.0f} right) px '
      f'left of x={RAMP_FULL:.0f}, fading to 0 at x={RAMP_ZERO:.0f}')

# ---- composition: identical to merge8 ----
overlap = (cA != 205) & (cB != 205)
xgrid = np.arange(W)[None, :]
b_zone = ndimage.binary_dilation(
    overlap & ((xgrid < 750) | (xgrid > 1830)), iterations=15)

canvas = np.full((H, W), 205, np.uint8)
canvas[cB != 205] = cB[cB != 205]
a_wins = (cA != 205) & ~(b_zone & (cB != 205))
ghost = (cA < 100) & b_zone & \
    cv2.dilate((cB < 100).astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
a_wins &= ~ghost
canvas[a_wins] = cA[a_wins]

known = canvas != 205
rows = np.where(known.any(axis=1))[0]
cols = np.where(known.any(axis=0))[0]
r0, r1, c0, c1 = rows.min(), rows.max(), cols.min(), cols.max()
pad = 4
r0, c0 = max(0, r0 - pad), max(0, c0 - pad)
r1, c1 = min(H - 1, r1 + pad), min(W - 1, c1 + pad)
out = canvas[r0:r1 + 1, c0:c1 + 1]
print(f'trimmed to {out.shape} (rows {r0}..{r1}, cols {c0}..{c1})')

out_h = out.shape[0]
a_bl_row = oy + A.shape[0] - 1 - r0
a_bl_col = ox - c0
origin_x = A_origin[0] - a_bl_col * RES
origin_y = A_origin[1] - (out_h - 1 - a_bl_row) * RES
print(f'merged origin: [{origin_x:.3f}, {origin_y:.3f}]')

with open(M + 'first_floor.pgm', 'wb') as f:
    f.write(b'P5\n%d %d\n255\n' % (out.shape[1], out.shape[0]))
    f.write(out.tobytes())
with open(M + 'first_floor.yaml', 'w') as f:
    f.write(f"""image: first_floor.pgm
mode: trinary
resolution: 0.050
origin: [{origin_x:.3f}, {origin_y:.3f}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")

# ---- verification ----
# 1) world frame of A preserved outside B zones
occ = np.argwhere(A < 100)
rng = np.random.default_rng(0)
ok, checked = True, 0
for r, c in occ[rng.choice(len(occ), 50, replace=False)]:
    if b_zone[oy + r, ox + c]:
        continue
    checked += 1
    wx = A_origin[0] + (c + 0.5) * RES
    wy = A_origin[1] + (A.shape[0] - 1 - r + 0.5) * RES
    mr, mc = oy + r - r0, ox + c - c0
    wx2 = origin_x + (mc + 0.5) * RES
    wy2 = origin_y + (out_h - 1 - mr + 0.5) * RES
    ok &= abs(wx - wx2) < 1e-9 and abs(wy - wy2) < 1e-9 and out[mr, mc] == A[r, c]
print(f'WORLD-FRAME CHECK ({checked} samples):', 'PASS' if ok else 'FAIL')

# 2) local agreement at the matched corner and in the middle, post-warp
def agreement(box, R=6):
    yy0, yy1, xx0, xx1 = box
    wA = (cA[yy0:yy1, xx0:xx1] < 100)
    dA = cv2.dilate(wA.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    wB = (cB < 100)
    best, base = 0.0, 0.0
    for dy in range(-R, R + 1):
        for dx in range(-R, R + 1):
            sb = wB[yy0 + dy:yy1 + dy, xx0 + dx:xx1 + dx]
            if sb.sum() < 100:
                continue
            s = (sb & dA).sum() / sb.sum()
            if dy == 0 and dx == 0:
                base = s
            best = max(best, s)
    return base, best

for name, box in [('corner', (900, 1010, 580, 700)),
                  ('middle', (1172, 1703, 860, 1800))]:
    base, best = agreement(box)
    print(f'{name}: agreement at (0,0) = {base:.3f} (best nearby {best:.3f})')
