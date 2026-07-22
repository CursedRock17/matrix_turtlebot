"""Recompose first_floor with the stairwell artifacts fixed.

Same rigid transform and world-frame-preserving origin math as
merge5_compose.py, with one change: inside the LEFT overlap zone,
second_half_building wins conflicts instead of some_map. some_map's data
there is lidar returns over a descending stairwell (phantom walls floating
in the corridor + false free space that punched holes through the real
corridor walls), while second_half_building actually drove that corridor.

Everywhere else the original rule stands: some_map wins where known, so
the north wing stays pixel-identical and surveyed coordinates stay valid.
"""
import cv2
import numpy as np

from PIL import Image
from scipy import ndimage

M = '/home/ros/Documents/matrix_turtlebot/maps/'
RES = 0.05
ANG, DY, DX = 196.0, -474, 392

A = np.array(Image.open(M + 'some_map.pgm'))
B = np.array(Image.open(M + 'second_half_building.pgm'))
A_origin = (-63.234, -11.410)


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

# Full canvases of each source for zone logic
cA = np.full((H, W), 205, np.uint8)
cA[oy:oy + A.shape[0], ox:ox + A.shape[1]] = A
cB = np.full((H, W), 205, np.uint8)
by, bx = DY + oy, DX + ox
ys, xs = max(0, by), max(0, bx)
ye, xe = min(H, by + Brot.shape[0]), min(W, bx + Brot.shape[1])
cB[ys:ye, xs:xe] = Brot[ys - by:ye - by, xs - bx:xe - bx]

# B wins where the maps overlap at the building's ENDS: on the left A's
# data is stairwell artifacts (lidar over a descending stair), on the right
# it's A's far-end drift (wall agreement is only 4-7% at ANY rigid shift,
# so blending doubles every wall; B drove that wing for real). The middle
# overlap (canvas x ~850-1790) stays A-wins: agreement there is ~48%.
# Zones are ALL overlap pixels beyond the x cuts — component-picking missed
# detached overlap fragments and left A-wall islands inside B's rooms.
overlap = (cA != 205) & (cB != 205)
xgrid = np.arange(W)[None, :]
b_zone = ndimage.binary_dilation(
    overlap & ((xgrid < 750) | (xgrid > 1830)), iterations=15)
print(f'B-precedence zones: {b_zone.sum()} px after dilation')

canvas = np.full((H, W), 205, np.uint8)
canvas[cB != 205] = cB[cB != 205]
a_wins = (cA != 205) & ~(b_zone & (cB != 205))
# Ghost-wall suppression: inside the B zones, A's walls that hug a B wall
# within ~20 cm are the same physical wall drawn twice (A's far-end drift).
# They mostly sit where B is unknown (beyond B's wall), so the conflict rule
# above never sees them. Drop them; the pixel falls back to B's value
# (usually unknown, which is right for space behind a wall).
ghost = (cA < 100) & b_zone & \
    cv2.dilate((cB < 100).astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
a_wins &= ~ghost
print(f'ghost wall pixels suppressed: {int(((cA < 100) & a_wins & b_zone).sum())} '
      f'A-walls kept in zones, {int(ghost.sum())} dropped')
canvas[a_wins] = cA[a_wins]
changed = int((canvas != np.where(cA != 205, cA, cB)).sum())
print(f'pixels that differ from the old rule: {changed}')

# trim fully-unknown border (identical to merge5)
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

# verification: some_map world coordinates preserved (outside the left zone)
occ = np.argwhere((A < 100))
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
print(f'WORLD-FRAME CHECK ({checked} samples outside zone):',
      'PASS' if ok else 'FAIL')

# regenerate the composed-left crop for before/after comparison
y0, y1, x0, x1 = 817, 1103, 462, 685
crop = canvas[y0:y1, x0:x1]
img = np.full((*crop.shape, 3), 235, np.uint8)
img[crop > 240] = (255, 255, 255)
img[crop < 100] = (30, 30, 30)
cv2.imwrite('/tmp/composed_left_fixed.png',
            cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
print('/tmp/composed_left_fixed.png')
