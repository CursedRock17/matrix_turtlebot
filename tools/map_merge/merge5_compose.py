import numpy as np, cv2
from PIL import Image

M = '/home/ros/Documents/matrix_turtlebot/maps/'
RES = 0.05
ANG, DY, DX = 196.0, -474, 392

A = np.array(Image.open(M + 'some_map.pgm'))
B = np.array(Image.open(M + 'second_half_building.pgm'))
A_origin = (-63.234, -11.410)

def rotate(im, ang, fill):
    h, w = im.shape
    diag = int(np.ceil(np.hypot(h, w)))
    Mr = cv2.getRotationMatrix2D((w/2, h/2), ang, 1.0)
    Mr[0, 2] += (diag - w) / 2
    Mr[1, 2] += (diag - h) / 2
    return cv2.warpAffine(im, Mr, (diag, diag), flags=cv2.INTER_NEAREST,
                          borderValue=fill)

Brot = rotate(B, -ANG, 205)

H = max(A.shape[0], DY + Brot.shape[0]) - min(0, DY)
W = max(A.shape[1], DX + Brot.shape[1]) - min(0, DX)
oy, ox = -min(0, DY), -min(0, DX)
print(f'canvas {H}x{W}, A at ({oy},{ox}), Brot at ({DY+oy},{DX+ox})')

canvas = np.full((H, W), 205, np.uint8)

# B first (only fills), then A overwrites wherever A is known
by, bx = DY + oy, DX + ox
ys, xs = max(0, by), max(0, bx)
ye, xe = min(H, by + Brot.shape[0]), min(W, bx + Brot.shape[1])
sub = Brot[ys - by:ye - by, xs - bx:xe - bx]
region = canvas[ys:ye, xs:xe]
region[sub != 205] = sub[sub != 205]

ay, ax = oy, ox
regionA = canvas[ay:ay + A.shape[0], ax:ax + A.shape[1]]
regionA[A != 205] = A[A != 205]

# trim fully-unknown border rows/cols
known = canvas != 205
rows = np.where(known.any(axis=1))[0]
cols = np.where(known.any(axis=0))[0]
r0, r1, c0, c1 = rows.min(), rows.max(), cols.min(), cols.max()
pad = 4
r0, c0 = max(0, r0 - pad), max(0, c0 - pad)
r1, c1 = min(H - 1, r1 + pad), min(W - 1, c1 + pad)
out = canvas[r0:r1 + 1, c0:c1 + 1]
print(f'trimmed to {out.shape} (rows {r0}..{r1}, cols {c0}..{c1})')

# merged yaml origin preserving some_map's world frame:
# A grid cell (0,0) = image pixel (row A_h-1, col 0) sits at world A_origin.
# In OUT image, A's bottom-left pixel is at row (ay + A_h - 1 - r0), col (ax - c0).
out_h = out.shape[0]
a_bl_row = ay + A.shape[0] - 1 - r0
a_bl_col = ax - c0
origin_x = A_origin[0] - a_bl_col * RES
origin_y = A_origin[1] - (out_h - 1 - a_bl_row) * RES
print(f'merged origin: [{origin_x:.3f}, {origin_y:.3f}]')

vals = np.unique(out)
print('pixel values in output:', vals)

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

# ---- verification 1: world-coordinate preservation for some_map content ----
occ = np.argwhere(A < 100)
rng = np.random.default_rng(0)
ok = True
for r, c in occ[rng.choice(len(occ), 5, replace=False)]:
    wx = A_origin[0] + (c + 0.5) * RES
    wy = A_origin[1] + (A.shape[0] - 1 - r + 0.5) * RES
    mr, mc = ay + r - r0, ax + c - c0
    wx2 = origin_x + (mc + 0.5) * RES
    wy2 = origin_y + (out_h - 1 - mr + 0.5) * RES
    same_val = out[mr, mc] == A[r, c]
    match = abs(wx - wx2) < 1e-9 and abs(wy - wy2) < 1e-9
    ok &= match and same_val
    print(f'A px({r},{c}) world=({wx:.3f},{wy:.3f}) merged=({wx2:.3f},{wy2:.3f}) '
          f'val {A[r,c]}=={out[mr,mc]} -> {"OK" if match and same_val else "FAIL"}')
print('WORLD-FRAME CHECK:', 'PASS' if ok else 'FAIL')

Image.fromarray(out).save('/tmp/first_floor.png')
