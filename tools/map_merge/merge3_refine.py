import numpy as np, cv2
from PIL import Image

M = '/home/ros/Documents/matrix_turtlebot/maps/'
A = np.array(Image.open(M + 'some_map.pgm'))
B = np.array(Image.open(M + 'second_half_building.pgm'))

occA = (A < 100).astype(np.float32)
freeA = (A > 240).astype(np.float32)
occA_d = cv2.dilate(occA, np.ones((5, 5), np.float32))  # +/-2 px tolerance

def rotate(im, ang, fill=0):
    h, w = im.shape
    diag = int(np.ceil(np.hypot(h, w)))
    Mr = cv2.getRotationMatrix2D((w/2, h/2), ang, 1.0)
    Mr[0, 2] += (diag - w) / 2
    Mr[1, 2] += (diag - h) / 2
    return cv2.warpAffine(im, Mr, (diag, diag), flags=cv2.INTER_NEAREST, borderValue=fill)

def corr_peak(a, b):
    sh = (a.shape[0] + b.shape[0] - 1, a.shape[1] + b.shape[1] - 1)
    fs = (1 << int(np.ceil(np.log2(sh[0]))), 1 << int(np.ceil(np.log2(sh[1]))))
    c = np.fft.irfft2(np.fft.rfft2(a, fs) * np.fft.rfft2(b[::-1, ::-1], fs), fs)[:sh[0], :sh[1]]
    idx = np.unravel_index(np.argmax(c), c.shape)
    return c[idx], idx[0] - (b.shape[0] - 1), idx[1] - (b.shape[1] - 1)

def place(canvas_shape, img, dy, dx):
    out = np.zeros(canvas_shape, np.float32)
    h, w = img.shape
    y0, x0 = dy, dx
    ys, xs = max(0, y0), max(0, x0)
    ye, xe = min(canvas_shape[0], y0 + h), min(canvas_shape[1], x0 + w)
    if ye > ys and xe > xs:
        out[ys:ye, xs:xe] = img[ys - y0:ye - y0, xs - x0:xe - x0]
    return out

occB_raw = (B < 100).astype(np.float32)
freeB_raw = (B > 240).astype(np.float32)

best = []
for center in (18.4, 198.4):
    for ang in np.arange(center - 3, center + 3.01, 0.2):
        ob = rotate(occB_raw, -ang)
        s, dy, dx = corr_peak(occA_d, ob)
        best.append((s, ang, dy, dx))
best.sort(key=lambda r: -r[0])
print("top 5 by dilated-wall overlap:")
for s, ang, dy, dx in best[:5]:
    print(f"  ang={ang:7.2f} dy={dy:5d} dx={dx:5d} score={s:9.1f}")

# detailed scoring of top 3 distinct angles
print("\ndetailed (good = walls agree, bad = wall-on-free contradiction):")
seen = set()
for s, ang, dy, dx in best:
    key = round(ang, 1)
    if key in seen: continue
    seen.add(key)
    if len(seen) > 3: break
    ob = rotate(occB_raw, -ang)
    fb = rotate(freeB_raw, -ang)
    # pad A-side canvases to fit placement
    H = max(occA.shape[0], dy + ob.shape[0]) - min(0, dy)
    W = max(occA.shape[1], dx + ob.shape[1]) - min(0, dx)
    oy, ox = -min(0, dy), -min(0, dx)
    cA_occ = place((H, W), occA, oy, ox)
    cA_occd = place((H, W), occA_d, oy, ox)
    cA_free = place((H, W), freeA, oy, ox)
    cB_occ = place((H, W), ob, dy + oy, dx + ox)
    cB_free = place((H, W), fb, dy + oy, dx + ox)
    good = float((cB_occ * cA_occd).sum())
    freeA_solid = cv2.erode(cA_free, np.ones((5, 5), np.float32))
    freeB_solid = cv2.erode(cB_free, np.ones((5, 5), np.float32))
    bad = float((cB_occ * freeA_solid).sum() + (cA_occ * freeB_solid).sum())
    overlap_known = float(((cA_occ + cA_free) * (cB_occ + cB_free) > 0).sum())
    print(f"  ang={ang:7.2f} dy={dy:5d} dx={dx:5d} good={good:8.0f} bad={bad:8.0f} "
          f"overlap_px={overlap_known:9.0f} good/bad={good/max(bad,1):.2f}")
