"""Render zoomed RGB overlays of each overlap seam of the first_floor merge.

Red = some_map walls, blue = second_half_building walls, yellow-ish tint
where both maps claim free space. Output: /tmp/seam_left.png,
/tmp/seam_mid.png, /tmp/seam_right.png — for eyeballing the true local
misalignment instead of trusting a correlation metric.
"""
import cv2
import numpy as np

from PIL import Image
from scipy import ndimage

M = '/home/ros/Documents/matrix_turtlebot/maps/'
ANG, DY, DX = 196.0, -474, 392

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


def place(shape, img, dy, dx, fill=205):
    out = np.full(shape, fill, img.dtype)
    ys, xs = max(0, dy), max(0, dx)
    ye = min(shape[0], dy + img.shape[0])
    xe = min(shape[1], dx + img.shape[1])
    out[ys:ye, xs:xe] = img[ys - dy:ye - dy, xs - dx:xe - dx]
    return out


Brot = rotate(B, -ANG, 205)
H = max(A.shape[0], DY + Brot.shape[0]) - min(0, DY)
W = max(A.shape[1], DX + Brot.shape[1]) - min(0, DX)
oy, ox = -min(0, DY), -min(0, DX)
cA = place((H, W), A, oy, ox)
cB = place((H, W), Brot, DY + oy, DX + ox)

overlap = (cA != 205) & (cB != 205)
labels, n = ndimage.label(ndimage.binary_closing(overlap, iterations=3))
sizes = ndimage.sum(overlap, labels, range(1, n + 1))
zones = sorted((i + 1 for i in range(n) if sizes[i] > 1600),
               key=lambda z: ndimage.center_of_mass(labels == z)[1])

names = ['left', 'mid', 'right']
for name, z in zip(names, zones):
    ys, xs = np.where(labels == z)
    m = 60
    y0, y1 = max(0, ys.min() - m), min(H, ys.max() + m)
    x0, x1 = max(0, xs.min() - m), min(W, xs.max() + m)
    a, b = cA[y0:y1, x0:x1], cB[y0:y1, x0:x1]
    img = np.full((*a.shape, 3), 235, np.uint8)
    img[(a > 240) & (b > 240)] = (255, 255, 210)   # both free: pale yellow
    img[(a < 100)] = (60, 60, 220)                 # A walls red (BGR)
    img[(b < 100)] = (220, 80, 60)                 # B walls blue
    img[(a < 100) & (b < 100)] = (140, 40, 140)    # agree: purple
    # 1 m grid for scale
    for gy in range(0, img.shape[0], 20):
        img[gy, ::4] = (190, 190, 190)
    for gx in range(0, img.shape[1], 20):
        img[::4, gx] = (190, 190, 190)
    out = f'/tmp/seam_{name}.png'
    cv2.imwrite(out, cv2.resize(img, None, fx=2, fy=2,
                                interpolation=cv2.INTER_NEAREST))
    print(out, 'canvas region', (y0, y1, x0, x1))
