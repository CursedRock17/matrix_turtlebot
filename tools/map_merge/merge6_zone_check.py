"""Measure per-overlap-zone misalignment of the first_floor merge.

For a candidate transform (angle, dy, dx) of second_half_building onto
some_map, find the connected overlap zones (both maps known), and inside
each zone locally cross-correlate the two wall masks to get the residual
offset vector there. A rigid fit is right when every zone's residual is
~0; a rotation error shows up as residuals that grow with distance from
the well-anchored zone.

Usage: python3 merge6_zone_check.py [angle dy dx]   (default 196.0 -474 392)
"""
import sys

import cv2
import numpy as np

from PIL import Image
from scipy import ndimage

M = '/home/ros/Documents/matrix_turtlebot/maps/'
ANG, DY, DX = (float(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])) \
    if len(sys.argv) == 4 else (196.0, -474, 392)

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


def zone_offsets(ang, dy, dx, verbose=True):
    Brot = rotate(B, -ang, 205)
    H = max(A.shape[0], dy + Brot.shape[0]) - min(0, dy)
    W = max(A.shape[1], dx + Brot.shape[1]) - min(0, dx)
    oy, ox = -min(0, dy), -min(0, dx)
    cA = place((H, W), A, oy, ox)
    cB = place((H, W), Brot, dy + oy, dx + ox)

    overlap = (cA != 205) & (cB != 205)
    # ignore slivers: keep zones with real area (>= ~4 m^2)
    labels, n = ndimage.label(ndimage.binary_closing(overlap, iterations=3))
    sizes = ndimage.sum(overlap, labels, range(1, n + 1))
    zones = [i + 1 for i in range(n) if sizes[i] > 1600]

    results = []
    for z in sorted(zones, key=lambda z: ndimage.center_of_mass(overlap, labels, z)[1]):
        mask = labels == z
        cy, cx = ndimage.center_of_mass(mask)
        ys, xs = np.where(mask)
        y0, y1, x0, x1 = ys.min(), ys.max(), xs.min(), xs.max()
        wallsA = ((cA < 100) & mask)[y0:y1 + 1, x0:x1 + 1].astype(np.float32)
        wallsB = ((cB < 100) & mask)[y0:y1 + 1, x0:x1 + 1].astype(np.float32)
        if wallsA.sum() < 50 or wallsB.sum() < 50:
            continue
        # correlate B against A inside the zone over a +/-25 px search window
        pad = 25
        tpl = wallsB[pad:-pad, pad:-pad]
        if min(tpl.shape) < 20:
            continue
        cc = cv2.matchTemplate(wallsA, tpl, cv2.TM_CCORR)
        _, score, _, loc = cv2.minMaxLoc(cc)
        off_dx, off_dy = loc[0] - pad, loc[1] - pad
        results.append((cx, cy, off_dx, off_dy, mask.sum(), score))
        if verbose:
            print(f'zone @canvas x={cx:6.0f} y={cy:6.0f} area={mask.sum():7d}px  '
                  f'B is off by (dx={off_dx:+d}, dy={off_dy:+d}) px '
                  f'[{off_dx * 5:+d} cm, {off_dy * 5:+d} cm]')
    return results


if __name__ == '__main__':
    print(f'transform: angle={ANG} dy={DY} dx={DX}')
    res = zone_offsets(ANG, DY, DX)
    if len(res) >= 2:
        # residual rotation implied by the two most distant zones
        (x1, y1, dx1, dy1, *_), (x2, y2, dx2, dy2, *_) = res[0], res[-1]
        lever = np.hypot(x2 - x1, y2 - y1)
        dvx, dvy = dx2 - dx1, dy2 - dy1
        # tangential component of the differential offset across the lever
        ux, uy = (x2 - x1) / lever, (y2 - y1) / lever
        tangential = -dvx * uy + dvy * ux
        theta = np.degrees(tangential / lever)
        print(f'\nlever {lever:.0f}px between outer zones; differential offset '
              f'({dvx:+d},{dvy:+d})px -> residual rotation ~{theta:+.3f} deg')
