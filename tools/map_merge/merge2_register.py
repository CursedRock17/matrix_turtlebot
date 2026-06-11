import numpy as np, cv2
from PIL import Image

M = '/home/ros/Documents/matrix_turtlebot/maps/'
A = np.array(Image.open(M + 'some_map.pgm'))            # reference
B = np.array(Image.open(M + 'second_half_building.pgm'))  # to transform

def masks(im):
    return (im < 100).astype(np.float32), (im > 240).astype(np.float32)  # occ, free

occA, freeA = masks(A)

def rotate_img(im, ang_deg, fill):
    h, w = im.shape
    # output canvas large enough for any rotation
    diag = int(np.ceil(np.hypot(h, w)))
    Mrot = cv2.getRotationMatrix2D((w/2, h/2), ang_deg, 1.0)
    Mrot[0, 2] += (diag - w) / 2
    Mrot[1, 2] += (diag - h) / 2
    return cv2.warpAffine(im, Mrot, (diag, diag), flags=cv2.INTER_NEAREST,
                          borderValue=fill), Mrot

def corr_peak(a, b):
    # full cross-correlation via FFT; returns peak score and offset of b rel to a
    sh = (a.shape[0] + b.shape[0] - 1, a.shape[1] + b.shape[1] - 1)
    fs = (1 << int(np.ceil(np.log2(sh[0]))), 1 << int(np.ceil(np.log2(sh[1]))))
    Fa = np.fft.rfft2(a, fs)
    Fb = np.fft.rfft2(b[::-1, ::-1], fs)
    c = np.fft.irfft2(Fa * Fb, fs)[:sh[0], :sh[1]]
    idx = np.unravel_index(np.argmax(c), c.shape)
    # offset: b placed at (dy, dx) in a-coords
    dy = idx[0] - (b.shape[0] - 1)
    dx = idx[1] - (b.shape[1] - 1)
    return c[idx], dy, dx

base = 18.4
results = []
for k in range(4):
    ang = base + 90 * k
    occB, _ = rotate_img((B < 100).astype(np.float32), -ang, 0)  # cv2 angle is CCW for y-down? test both signs
    s, dy, dx = corr_peak(occA, occB)
    results.append(('k=%d ang=%.1f sign=-' % (k, ang), s, dy, dx))
    occB2, _ = rotate_img((B < 100).astype(np.float32), ang, 0)
    s2, dy2, dx2 = corr_peak(occA, occB2)
    results.append(('k=%d ang=%.1f sign=+' % (k, ang), s2, dy2, dx2))

results.sort(key=lambda r: -r[1])
for r in results:
    print('%-22s score=%9.1f  dy=%5d dx=%5d' % r)
