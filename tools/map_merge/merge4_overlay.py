import numpy as np, cv2
from PIL import Image

M = '/home/ros/Documents/matrix_turtlebot/maps/'
A = np.array(Image.open(M + 'some_map.pgm'))
B = np.array(Image.open(M + 'second_half_building.pgm'))
ANG, DY, DX = 196.0, -474, 392

def rotate(im, ang, fill=0):
    h, w = im.shape
    diag = int(np.ceil(np.hypot(h, w)))
    Mr = cv2.getRotationMatrix2D((w/2, h/2), ang, 1.0)
    Mr[0, 2] += (diag - w) / 2
    Mr[1, 2] += (diag - h) / 2
    return cv2.warpAffine(im, Mr, (diag, diag), flags=cv2.INTER_NEAREST, borderValue=fill)

occA = (A < 100); freeA = (A > 240)
Brot = rotate(B.astype(np.float32), -ANG, 205).astype(np.uint8)
occB = (Brot < 100); freeB = (Brot > 240)

H = max(A.shape[0], DY + Brot.shape[0]) - min(0, DY)
W = max(A.shape[1], DX + Brot.shape[1]) - min(0, DX)
oy, ox = -min(0, DY), -min(0, DX)

def place(mask, y, x):
    out = np.zeros((H, W), bool)
    h, w = mask.shape
    ys, xs = max(0, y), max(0, x)
    ye, xe = min(H, y + h), min(W, x + w)
    out[ys:ye, xs:xe] = mask[ys - y:ye - y, xs - x:xe - x]
    return out

pA_occ, pA_free = place(occA, oy, ox), place(freeA, oy, ox)
pB_occ, pB_free = place(occB, DY + oy, DX + ox), place(freeB, DY + oy, DX + ox)

rgb = np.full((H, W, 3), 230, np.uint8)
rgb[pA_free] = (255, 245, 245)
rgb[pB_free] = (245, 245, 255)
rgb[pA_free & pB_free] = (255, 255, 230)
rgb[pA_occ] = (220, 40, 40)
rgb[pB_occ] = (40, 40, 220)
rgb[pA_occ & pB_occ] = (90, 0, 90)
Image.fromarray(rgb).save('/tmp/merge_overlay.png')
print('canvas', H, W, 'saved /tmp/merge_overlay.png')
# zoom of the overlap region for detail
ys, xs = np.where(pA_occ & pB_occ)
if len(ys):
    y0, y1, x0, x1 = ys.min(), ys.max(), xs.min(), xs.max()
    print('overlap bbox rows', y0, y1, 'cols', x0, x1)
    crop = rgb[max(0,y0-20):y1+20, max(0,x0-20):x1+20]
    Image.fromarray(crop).save('/tmp/merge_overlap_zoom.png')
    print('zoom size', crop.shape)
