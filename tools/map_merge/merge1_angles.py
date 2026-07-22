import numpy as np
from PIL import Image

def load(name):
    im = np.array(Image.open(f'/home/ros/Documents/matrix_turtlebot/maps/{name}.pgm'))
    return im

def dominant_angle(occ):
    # PCA-free approach: histogram of edge orientations from wall pixel pairs
    # Use Hough-style: for each occupied pixel, look at gradient of distance? Simpler:
    # take all occupied pixel coords, compute orientation histogram of vectors
    # between nearby occupied pixels.
    import cv2
    img = (occ * 255).astype(np.uint8)
    lines = cv2.HoughLinesP(img, 1, np.pi/720, threshold=60,
                            minLineLength=40, maxLineGap=5)
    if lines is None:
        return None
    angs, weights = [], []
    for l in lines[:, 0]:
        x1, y1, x2, y2 = l
        ang = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 90.0  # fold to [0,90)
        length = np.hypot(x2 - x1, y2 - y1)
        angs.append(ang); weights.append(length)
    angs = np.array(angs); weights = np.array(weights)
    # weighted circular mean on 90-degree period
    theta = np.radians(angs * 4)  # map [0,90) onto full circle
    c = np.average(np.cos(theta), weights=weights)
    s = np.average(np.sin(theta), weights=weights)
    mean = np.degrees(np.arctan2(s, c)) / 4 % 90
    # also report histogram peak for robustness
    hist, edges = np.histogram(angs, bins=180, range=(0, 90), weights=weights)
    peak = edges[np.argmax(hist)] + 0.25
    return mean, peak, len(lines)

for name in ['some_map', 'second_half_building']:
    im = load(name)
    occ = im < 100
    print(name, 'shape', im.shape, 'occupied px', occ.sum())
    print('  dominant angle (mean, hist-peak, nlines):', dominant_angle(occ))
