"""Wall off the stairwell overlook in first_floor (run after merge9).

West of the corridor-connecting passage at the left end, some_map's lidar
saw over a railing and down a staircase, leaving false free space that
nothing in second_half_building can veto (B never drove over stairs). The
planner has allow_unknown: true, so the only safe fix is a real occupied
barrier: solidify the railing line (the dotted returns are the posts) and
mark the stair flight below it unknown, up to where B's data begins (the
stoop). The passage east of the barrier stays open — it is the drivable
connection between the corridors (user-confirmed).

Coordinates are first_floor pixel coords (canvas minus trim (470, 13)).
"""
import numpy as np

from PIL import Image

M = '/home/ros/Documents/matrix_turtlebot/maps/'
FF_PATH = M + 'first_floor.pgm'

# railing band to fit the barrier line through, and barrier x extent
FIT_Y0, FIT_Y1 = 355, 400
BAR_X0, BAR_X1 = 228, 543     # west taper .. passage west wall
BAND_DEPTH = 95               # how far below the line A's false free reaches

FF = np.array(Image.open(FF_PATH)).copy()
cBw = np.load('/tmp/cBw.npy')          # warped B canvas from merge9 run
bknown = cBw[470:470 + FF.shape[0], 13:13 + FF.shape[1]] != 205

# fit the railing line y = m*x + c from wall pixels in the band
ys, xs = np.where(FF[FIT_Y0:FIT_Y1, BAR_X0:BAR_X1] < 100)
ys = ys + FIT_Y0
xs = xs + BAR_X0
for _ in range(3):  # trim outliers, refit
    mfit, cfit = np.polyfit(xs, ys, 1)
    resid = np.abs(ys - (mfit * xs + cfit))
    keep = resid < max(3.0, np.percentile(resid, 80))
    xs, ys = xs[keep], ys[keep]
print(f'railing fit: y = {mfit:.4f}x + {cfit:.1f} from {len(xs)} wall px')

changed_wall = changed_unknown = 0
for x in range(BAR_X0, BAR_X1 + 1):
    yline = int(round(mfit * x + cfit))
    # 3 px solid barrier on the railing line
    for y in range(yline - 1, yline + 2):
        if FF[y, x] != 0:
            FF[y, x] = 0
            changed_wall += 1
    # false free below the barrier becomes unknown until B's data starts
    for y in range(yline + 2, yline + BAND_DEPTH):
        if bknown[y, x]:
            break
        if FF[y, x] > 240:
            FF[y, x] = 205
            changed_unknown += 1
print(f'barrier wall px painted: {changed_wall}, '
      f'false-free px -> unknown: {changed_unknown}')

with open(FF_PATH, 'wb') as f:
    f.write(b'P5\n%d %d\n255\n' % (FF.shape[1], FF.shape[0]))
    f.write(FF.tobytes())
print('first_floor.pgm updated (yaml unchanged)')

# connectivity check: the stair band must not be reachable from the corridors
from scipy import ndimage
free = FF > 240
labels, _ = ndimage.label(free)
corridor = labels[345, 600]            # north corridor, east of the junction
passage = labels[400, 560]             # inside the passage
band_probe = labels[int(mfit * 380 + cfit) + 20, 380]  # below barrier @x=380
print(f'corridor comp {corridor}, passage comp {passage}, '
      f'band probe comp {band_probe} '
      f'({"SEALED" if band_probe != corridor else "STILL CONNECTED — CHECK"})')
print('corridor<->passage connected:', corridor == passage)
