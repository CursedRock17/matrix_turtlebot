#!/usr/bin/env python3
"""Merge two map_server maps (PGM + YAML) into one larger map.

The two halves come from separate SLAM runs, so their map frames are
unrelated — you must tell the tool where map B's frame sits in map A's frame:

    ros2 run turtlebot4_custom_py merge_maps \
        maps/first_floor.yaml maps/second_half_building.yaml \
        --dx 31.2 --dy -3.4 --dtheta 1.5 -o maps/entire_building_merged

To find dx/dy/dtheta, pick one landmark visible in both maps (a doorway both
runs drove through works well), read its (x, y) off each map in RViz with
'Publish Point', and iterate: start with dtheta 0 and dx/dy from the landmark
difference, merge, look at the seam, adjust. Walls in the overlap should land
on top of each other — a doubled wall means the transform is off.

Where both maps know a cell, occupied wins over free (safer for navigation);
known always wins over unknown. Offline tool, runs anywhere with numpy.
"""
import argparse

import numpy as np
import yaml

from pathlib import Path

UNKNOWN = 205  # map_saver's trinary gray


def _read_pgm(path):
    """Binary (P5) PGM -> 2D uint8 array. Row 0 is the TOP of the map."""
    data = Path(path).read_bytes()
    # Header: P5, width, height, maxval — whitespace separated, # comments.
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b'P5':
        raise ValueError(f'{path}: not a binary (P5) PGM')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval != 255:
        raise ValueError(f'{path}: expected maxval 255, got {maxval}')
    pixels = np.frombuffer(data, dtype=np.uint8,
                           count=width * height, offset=pos + 1)
    return pixels.reshape(height, width)


class MapHalf:
    """One map_server map: image array + the yaml metadata that places it."""

    def __init__(self, yaml_path):
        self.yaml_path = Path(yaml_path)
        self.meta = yaml.safe_load(self.yaml_path.read_text())
        if self.meta.get('negate', 0):
            raise ValueError(f'{yaml_path}: negate != 0 is not supported')
        self.image = _read_pgm(self.yaml_path.parent / self.meta['image'])
        self.resolution = float(self.meta['resolution'])
        self.origin = (float(self.meta['origin'][0]),
                       float(self.meta['origin'][1]))

    def corners(self):
        """The four world-frame corners of the image, own frame."""
        h, w = self.image.shape
        x0, y0 = self.origin
        x1, y1 = x0 + w * self.resolution, y0 + h * self.resolution
        return [(x0, y0), (x1, y0), (x0, y1), (x1, y1)]


def merge(map_a, map_b, dx, dy, dtheta_deg):
    """Composite map_b (its frame posed at dx/dy/dtheta in A's frame) onto
    map_a's frame. Returns (image, origin) of the merged map."""
    res = map_a.resolution
    if abs(res - map_b.resolution) > 1e-9:
        raise ValueError('maps have different resolutions — remake one of them')

    theta = np.radians(dtheta_deg)
    cos_t, sin_t = np.cos(theta), np.sin(theta)

    # Canvas bounds: A's extent plus B's corners transformed into A's frame.
    xs, ys = zip(*map_a.corners())
    for (bx, by) in map_b.corners():
        xs += (dx + bx * cos_t - by * sin_t,)
        ys += (dy + bx * sin_t + by * cos_t,)
    min_x, min_y = min(xs), min(ys)
    width = int(np.ceil((max(xs) - min_x) / res))
    height = int(np.ceil((max(ys) - min_y) / res))

    canvas = np.full((height, width), UNKNOWN, dtype=np.uint8)

    # Paint A straight in (pure translation, same resolution).
    a_h, a_w = map_a.image.shape
    col = int(round((map_a.origin[0] - min_x) / res))
    # PGM row 0 is the top of the map, so the origin (bottom-left) lands on
    # the LAST row of the image block.
    row = height - int(round((map_a.origin[1] - min_y) / res)) - a_h
    canvas[row:row + a_h, col:col + a_w] = map_a.image

    # Sample B with the inverse transform: for every canvas cell, where in
    # B's image would it have come from?
    rows, cols = np.mgrid[0:height, 0:width]
    wx = min_x + (cols + 0.5) * res
    wy = min_y + (height - 1 - rows + 0.5) * res
    # Inverse of (rotate by theta, then translate by dx/dy)
    bx = (wx - dx) * cos_t + (wy - dy) * sin_t
    by = -(wx - dx) * sin_t + (wy - dy) * cos_t
    b_h, b_w = map_b.image.shape
    b_col = np.floor((bx - map_b.origin[0]) / res).astype(int)
    b_row = b_h - 1 - np.floor((by - map_b.origin[1]) / res).astype(int)
    inside = (b_col >= 0) & (b_col < b_w) & (b_row >= 0) & (b_row < b_h)

    sample = np.full((height, width), UNKNOWN, dtype=np.uint8)
    sample[inside] = map_b.image[b_row[inside], b_col[inside]]

    known_b = sample != UNKNOWN
    known_canvas = canvas != UNKNOWN
    # Known beats unknown; where both are known, darker (occupied) wins.
    canvas = np.where(known_b & ~known_canvas, sample, canvas)
    both = known_b & known_canvas
    canvas[both] = np.minimum(canvas[both], sample[both])

    return canvas, (min_x, min_y)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Merge two map_server maps into one larger map')
    parser.add_argument('map_a', help='base map yaml (its frame is kept)')
    parser.add_argument('map_b', help='map yaml to transform onto map_a')
    parser.add_argument('--dx', type=float, default=0.0,
                        help="x of map_b's origin frame in map_a's frame [m]")
    parser.add_argument('--dy', type=float, default=0.0,
                        help="y of map_b's origin frame in map_a's frame [m]")
    parser.add_argument('--dtheta', type=float, default=0.0,
                        help="rotation of map_b's frame in map_a's [degrees]")
    parser.add_argument('-o', '--output', required=True,
                        help='output path prefix, e.g. maps/entire_building_merged')
    parsed = parser.parse_args(args)

    map_a = MapHalf(parsed.map_a)
    map_b = MapHalf(parsed.map_b)
    image, origin = merge(map_a, map_b, parsed.dx, parsed.dy, parsed.dtheta)

    out = Path(parsed.output)
    pgm_path = out.with_suffix('.pgm')
    header = f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'
    pgm_path.write_bytes(header.encode() + image.tobytes())

    meta = dict(map_a.meta)
    meta['image'] = pgm_path.name
    meta['origin'] = [round(origin[0], 3), round(origin[1], 3), 0]
    out.with_suffix('.yaml').write_text(
        ''.join(f'{key}: {value}\n' for key, value in meta.items()))

    h, w = image.shape
    print(f'Wrote {pgm_path} ({w}x{h} px, '
          f'{w * map_a.resolution:.1f}x{h * map_a.resolution:.1f} m) '
          f'and {out.with_suffix(".yaml")}')
    print('Open the PGM in an image viewer and check the seam: doubled '
          'walls in the overlap mean dx/dy/dtheta need adjusting. The poses '
          'in the merged map match the base map, so its locations file '
          'carries over; locations from the second map must be re-surveyed.')


if __name__ == '__main__':
    main()
