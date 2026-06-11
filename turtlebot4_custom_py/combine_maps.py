import yaml
import numpy as np
from PIL import Image


def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    img = Image.open(data['image'])
    img = np.array(img)

    resolution = data['resolution']
    origin = data['origin']  # [x, y, theta]

    return img, resolution, origin


def merge_maps(map1_yaml, map2_yaml, output_prefix):
    img1, res1, origin1 = load_map(map1_yaml)
    img2, res2, origin2 = load_map(map2_yaml)

    assert res1 == res2, "Resolutions must match"

    # Convert origins to pixel offsets
    def origin_to_pixels(origin, resolution):
        return int(origin[0] / resolution), int(origin[1] / resolution)

    x1, y1 = origin_to_pixels(origin1, res1)
    x2, y2 = origin_to_pixels(origin2, res2)

    # Determine bounds
    min_x = min(x1, x2)
    min_y = min(y1, y2)

    max_x = max(x1 + img1.shape[1], x2 + img2.shape[1])
    max_y = max(y1 + img1.shape[0], y2 + img2.shape[0])

    width = max_x - min_x
    height = max_y - min_y

    merged = np.full((height, width), 205, dtype=np.uint8)  # unknown space

    def paste(img, x, y):
        px = x - min_x
        py = y - min_y

        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                val = img[i, j]
                if val < 250:  # prefer known over unknown
                    merged[py + i, px + j] = val

    paste(img1, x1, y1)
    paste(img2, x2, y2)

    # Save image
    Image.fromarray(merged).save(output_prefix + ".pgm")

    # Save YAML
    new_origin = [min_x * res1, min_y * res1, 0.0]

    yaml_data = {
        'image': output_prefix + ".pgm",
        'resolution': res1,
        'origin': new_origin,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    with open(output_prefix + ".yaml", 'w') as f:
        yaml.dump(yaml_data, f)

    print(f"Merged map saved as {output_prefix}.pgm/.yaml")


# Example usage
merge_maps("map1.yaml", "map2.yaml", "merged_map")
