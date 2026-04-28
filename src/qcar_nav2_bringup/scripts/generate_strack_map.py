#!/usr/bin/env python3
"""Generate an occupancy grid map (PGM + YAML) from the S-track geometry."""

# Only to be used if there is a custom slam map that needs to be generated for NAV2 to use. The map is generated from the track geometry parameters defined in this script, so if the track changes, this script should be updated and re-run to produce a new map.

import math
import struct
import os

import numpy as np


# Track parameters 
track_width = 0.60
R_center = 1.3
R_inner = R_center - track_width / 2  # 1.00
R_outer = R_center + track_width / 2  # 1.60
straight_len = 2.5
x_left = -1.25
x_right = 1.25
wall_thickness = 0.01

# Straight centerline Y positions
y1 = 0.0   # bottom
y2 = 2.6   # middle
y3 = 5.2   # top

# Turn centers
right_turn_cx, right_turn_cy = 1.25, 1.3
left_turn_cx, left_turn_cy = -1.25, 3.9


# Map parameters
resolution = 0.05  # m/pixel
padding = 1.0      # m around track extents

# Compute map bounds
x_min = x_left - R_outer - padding
x_max = x_right + R_outer + padding
y_min = y1 - 0.30 - padding
y_max = y3 + 0.30 + padding

origin_x = x_min
origin_y = y_min

width_pixels = int(math.ceil((x_max - x_min) / resolution))
height_pixels = int(math.ceil((y_max - y_min) / resolution))

print(f"Map size: {width_pixels} x {height_pixels} pixels")
print(f"World bounds: x=[{x_min:.2f}, {x_max:.2f}], y=[{y_min:.2f}, {y_max:.2f}]")
print(f"Origin: ({origin_x:.4f}, {origin_y:.4f})")

# Create map filled with free space (254)
grid = np.full((height_pixels, width_pixels), 254, dtype=np.uint8)

# Half-thickness for wall hit detection (inflated to cover SDF box-segment gaps)
half_wall = 0.04


# Helper: pixel center to world coordinates
def pixel_to_world(row, col):
    """Convert pixel (row, col) to world (x, y). Row 0 = bottom of image."""
    wx = origin_x + (col + 0.5) * resolution
    wy = origin_y + (row + 0.5) * resolution
    return wx, wy



# Draw straight walls
straights = [
    (-0.30, x_left, x_right),   # bottom outer
    (0.30, x_left, x_right),    # bottom inner
    (2.30, x_left, x_right),    # middle inner
    (2.90, x_left, x_right),    # middle outer
    (4.90, x_left, x_right),    # top inner
    (5.50, x_left, x_right),    # top outer
]

for wall_y, wx_start, wx_end in straights:
    # Find row range near this wall
    row_center = (wall_y - origin_y) / resolution
    row_lo = max(0, int(row_center - half_wall / resolution) - 1)
    row_hi = min(height_pixels, int(row_center + half_wall / resolution) + 2)

    col_start = max(0, int((wx_start - origin_x) / resolution))
    col_end = min(width_pixels, int((wx_end - origin_x) / resolution) + 1)

    for r in range(row_lo, row_hi):
        _, wy = pixel_to_world(r, 0)
        if abs(wy - wall_y) < half_wall:
            grid[r, col_start:col_end] = 0


# Draw arc walls
arcs = [
    (right_turn_cx, right_turn_cy, R_outer, -math.pi / 2, math.pi / 2),
    (right_turn_cx, right_turn_cy, R_inner, -math.pi / 2, math.pi / 2),
    (left_turn_cx, left_turn_cy, R_outer, math.pi / 2, 3 * math.pi / 2),
    (left_turn_cx, left_turn_cy, R_inner, math.pi / 2, 3 * math.pi / 2),
]

for cx, cy, radius, angle_start, angle_end in arcs:
    # Bounding box for this arc in pixels
    r_lo = max(0, int(((cy - radius - half_wall) - origin_y) / resolution) - 1)
    r_hi = min(height_pixels, int(((cy + radius + half_wall) - origin_y) / resolution) + 2)
    c_lo = max(0, int(((cx - radius - half_wall) - origin_x) / resolution) - 1)
    c_hi = min(width_pixels, int(((cx + radius + half_wall) - origin_x) / resolution) + 2)

    for r in range(r_lo, r_hi):
        for c in range(c_lo, c_hi):
            wx, wy = pixel_to_world(r, c)
            dx = wx - cx
            dy = wy - cy
            dist = math.sqrt(dx * dx + dy * dy)

            if abs(dist - radius) < half_wall:
                angle = math.atan2(dy, dx)
                # Normalize angle to [angle_start, angle_start + 2*pi)
                while angle < angle_start:
                    angle += 2 * math.pi
                while angle >= angle_start + 2 * math.pi:
                    angle -= 2 * math.pi
                if angle <= angle_end:
                    grid[r, c] = 0


# Save PGM (P5 binary format)
# Row 0 in the image file = top of map = highest Y in world coords
# So we flip vertically.

pgm_path = os.path.join(os.getcwd(), "strack_map.pgm")
yaml_path = os.path.join(os.getcwd(), "strack_map.yaml")

flipped = np.flipud(grid)

with open(pgm_path, "wb") as f:
    header = f"P5\n{width_pixels} {height_pixels}\n255\n"
    f.write(header.encode("ascii"))
    f.write(flipped.tobytes())

print(f"Wrote {pgm_path}")


# Save YAML
yaml_content = f"""\
image: strack_map.pgm
mode: trinary
resolution: {resolution}
origin: [{origin_x:.4f}, {origin_y:.4f}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""

with open(yaml_path, "w") as f:
    f.write(yaml_content)

print(f"Wrote {yaml_path}")
print("Done.")
