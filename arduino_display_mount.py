"""
Parametric mount: bolts to the back of a Waveshare 7-DSI-TOUCH-C (SKU 33259)
via its Raspberry Pi-pattern M2.5 holes, and carries an Arduino UNO R3 on
5 mm standoffs; two standoffs have locating pins, two take M3 screws.  Edit the PARAMETERS block and re-run to regenerate the STL.

    pip install trimesh manifold3d numpy
    python arduino_display_mount.py
"""
import numpy as np
import trimesh
from trimesh.creation import box, cylinder

# ---------------- PARAMETERS (mm) ----------------
PLATE_T        = 3.0            # base plate thickness
PLATE_MARGIN   = 5.0            # plate border beyond outermost hole
CORNER_R       = 4.0            # plate corner radius

# Display-side holes: Raspberry Pi pattern on the display back cover (M2.5)
DISP_HOLE_DX   = 58.0
DISP_HOLE_DY   = 49.0
DISP_HOLE_D    = 2.8            # M2.5 clearance
DISP_CSK_D     = 5.5            # recess for screw head
DISP_CSK_DEPTH = 1.5

# Arduino UNO R3 hole positions, origin = board corner (USB end at x=0)
ARDUINO_HOLES  = [(14.0, 2.5), (15.3, 50.7), (66.1, 7.6), (66.1, 35.5)]
ARDUINO_BOARD  = (68.6, 53.3)
ARDUINO_OFFSET = (0.0, 0.0)     # shift Arduino relative to plate centre

STANDOFF_H     = 5.0            # standoff height
STANDOFF_OD    = 6.5
STANDOFF_HOLE  = 2.5            # M3 thread-forming; use 3.2 for clearance
PIN_HOLES      = [0, 3]         # indices into ARDUINO_HOLES that get a locating pin
PIN_D          = 2.9            # Arduino holes are ~3.2 mm
PIN_H          = 3.0            # pin height above standoff (board is 1.6 mm thick)
# --------------------------------------------------


def rounded_plate(w, h, t, r):
    core = box(extents=(w - 2 * r, h, t))
    core2 = box(extents=(w, h - 2 * r, t))
    m = trimesh.boolean.union([core, core2], engine="manifold")
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = cylinder(radius=r, height=t, sections=48)
            c.apply_translation(((w / 2 - r) * sx, (h / 2 - r) * sy, 0))
            m = trimesh.boolean.union([m, c], engine="manifold")
    return m


def cyl(r, h, x, y, z0):
    c = cylinder(radius=r, height=h, sections=48)
    c.apply_translation((x, y, z0 + h / 2))
    return c


# Hole coordinates about the plate centre
disp_pts = [(sx * DISP_HOLE_DX / 2, sy * DISP_HOLE_DY / 2)
            for sx in (-1, 1) for sy in (-1, 1)]
ard_pts = [(x - ARDUINO_BOARD[0] / 2 + ARDUINO_OFFSET[0],
            y - ARDUINO_BOARD[1] / 2 + ARDUINO_OFFSET[1])
           for x, y in ARDUINO_HOLES]

all_pts = np.array(disp_pts + ard_pts)
W = (all_pts[:, 0].max() - all_pts[:, 0].min()) + 2 * (PLATE_MARGIN + STANDOFF_OD / 2)
H = (all_pts[:, 1].max() - all_pts[:, 1].min()) + 2 * (PLATE_MARGIN + STANDOFF_OD / 2)
cx = (all_pts[:, 0].max() + all_pts[:, 0].min()) / 2
cy = (all_pts[:, 1].max() + all_pts[:, 1].min()) / 2

plate = rounded_plate(W, H, PLATE_T, CORNER_R)
plate.apply_translation((cx, cy, PLATE_T / 2))

# Standoffs
solids = [plate]
for i, (x, y) in enumerate(ard_pts):
    solids.append(cyl(STANDOFF_OD / 2, STANDOFF_H, x, y, PLATE_T))
    if i in PIN_HOLES:
        solids.append(cyl(PIN_D / 2, PIN_H, x, y, PLATE_T + STANDOFF_H))  # flat-topped pin
body = trimesh.boolean.union(solids, engine="manifold")

# Holes
cuts = []
for x, y in disp_pts:
    cuts.append(cyl(DISP_HOLE_D / 2, PLATE_T + 2, x, y, -1))
    cuts.append(cyl(DISP_CSK_D / 2, DISP_CSK_DEPTH + 1, x, y, PLATE_T - DISP_CSK_DEPTH))
for i, (x, y) in enumerate(ard_pts):
    if i not in PIN_HOLES:
        cuts.append(cyl(STANDOFF_HOLE / 2, STANDOFF_H + PLATE_T + 2, x, y, -1))
body = trimesh.boolean.difference([body] + cuts, engine="manifold")

assert body.is_watertight, "mesh not watertight"
body.export("arduino_display_mount.stl")
print(f"plate {W:.1f} x {H:.1f} x {PLATE_T} mm, standoffs {STANDOFF_H} mm, "
      f"total height {PLATE_T + STANDOFF_H} mm, volume {body.volume/1000:.1f} cm^3")
