"""
tag_spawner.py  --  Webots supervisor controller that spawns the 24 arena
AprilTags at runtime using the AprilTag PROTO.

REQUIREMENTS
  - The Robot node running this controller must have its `supervisor` field TRUE.
  - AprilTag.proto must be in <project>/protos/.
  - The PROTO must be declared in the .wbt header as an IMPORTABLE EXTERNPROTO,
    not a plain EXTERNPROTO, because it is imported at runtime:

        IMPORTABLE EXTERNPROTO "../protos/AprilTag.proto"

    (Or add it via the GUI: world settings -> Importable PROTO nodes.)

COORDINATE FRAME
  Arena centred at the origin, ENU (x east/right, y north/forward, z up).
  Inner wall faces at x = +/-1.0 and y = +/-1.0; floor at z = 0.
"""

import math
import os

from controller import Supervisor

# --- geometry constants ------------------------------------------------
HALF = 1.0       # inner wall face distance from arena centre (m)
Z = 0.10         # tag centre height: 100 mm tag flush with 150 mm wall -> spans 0.05..0.15
EPS = 0.005      # plate sits this far in front of the wall face

# Tags live next to this controller, in a 'tags' subfolder. Build an ABSOLUTE
# path so Webots resolves it the same no matter how the world references things.
TAGS_DIR = os.path.join(os.path.dirname(__file__), "images/")


def tag_pose(wall, d):
    """World pose (x, y, z, yaw) for a tag a distance d (m) along `wall`,
    measured from that wall's reference corner. yaw rotates the PROTO's +Y face
    to point into the arena."""
    if wall == "north":   # inner face y=+HALF, faces -y; d from the west corner
        return (-HALF + d, HALF - EPS, Z, math.pi)
    if wall == "east":    # x=+HALF, faces -x; d from the north corner
        return (HALF - EPS, HALF - d, Z, math.pi / 2)
    if wall == "south":   # y=-HALF, faces +y; d from the east corner
        return (HALF - d, -HALF + EPS, Z, 0.0)
    if wall == "west":    # x=-HALF, faces +x; d from the south corner
        return (-HALF + EPS, -HALF + d, Z, -math.pi / 2)
    raise ValueError(wall)


# tag id -> (wall, distance-from-corner in metres).
# Distances are PLACEHOLDERS (evenly spaced). Replace with the exact tag-centre
# distances from Appendix A, Figure 1, which are uneven to clear the nets.
def _even(start_id, wall, n=6, first=0.15, pitch=0.34):
    return {start_id + i: (wall, first + i * pitch) for i in range(n)}

TAGS = {}
TAGS.update(_even(0,  "north"))   # 0-5
TAGS.update(_even(6,  "east"))    # 6-11
TAGS.update(_even(12, "south"))   # 12-17
TAGS.update(_even(18, "west"))    # 18-23


def spawn_tags(supervisor):
    children = supervisor.getRoot().getField("children")
    for tag_id, (wall, d) in TAGS.items():
        x, y, z, yaw = tag_pose(wall, d)
        url = os.path.join(TAGS_DIR, f"tag36_11_{tag_id:05d}.png").replace("\\", "/")
        node_string = (
            f'DEF TAG_{tag_id} AprilTag {{ '
            f'name "tag_{tag_id}" '
            f'translation {x:.4f} {y:.4f} {z:.4f} '
            f'rotation 0 0 1 {yaw:.5f} '
            f'url [ "{url}" ] '
            f'}}'
        )
        children.importMFNodeFromString(-1, node_string)

    print(f"[tag_spawner] spawned {len(TAGS)} AprilTags") # can remove later


def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    spawn_tags(supervisor)
    while supervisor.step(timestep) != -1:
        pass


if __name__ == "__main__":
    main()