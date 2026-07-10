"""
ground_projector.py
===================
Convert a detected ball's bounding box into a 2D Cartesian position on the
ground plane, relative to the robot, using the known camera geometry.

This is the analytic version of the ground-plane homography. In simulation the
camera intrinsics (from field of view and resolution) and extrinsics (mount
height and downward pitch) are known exactly, so no click calibration is
needed. The homography is built directly from those numbers.

METHOD
    Each image pixel defines a ray out of the camera. The ball touches the
    floor, so the pixel at the bottom of the ball's circle is the ground-contact
    point. We cast that ray, intersect it with the ground plane (Z = 0), and
    read off the (X, Y) hit point. That is the ball position from a single
    frame, with no assumption about the ball's true diameter.

OUTPUT FRAME (ground plane, top-down, origin under the camera)
    +X = robot's right
    +Y = forward (direction the camera faces)
    units = metres (same units as `cam_height`)
    To use a (forward, left) convention instead, return (Y, -X) from the caller.

ASSUMPTIONS
    Pinhole camera with square pixels and the principal point at the image
    centre (the Webots camera model). Camera roll and yaw are zero, that is the
    optical axis lies in the robot's forward-vertical plane and is pitched only
    downward. Constant, known mount height and pitch.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class GroundProjector:
    """Projects image pixels onto the ground plane using known camera geometry.

    Parameters
    ----------
    fov_h_rad   : horizontal field of view in radians (Webots: camera.getFov()).
    width       : image width in pixels  (Webots: camera.getWidth()).
    height      : image height in pixels (Webots: camera.getHeight()).
    cam_height  : camera height above the ground, in metres.
    pitch_down_rad : downward pitch of the optical axis from horizontal, in
                     radians. Positive means looking down.
    """

    fov_h_rad: float
    width: int
    height: int
    cam_height: float
    pitch_down_rad: float

    def __post_init__(self):
        # --- Intrinsics (Webots pinhole: square pixels, centre principal point)
        self.fx = (self.width / 2.0) / np.tan(self.fov_h_rad / 2.0)
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        self.K = np.array([[self.fx, 0, self.cx],
                           [0, self.fy, self.cy],
                           [0, 0, 1.0]])

        # --- Camera-to-world rotation R_wc -------------------------------
        # Camera frame (OpenCV): x right, y down, z forward (optical axis).
        # World frame: X right, Y forward, Z up. Origin on the ground under the
        # camera. At zero pitch the optical axis is horizontal (+Y). Pitching
        # the camera down by theta rotates the optical axis toward the ground.
        c, s = np.cos(self.pitch_down_rad), np.sin(self.pitch_down_rad)
        self.R_wc = np.array([[1.0, 0.0, 0.0],
                              [0.0, -s,  c],
                              [0.0, -c, -s]])
        self.R_cw = self.R_wc.T
        self.C = np.array([0.0, 0.0, self.cam_height])  # camera position in world

        # --- Ground-plane homography (pixel -> ground), for reference ----
        # For points on Z = 0: pixel ~ K [r1 r2 t] [X Y 1]^T, with the world->cam
        # rotation columns r1, r2 and translation t = -R_cw C.
        t = -self.R_cw @ self.C
        H_ground_to_pixel = self.K @ np.column_stack(
            (self.R_cw[:, 0], self.R_cw[:, 1], t))
        self.H_pixel_to_ground = np.linalg.inv(H_ground_to_pixel)

        # Image row of the horizon. Rays at or above this never hit the ground.
        self._horizon_v = self._compute_horizon_row()

    # ------------------------------------------------------------------
    # Core projection
    # ------------------------------------------------------------------
    def pixel_to_ground(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        """Map an image pixel on the floor to (X, Y) in metres, or None.

        Returns None when the pixel is at or above the horizon, i.e. its ray
        points up or parallel and never meets the ground.
        """
        # Ray direction in the camera frame, then rotated into the world frame.
        d_cam = np.array([(u - self.cx) / self.fx,
                          (v - self.cy) / self.fy,
                          1.0])
        d_world = self.R_wc @ d_cam

        if d_world[2] >= -1e-9:        # not pointing downward -> no ground hit
            return None
        t = -self.C[2] / d_world[2]    # solve C_z + t * d_z = 0
        ground = self.C + t * d_world
        return float(ground[0]), float(ground[1])

    # ------------------------------------------------------------------
    # Ball-level API
    # ------------------------------------------------------------------
    def ball_ground_position(self, bbox) -> Optional[Tuple[float, float]]:
        """Ground (X, Y) of a ball from its detection.

        Uses the ground-contact pixel = bottom of the detected circle
        (cx, cy + radius). Accepts any object exposing cx, cy, radius.
        """
        u = bbox.cx
        v = bbox.cy + bbox.radius
        return self.pixel_to_ground(u, v)

    def locate_balls(self, bboxes: Sequence) -> list:
        """Return a list of (bbox, (X, Y) or None) for a set of detections."""
        return [(b, self.ball_ground_position(b)) for b in bboxes]

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _compute_horizon_row(self) -> float:
        """Image v-coordinate where the ground meets the horizon (informational)."""
        # The horizon is where a horizontal forward ray projects. Take a far
        # ground point and read its row; fall back to the analytic limit.
        far = self.pixel_to_ground(self.cx, self.height - 1)
        if far is None:
            return 0.0
        # Pitch sets the horizon at angle pitch above image centre.
        return self.cy - self.fy * np.tan(self.pitch_down_rad)

    def forward_project(self, X: float, Y: float) -> Optional[Tuple[float, float]]:
        """Ground (X, Y) -> pixel (u, v). Inverse of pixel_to_ground; for tests."""
        P_world = np.array([X, Y, 0.0])
        P_cam = self.R_cw @ (P_world - self.C)
        if P_cam[2] <= 1e-9:           # behind the camera
            return None
        u = self.cx + self.fx * P_cam[0] / P_cam[2]
        v = self.cy + self.fy * P_cam[1] / P_cam[2]
        return float(u), float(v)


# ----------------------------------------------------------------------
# Self-test: round-trip and a few sanity values (no camera needed)
# ----------------------------------------------------------------------
if __name__ == "__main__":
    proj = GroundProjector(
        fov_h_rad=1.047,     # ~60 deg; replace with camera.getFov()
        width=640, height=480,
        cam_height=0.06,
        pitch_down_rad=0.262,
    )

    print(f"fx=fy={proj.fx:.1f}px  principal=({proj.cx:.0f},{proj.cy:.0f})")
    centre = proj.pixel_to_ground(proj.cx, proj.cy)
    print(f"image-centre ground hit: X={centre[0]:+.3f} Y={centre[1]:+.3f} m")
    print(f"horizon row ~ v={proj._horizon_v:.1f}")

    # Round-trip: known ground points -> pixel -> back to ground.
    test_pts = [(0.0, 0.30), (0.10, 0.50), (-0.20, 0.80), (0.05, 0.224)]
    print("\nround-trip (ground -> pixel -> ground):")
    max_err = 0.0
    for X, Y in test_pts:
        px = proj.forward_project(X, Y)
        if px is None:
            print(f"  ({X:+.2f},{Y:+.2f}) not visible")
            continue
        back = proj.pixel_to_ground(*px)
        err = np.hypot(back[0] - X, back[1] - Y)
        max_err = max(max_err, err)
        print(f"  ({X:+.2f},{Y:+.2f}) -> px({px[0]:6.1f},{px[1]:6.1f}) "
              f"-> ({back[0]:+.3f},{back[1]:+.3f})  err={err:.2e}")
    print(f"max round-trip error: {max_err:.2e} m")

    # Above-horizon pixel must return None.
    top = proj.pixel_to_ground(proj.cx, 5)
    print(f"\ntop-of-image pixel maps to: {top}  (expected None)")

    # Mock a BBox to exercise the ball API.
    class _B:
        cx, cy, radius = 320, 360, 24
    print("ball_ground_position(mock bbox):", proj.ball_ground_position(_B()))