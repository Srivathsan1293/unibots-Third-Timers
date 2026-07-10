import math

import cv2
import numpy as np
from pupil_apriltags import Detector


class AprilTagDetector:
    def __init__(self, fov=0.785):
        self.detector = Detector(families="tag36h11", quad_decimate=1.0)  # 1.0, not 2.0, so small/far tags still decode
        self.fov = fov

    def detect(self, frame: np.ndarray) -> list[int]:
        H, W = frame.shape[:2]

        fx = (W / 2) / math.tan(self.fov / 2)
        fy = fx
        cx = W / 2
        cy = H / 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dets = self.detector.detect(gray, estimate_tag_pose=True,
                            camera_params=(fx, fy, cx, cy), tag_size=0.0781)  # 100 mm
        
        # IMPORTANT
        # tag size is the size of the black tag as it appears on the plate and not the plate size
        # refere to conversation for more details
        # we can measure tag_size by a calibration function (not shown here)

        dets_processed = []

        for d in dets:
            tag_id = d.tag_id      # 0..23
            R = d.pose_R           # 3x3, tag orientation in the camera frame (all entries should be close to 0)
            t = d.pose_t

            dets_processed.append([tag_id, R, t])

        return dets_processed  

class TagLocalizer:
    def __init__(self, tag_world, cam_pitch_down=0.262,
                 cam_height=0.06, cam_offset_forward=0.0, cam_offset_right=0.0):
        """
        tag_world : {tag_id: (x, y)} world positions of the tag centres. MUST
                    match where the spawner actually places them.
        cam_pitch_down, cam_offset_* : the same camera mounting you use elsewhere.
        """
        self.tag_world = tag_world
        self.cam_height = cam_height
        self.off_fwd = cam_offset_forward
        self.off_right = cam_offset_right
        # camera (x right, y down, z forward) -> body (x right, y forward, z up)
        c, s = math.cos(cam_pitch_down), math.sin(cam_pitch_down)
        self.R_wc = np.array([[1.0, 0.0, 0.0],
                              [0.0, -s,  c],
                              [0.0, -c, -s]])
 
    def tag_in_body(self, t):
        """Tag position relative to the camera, in the robot ground frame:
        returns (right, forward, up)."""
        right, forward, up = self.R_wc @ np.asarray(t, dtype=float).reshape(3)
        return right, forward, up
 
    def robot_pose(self, detections):
        """Estimate full robot pose (x, y, yaw) from tags ALONE, no odometry.
 
        Needs at least two tags visible in the frame. Uses only the
        translations (pose_R is ignored), via a 2D rigid registration of the
        tags' body-frame positions onto their known world positions.
 
        detections : list of (tag_id, R, t).
        Returns (x, y, yaw) or None if fewer than two usable tags.
        """
        B, W = [], []
        for tag_id, _R, t in detections:
            if tag_id not in self.tag_world:
                continue
            right, forward, _up = self.tag_in_body(t)
            B.append((right, forward))
            W.append(self.tag_world[tag_id])
        if len(B) < 2:
            return None
 
        B = np.asarray(B, dtype=float)
        W = np.asarray(W, dtype=float)
        bbar, wbar = B.mean(axis=0), W.mean(axis=0)
        Bc, Wc = B - bbar, W - wbar
 
        dot = float(np.sum(Bc[:, 0] * Wc[:, 0] + Bc[:, 1] * Wc[:, 1]))
        crs = float(np.sum(Bc[:, 0] * Wc[:, 1] - Bc[:, 1] * Wc[:, 0]))
        phi = math.atan2(crs, dot)                       # rotation aligning body -> world
        c, s = math.cos(phi), math.sin(phi)
        Rphi = np.array([[c, -s], [s, c]])
        p = wbar - Rphi @ bbar                           # camera ground position
 
        yaw = math.atan2(math.sin(phi + math.pi / 2),    # wrap to (-pi, pi]
                         math.cos(phi + math.pi / 2))
        cy, sy = math.cos(yaw), math.sin(yaw)
        rob_x = p[0] - (self.off_fwd * cy + self.off_right * sy)
        rob_y = p[1] - (self.off_fwd * sy - self.off_right * cy)
        return rob_x, rob_y, yaw
 
    def robot_position(self, detections, yaw, debug=False):
        """Estimate robot (x, y) in world from one or more tags + known heading.
 
        Works with a SINGLE tag. Returns (x, y) averaged over usable tags, or
        None when none are usable. With debug=True it prints why a frame yields
        nothing, which is almost always one of:
          - no tags were detected at all (camera not framing the wall tags, or
            they are not decoding),
          - a detected id is not in tag_world (id/table mismatch),
          - a detection has no translation (pose estimation failed -> t is None).
 
        detections : list of (tag_id, R, t); R is ignored.
        yaw        : robot heading in radians (from the IMU).
        """
        cy, sy = math.cos(yaw), math.sin(yaw)
        xs, ys = [], []
        n_seen = 0

        for det in detections:
            n_seen += 1
            tag_id, _R, t = det[0], det[1], det[2]

            if tag_id not in self.tag_world:
                if debug:
                    print(f"[loc] tag {tag_id} not in tag_world "
                          f"(known ids: {sorted(self.tag_world)})")
                continue

            if t is None:
                if debug:
                    print(f"[loc] tag {tag_id}: no translation (pose estimate failed)")
                continue

            right, forward, _up = self.tag_in_body(t)
            X_tag, Y_tag = self.tag_world[tag_id]
            # Invert body_to_world for the camera's ground point:
            #   X_tag = cam_x + forward*cos(yaw) + right*sin(yaw)
            #   Y_tag = cam_y + forward*sin(yaw) - right*cos(yaw)
            cam_x = X_tag - (forward * cy + right * sy)
            cam_y = Y_tag - (forward * sy - right * cy)
            # Shift from the camera ground point back to the robot origin.
            xs.append(cam_x - (self.off_fwd * cy + self.off_right * sy))
            ys.append(cam_y - (self.off_fwd * sy - self.off_right * cy))

        if not xs:
            if debug and n_seen == 0:
                print("[loc] no AprilTags detected this frame")
            return None
    
        return (np.mean(xs).item(), np.mean(ys).item())
 
 
# ----------------------------------------------------------------------
# Build the tag world table. These MUST match your spawner's placement.
# (Same tag_pose as the spawner, positions only; replace the placeholder
# distances with the real Appendix A centres you used.)
# ----------------------------------------------------------------------
def build_tag_world(half=1.0, eps=0.005):
    def pos(wall, d):
        if wall == "north": return (-half + d, half - eps)
        if wall == "east":  return (half - eps, half - d)
        if wall == "south": return (half - d, -half + eps)
        if wall == "west":  return (-half + eps, -half + d)
 
    def even(start_id, wall, n=6, first=0.15, pitch=0.34):
        return {start_id + i: pos(wall, first + i * pitch) for i in range(n)}
 
    table = {}
    table.update(even(0, "north"))
    table.update(even(6, "east"))
    table.update(even(12, "south"))
    table.update(even(18, "west"))
    return table
