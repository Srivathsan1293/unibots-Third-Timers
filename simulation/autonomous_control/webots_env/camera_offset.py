"""
calibrate_camera_offset.py
Recover the camera mount offset (forward, right) so TagLocalizer reports the
robot origin instead of the camera position. Two ways:

  measure_camera_offset(...)  -- exact, reads the camera and robot nodes from
                                 the running simulation. Use this in Webots.
  offset_from_residual(...)   -- empirical, derives the offset from one known
                                 pose by comparing the (uncalibrated) tag fix to
                                 the true robot position. Works on real hardware
                                 too, where you cannot read the scene tree.

Plug the result into TagLocalizer(cam_offset_forward=..., cam_offset_right=...).
"""

import math

import numpy as np


def offset_from_residual(predicted_xy, actual_xy, yaw):
    """Empirical recalibration from one known pose.

    Run TagLocalizer with the offsets at 0 (so it returns the CAMERA position),
    capture one frame where the true robot pose is known, and pass:
      predicted_xy : the localizer's (x, y) for that frame
      actual_xy    : the true robot (x, y) (e.g. supervisor translation)
      yaw          : the robot heading at that frame
    Returns (cam_offset_forward, cam_offset_right) to set on the localizer.
    """
    dx = predicted_xy[0] - actual_xy[0]
    dy = predicted_xy[1] - actual_xy[1]
    c, s = math.cos(yaw), math.sin(yaw)
    off_fwd = dx * c + dy * s          # body-frame components of the residual
    off_right = dx * s - dy * c
    return off_fwd, off_right


def measure_camera_offset(supervisor, camera_def="CAM", forward_axis=0):
    """Exact recalibration from the simulation, using the camera node's DEF name.
 
    Avoids getFromDevice (which is version-fragile). Give your Camera node a DEF
    name in the scene tree, e.g. `DEF CAM Camera { ... }`, and pass it here.
 
    forward_axis : which local axis of the robot is "forward" (same convention
                   as robot_pose_from_node; 0 = local +x for most TurtleBots).
    Returns (cam_offset_forward, cam_offset_right).
    """
    cam_node = supervisor.getFromDef(camera_def)
    if cam_node is None:
        raise RuntimeError(
            f"No node with DEF '{camera_def}'. Add `DEF {camera_def}` to your "
            f"Camera node in the scene tree, or pass the correct DEF name.")
    robot_node = supervisor.getSelf()
    cam_w = np.array(cam_node.getPosition())
    rob_w = np.array(robot_node.getPosition())
    R = np.array(robot_node.getOrientation()).reshape(3, 3)   # robot local -> world
 
    delta_local = R.T @ (cam_w - rob_w)        # camera offset in the robot frame
    fwd_unit = np.eye(3)[forward_axis]
    up_unit = np.array([0.0, 0.0, 1.0])        # robot local up (ENU)
    right_unit = np.cross(fwd_unit, up_unit)
 
    off_fwd = float(delta_local @ fwd_unit)
    off_right = float(delta_local @ right_unit)
    return off_fwd, off_right