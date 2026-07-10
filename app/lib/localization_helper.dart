// lib/localization_helper.dart

import 'dart:math';
import 'package:vector_math/vector_math_64.dart';

class AprilTagDetection {
  final int tagId;
  final Vector3 t;
  final Matrix3 R;

  AprilTagDetection({required this.tagId, required this.t, required this.R});
}

class TagLocalizer {
  final Map<int, List<double>> tagWorld;
  final double camHeight;
  final double offFwd;
  final double offRight;
  late Matrix3 rWc;

  TagLocalizer({
    required this.tagWorld,
    this.camHeight = 0.16,
    double camPitchDown = 0.262,
    this.offFwd = 0.03,
    this.offRight = 0.0,
  }) {
    double c = cos(camPitchDown);
    double s = sin(camPitchDown);
    rWc = Matrix3.zero();
    rWc.setRow(0, Vector3(1.0, 0.0, 0.0));
    rWc.setRow(1, Vector3(0.0, -s, c));
    rWc.setRow(2, Vector3(0.0, -c, -s));
  }

  Vector3 tagInBody(Vector3 t) {
    return rWc.transformed(t);
  }

  /// Returns [X, Y, Yaw] derived purely from the visual matrix
  List<double>? robotPosition(List<AprilTagDetection> detections) {
    if (detections.isEmpty) return null;

    AprilTagDetection bestTag = detections.first;
    int tagId = bestTag.tagId;

    if (!tagWorld.containsKey(tagId)) return null;

    // 1. Calculate the exact Yaw using the tag's Rotation Matrix
    // The tag's Z-axis (normal vector facing out from the wall) in camera frame
    double nx = bestTag.R.row0.z; // R02
    double nz = bestTag.R.row2.z; // R22
    double relativeYaw = atan2(nx, nz);

    // Determine the absolute rotation of the camera when facing the tag head-on
    double tagAbsoluteYaw = 0.0;
    if (tagId >= 0 && tagId <= 5)        tagAbsoluteYaw = pi / 2;  // Looking at North wall, you face North (+pi/2)
    else if (tagId >= 6 && tagId <= 11)  tagAbsoluteYaw = 0.0;     // Looking at East wall, you face East (0.0)
    else if (tagId >= 12 && tagId <= 17) tagAbsoluteYaw = -pi / 2; // Looking at South wall, you face South (-pi/2)
    else if (tagId >= 18 && tagId <= 23) tagAbsoluteYaw = pi;      // Looking at West wall, you face West (pi)

    // Add the relative angle (how much you are turned away from straight-on)
    double yaw = tagAbsoluteYaw + relativeYaw;
    yaw = atan2(sin(yaw), cos(yaw)); // Normalize to (-pi, pi]

    // 2. Compute Absolute X, Y using the visual yaw
    double cy = cos(yaw);
    double sy = sin(yaw);

    List<double> xs = [];
    List<double> ys = [];

    for (var det in detections) {
      if (!tagWorld.containsKey(det.tagId)) continue;

      Vector3 bodyFrame = tagInBody(det.t);
      double right = bodyFrame.x;
      double forward = bodyFrame.y;

      List<double> tagPos = tagWorld[det.tagId]!;
      double xTag = tagPos[0];
      double yTag = tagPos[1];

      // Shift world coordinates back from the wall to the camera
      double camX = xTag - (forward * cy + right * sy);
      double camY = yTag - (forward * sy - right * cy);

      // Shift from the camera lens to the actual center of the chassis
      xs.add(camX - (offFwd * cy + offRight * sy));
      ys.add(camY - (offFwd * sy - offRight * cy));
    }

    if (xs.isEmpty) return null;

    double avgX = xs.reduce((a, b) => a + b) / xs.length;
    double avgY = ys.reduce((a, b) => a + b) / ys.length;

    return [avgX, avgY, yaw];
  }
}

Map<int, List<double>> buildTagWorld({double half = 1.0, double eps = 0.005}) {
  List<double> pos(String wall, double d) {
    switch (wall) {
      case "north": return [-half + d, half - eps];
      case "east":  return [half - eps, half - d];
      case "south": return [half - d, -half + eps];
      case "west":  return [-half + eps, -half + d];
      default: return [0.0, 0.0];
    }
  }

  Map<int, List<double>> table = {};
  for (int i = 0; i < 6; i++) { table[0 + i] = pos("north", 0.15 + i * 0.34); }
  for (int i = 0; i < 6; i++) { table[6 + i] = pos("east", 0.15 + i * 0.34); }
  for (int i = 0; i < 6; i++) { table[12 + i] = pos("south", 0.15 + i * 0.34); }
  for (int i = 0; i < 6; i++) { table[18 + i] = pos("west", 0.15 + i * 0.34); }

  return table;
}