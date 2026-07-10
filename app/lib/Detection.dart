// lib/Detection.dart

import 'dart:async';
import 'dart:io';
import 'dart:isolate';
import 'dart:math';
import 'dart:typed_data';

import 'package:camera/camera.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:tflite_flutter/tflite_flutter.dart';

import 'serial_helper.dart';
import 'localization_helper.dart';
import 'apriltag_ffi.dart';

// ─────────────────────────────────────────────
// Core Data Models
// ─────────────────────────────────────────────
enum BallType { orange }
enum NavDirection { left, right, forward, standby }
enum SettingsCategory { none, orange, localization, distance, direction }

class Rect2D {
  final int x, y, w, h;
  const Rect2D(this.x, this.y, this.w, this.h);
}

class BallDetection {
  final double absX1, absY1, absX2, absY2;
  final double confidence;
  final double distanceCm;
  final bool isYolo;
  final BallType type;
  final double? worldX;
  final double? worldY;

  const BallDetection({
    required this.absX1, required this.absY1,
    required this.absX2, required this.absY2,
    required this.confidence, required this.distanceCm,
    required this.isYolo, required this.type,
    this.worldX, this.worldY,
  });
}

class TrackedBall {
  final int id;
  double x;
  double y;
  final BallType type;
  int hits = 1;
  int missedVisible = 0;
  bool confirmed = false;

  TrackedBall({required this.id, required this.x, required this.y, required this.type});
}

class DetectionCluster {
  final List<BallDetection> balls;
  double get centerX => balls.map((b) => (b.absX1 + b.absX2) / 2).reduce((a, b) => a + b) / balls.length;
  double get minDistance => balls.map((b) => b.distanceCm).reduce(min);
  DetectionCluster(this.balls);
}

class FrameResult {
  final List<BallDetection> detections;
  final Uint8List? debugImage;
  final double? robotX;
  final double? robotY;
  final double? robotYaw;
  final int tagCount;
  final double ceilingCutoffY;
  final bool collisionRisk;
  FrameResult(this.detections, this.debugImage, this.robotX, this.robotY, this.robotYaw, this.tagCount, this.ceilingCutoffY, this.collisionRisk);
}

class NavigationOutput {
  final NavDirection direction;
  final double distanceCm;
  final DetectionCluster? targetCluster;
  final List<BallDetection> allDetections;
  final List<TrackedBall> mapBalls;
  final Uint8List? debugImage;
  final double? robotX;
  final double? robotY;
  final double? robotYaw;
  final int tagCount;
  final double ceilingCutoffY;
  final TrackedBall? activeMapTarget;
  final double targetHeadingDelta;
  final bool collisionRisk;

  NavigationOutput({
    required this.direction, required this.distanceCm,
    this.targetCluster, required this.allDetections, required this.mapBalls,
    this.debugImage, this.robotX, this.robotY, this.robotYaw,
    required this.tagCount,
    required this.ceilingCutoffY, this.activeMapTarget, required this.targetHeadingDelta,
    required this.collisionRisk,
  });
}

class FrameRequest {
  final Uint8List yPlane, uPlane, vPlane;
  final int width, height, yRowStride, uvRowStride, uvPixelStride;

  // Settings State
  final SettingsCategory debugCategory;

  // Orange Tuning
  final int orangeRMin, orangeRGDiff, orangeRBDiff;
  final int orangeMinArea, orangeMaxArea;
  final double orangeMinAspect, orangeMaxAspect, orangeMinFill;

  // Localization Tuning
  final double camPitchDown, camHeight, offFwd, offRight;

  // Distance/Camera Tuning
  final double focalLengthPx, orangeDiameterCm, tagSizeCm;
  final double yoloConfThreshold;

  // State integration
  final double? lastKnownX, lastKnownY, lastKnownYaw;

  FrameRequest({
    required this.yPlane, required this.uPlane, required this.vPlane,
    required this.width, required this.height,
    required this.yRowStride, required this.uvRowStride, required this.uvPixelStride,
    required this.debugCategory,
    required this.orangeRMin, required this.orangeRGDiff, required this.orangeRBDiff,
    required this.orangeMinArea, required this.orangeMaxArea,
    required this.orangeMinAspect, required this.orangeMaxAspect, required this.orangeMinFill,
    required this.camPitchDown, required this.camHeight, required this.offFwd, required this.offRight,
    required this.focalLengthPx, required this.orangeDiameterCm,
    required this.tagSizeCm, required this.yoloConfThreshold,
    this.lastKnownX, this.lastKnownY, this.lastKnownYaw,
  });
}

// ─────────────────────────────────────────────
// Primary System Controller
// ─────────────────────────────────────────────
class DetectionSystem {
  Isolate? _isolate;
  SendPort? _sendPort;
  final ReceivePort _receivePort = ReceivePort();
  bool _isProcessing = false;

  SettingsCategory activeCategory = SettingsCategory.none;

  // Default parameters (will be overwritten by main.dart on load)
  int orangeRMin = 130, orangeRGDiff = 30, orangeRBDiff = 30;
  int orangeMinArea = 24, orangeMaxArea = 5000;
  double orangeMinAspect = 0.4, orangeMaxAspect = 2.2, orangeMinFill = 0.25;

  double camPitchDown = 0.349066, camHeight = 0.179, offFwd = 0.03, offRight = 0.0;
  double focalLengthPx = 615.0, orangeDiameterCm = 4.0, tagSizeCm = 7.81;
  double yoloConfThreshold = 0.50;

  double navDeadzoneLeft = 0.20;
  double navDeadzoneRight = -0.20;

  int _missedFrames = 0;
  final int _maxCoastFrames = 6;
  NavDirection _coastingDirection = NavDirection.standby;
  NavDirection _pendingDirection = NavDirection.standby;
  int _stableFrames = 0;
  final int _requiredStableFrames = 3;

  double? _lastKnownX, _lastKnownY, _lastKnownYaw;

  List<TrackedBall> _trackedBalls = [];
  int _nextTrackId = 0;
  final double _assocGate = 0.12, _emaAlpha = 0.35;
  final int _confirmHits = 3, _maxMissed = 8;
  final double _collectionRadius = 0.05, _cameraFovHalfRad = 0.523;

  final void Function(NavigationOutput)? onFrameProcessed;
  final SerialHelper? serialHelper;
  NavDirection _lastSentDirection = NavDirection.standby;

  DetectionSystem({this.onFrameProcessed, this.serialHelper}) {
    // Override local predictive state with Authoritative FUSED State from the Pi FSM
    serialHelper?.onFusedPositionData = (double fx, double fy, double fyaw) {
      _lastKnownX = fx;
      _lastKnownY = fy;
      _lastKnownYaw = fyaw;
    };
  }

  Future<void> start() async {
    RootIsolateToken rootIsolateToken = RootIsolateToken.instance!;
    _isolate = await Isolate.spawn(_isolateEntry, [_receivePort.sendPort, rootIsolateToken]);

    _receivePort.listen((message) {
      if (message is SendPort) {
        _sendPort = message;
      } else if (message is FrameResult) {
        _isProcessing = false;
        _calculateNavigation(message);
      }
    });
  }

  void processFrame(CameraImage image) {
    if (_isProcessing || _sendPort == null) return;
    _isProcessing = true;

    final req = FrameRequest(
      yPlane: image.planes[0].bytes, uPlane: image.planes[1].bytes, vPlane: image.planes[2].bytes,
      width: image.width, height: image.height,
      yRowStride: image.planes[0].bytesPerRow, uvRowStride: image.planes[1].bytesPerRow, uvPixelStride: image.planes[1].bytesPerPixel!,
      debugCategory: activeCategory,
      orangeRMin: orangeRMin, orangeRGDiff: orangeRGDiff, orangeRBDiff: orangeRBDiff,
      orangeMinArea: orangeMinArea, orangeMaxArea: orangeMaxArea,
      orangeMinAspect: orangeMinAspect, orangeMaxAspect: orangeMaxAspect, orangeMinFill: orangeMinFill,
      camPitchDown: camPitchDown, camHeight: camHeight, offFwd: offFwd, offRight: offRight,
      focalLengthPx: focalLengthPx, orangeDiameterCm: orangeDiameterCm,
      tagSizeCm: tagSizeCm, yoloConfThreshold: yoloConfThreshold,
      lastKnownX: _lastKnownX, lastKnownY: _lastKnownY, lastKnownYaw: _lastKnownYaw,
    );
    _sendPort!.send(req);
  }

  void _calculateNavigation(FrameResult result) {
    if (result.robotX != null) _lastKnownX = result.robotX;
    if (result.robotY != null) _lastKnownY = result.robotY;
    if (result.robotYaw != null) _lastKnownYaw = result.robotYaw;

    // 1. TRACKER UPDATE LOGIC
    if (_lastKnownX != null && _lastKnownY != null && _lastKnownYaw != null) {
      final double rX = _lastKnownX!;
      final double rY = _lastKnownY!;
      final double rYaw = _lastKnownYaw!;

      List<BallDetection> validWorldDets = result.detections
          .where((d) => d.worldX != null && d.worldY != null).toList();

      Set<int> matchedDetIndices = {}, matchedTrackIds = {};

      for (int i = 0; i < validWorldDets.length; i++) {
        var det = validWorldDets[i];
        TrackedBall? bestMatch;
        double bestDist = double.infinity;

        for (var t in _trackedBalls) {
          if (t.type != det.type) continue;
          double dist = sqrt(pow(det.worldX! - t.x, 2) + pow(det.worldY! - t.y, 2));
          if (dist < _assocGate && dist < bestDist) { bestDist = dist; bestMatch = t; }
        }

        if (bestMatch != null && !matchedTrackIds.contains(bestMatch.id)) {
          matchedDetIndices.add(i);
          matchedTrackIds.add(bestMatch.id);
          bestMatch.x = (1 - _emaAlpha) * bestMatch.x + _emaAlpha * det.worldX!;
          bestMatch.y = (1 - _emaAlpha) * bestMatch.y + _emaAlpha * det.worldY!;
          bestMatch.hits++;
          bestMatch.missedVisible = 0;
          if (bestMatch.hits >= _confirmHits) bestMatch.confirmed = true;
        }
      }

      List<TrackedBall> survivors = [];
      for (var t in _trackedBalls) {
        if (matchedTrackIds.contains(t.id)) { survivors.add(t); continue; }

        double distToRobot = sqrt(pow(t.x - rX, 2) + pow(t.y - rY, 2));
        if (distToRobot <= _collectionRadius) continue;

        double dx = t.x - rX, dy = t.y - rY;
        double angleDiff = (atan2(dy, dx) - rYaw);
        while (angleDiff > pi) {
          angleDiff -= 2 * pi;
        }
        while (angleDiff < -pi) {
          angleDiff += 2 * pi;
        }

        bool inFov = angleDiff.abs() < _cameraFovHalfRad && distToRobot < 2.0;
        if (inFov) {
          t.missedVisible++;
          if (t.missedVisible > _maxMissed) continue;
        }
        survivors.add(t);
      }
      _trackedBalls = survivors;

      for (int i = 0; i < validWorldDets.length; i++) {
        if (!matchedDetIndices.contains(i)) {
          _trackedBalls.add(TrackedBall(id: _nextTrackId++, x: validWorldDets[i].worldX!, y: validWorldDets[i].worldY!, type: validWorldDets[i].type));
        }
      }
    }

    // 2. NEW MAP-DRIVEN TRACK TRACKING NAVIGATION LOGIC
    NavDirection calculatedDirection = NavDirection.standby;
    TrackedBall? nearestMapTarget;
    double targetHeadingDelta = 0.0;
    double calculatedDistanceCm = 0.0;

    List<TrackedBall> confirmedMapBalls = _trackedBalls.where((t) => t.confirmed).toList();

    if (confirmedMapBalls.isNotEmpty && _lastKnownX != null && _lastKnownY != null && _lastKnownYaw != null) {
      double shortestDistance = double.infinity;
      for (var ball in confirmedMapBalls) {
        double dist = sqrt(pow(ball.x - _lastKnownX!, 2) + pow(ball.y - _lastKnownY!, 2));
        if (dist < shortestDistance) { shortestDistance = dist; nearestMapTarget = ball; }
      }

      if (nearestMapTarget != null) {
        calculatedDistanceCm = shortestDistance * 100.0;
        double absoluteAngleToTarget = atan2(nearestMapTarget.y - _lastKnownY!, nearestMapTarget.x - _lastKnownX!);
        targetHeadingDelta = absoluteAngleToTarget - _lastKnownYaw!;

        while (targetHeadingDelta > pi) {
          targetHeadingDelta -= 2 * pi;
        }
        while (targetHeadingDelta < -pi) {
          targetHeadingDelta += 2 * pi;
        }

        if (targetHeadingDelta < navDeadzoneRight) {
          calculatedDirection = NavDirection.right;
        } else if (targetHeadingDelta > navDeadzoneLeft) {
          calculatedDirection = NavDirection.left;
        } else {
          calculatedDirection = NavDirection.forward;
        }
        _missedFrames = 0;
        _coastingDirection = calculatedDirection;
      }
    } else {
      _missedFrames++;
      if (_missedFrames < _maxCoastFrames) {
        calculatedDirection = _coastingDirection;
      } else {
        calculatedDirection = NavDirection.standby;
        _coastingDirection = NavDirection.standby;
      }
    }

    // 3. SERIAL CHASSIS DISPATCH SIGNALS
    if (calculatedDirection == _pendingDirection) {
      _stableFrames++;
    } else {
      _pendingDirection = calculatedDirection;
      _stableFrames = 1;
    }

    if (_stableFrames >= _requiredStableFrames || calculatedDirection == NavDirection.standby) {
      _lastSentDirection = calculatedDirection;
    }

    if (serialHelper != null && activeCategory == SettingsCategory.none) {
      String dirChar = '0';
      switch (_lastSentDirection) {
        case NavDirection.forward: dirChar = 'w'; break;
        case NavDirection.left: dirChar = 'a'; break;
        case NavDirection.right: dirChar = 'd'; break;
        case NavDirection.standby: dirChar = '0'; break;
      }

      serialHelper!.sendTelemetry(
        direction: dirChar,
        x: _lastKnownX ?? 0.0,
        y: _lastKnownY ?? 0.0,
        yaw: _lastKnownYaw ?? 0.0,
        collisionRisk: result.collisionRisk,
      );
    }

    DetectionCluster? targetCluster;
    if (result.detections.isNotEmpty) {
      final sorted = List<BallDetection>.from(result.detections)..sort((a, b) => ((a.absX1 + a.absX2) / 2).compareTo((b.absX1 + b.absX2) / 2));
      targetCluster = DetectionCluster(sorted);
    }

    onFrameProcessed?.call(NavigationOutput(
      direction: _lastSentDirection,
      distanceCm: calculatedDistanceCm,
      targetCluster: targetCluster,
      allDetections: result.detections,
      mapBalls: confirmedMapBalls,
      debugImage: result.debugImage,
      robotX: _lastKnownX,
      robotY: _lastKnownY,
      robotYaw: _lastKnownYaw,
      tagCount: result.tagCount,
      ceilingCutoffY: result.ceilingCutoffY,
      activeMapTarget: nearestMapTarget,
      targetHeadingDelta: targetHeadingDelta,
      collisionRisk: result.collisionRisk,
    ));
  }

  void dispose() {
    _receivePort.close();
    _isolate?.kill();
  }
}

// ─────────────────────────────────────────────
// Background Isolate Logic
// ─────────────────────────────────────────────

void _isolateEntry(List<dynamic> args) async {
  final SendPort sendPort = args[0];
  final RootIsolateToken rootToken = args[1];
  BackgroundIsolateBinaryMessenger.ensureInitialized(rootToken);

  Interpreter? interpreter;
  try {
    final options = InterpreterOptions()..threads = 4;
    if (Platform.isAndroid) options.addDelegate(GpuDelegateV2());
    interpreter = await Interpreter.fromAsset('assets/best_float32.tflite', options: options);
  } catch (e) {
    debugPrint('Model initialization failed: $e');
  }

  final ReceivePort port = ReceivePort();
  sendPort.send(port.sendPort);

  port.listen((message) {
    if (message is FrameRequest) sendPort.send(_processFrameData(message, interpreter));
  });
}

const int _downscaleFactor = 2;
const int _modelInputSize = 640;

FrameResult _processFrameData(FrameRequest req, Interpreter? interpreter) {
  final fullW = req.width, fullH = req.height;

  List<AprilTagDetection> detectedTags = [];
  double? rX, rY, rYaw;

  try {
    Uint8List yPlanePacked;
    if (req.yRowStride == fullW) {
      yPlanePacked = req.yPlane;
    } else {
      yPlanePacked = Uint8List(fullW * fullH);
      for (int y = 0; y < fullH; y++) {
        yPlanePacked.setRange(y * fullW, (y + 1) * fullW, req.yPlane, y * req.yRowStride);
      }
    }
    detectedTags = AprilTagFFI.runDetection(
        yPlanePacked, fullW, fullH,
        req.focalLengthPx, req.focalLengthPx, req.tagSizeCm / 100.0
    );

    if (detectedTags.isNotEmpty) {
      final localizer = TagLocalizer(
          tagWorld: buildTagWorld(),
          camPitchDown: req.camPitchDown,
          camHeight: req.camHeight,
          offFwd: req.offFwd,
          offRight: req.offRight
      );
      final pos = localizer.robotPosition(detectedTags);
      if (pos != null) { rX = pos[0]; rY = pos[1]; rYaw = pos[2]; }
    }
  } catch (e) { debugPrint("AprilTag FFI Error in Isolate: $e"); }

  final double? effectiveRX = rX ?? req.lastKnownX;
  final double? effectiveRY = rY ?? req.lastKnownY;
  final double? effectiveRYaw = rYaw ?? req.lastKnownYaw;

  double ceilingCutoffY = (fullH / 2) - (req.focalLengthPx * tan(req.camPitchDown));
  double highestTagBlackBorderTopEdgeY = double.infinity;
  final List<Rect2D> tagMaskZones = [];
  final validArenaTags = buildTagWorld();

  for (var tag in detectedTags) {
    if (!validArenaTags.containsKey(tag.tagId)) continue;
    if (tag.t.z > 0.1) {
      double px = (req.focalLengthPx * (tag.t.x / tag.t.z)) + (fullW / 2);
      double py = (req.focalLengthPx * (tag.t.y / tag.t.z)) + (fullH / 2);
      double tagPixSize = ((req.tagSizeCm / 100.0) * req.focalLengthPx) / tag.t.z;
      int maskRadius = (tagPixSize * 1.0).round();

      double currentTagTopY = py - maskRadius;
      if (currentTagTopY < highestTagBlackBorderTopEdgeY) highestTagBlackBorderTopEdgeY = currentTagTopY;

      tagMaskZones.add(Rect2D((px - maskRadius).round() ~/ _downscaleFactor, (py - maskRadius).round() ~/ _downscaleFactor, (maskRadius * 2) ~/ _downscaleFactor, (maskRadius * 2) ~/ _downscaleFactor));
    }
  }
  if (highestTagBlackBorderTopEdgeY != double.infinity && highestTagBlackBorderTopEdgeY > 0) ceilingCutoffY = highestTagBlackBorderTopEdgeY;

  final W = fullW ~/ _downscaleFactor, H = fullH ~/ _downscaleFactor;
  final Uint8List maskOrange = Uint8List(W * H);

  int nonWhitePixelCount = 0;

  for (int y = 0; y < H; y++) {
    final int uvRow = (y * _downscaleFactor >> 1) * req.uvRowStride;
    final int yRow = (y * _downscaleFactor) * req.yRowStride;
    for (int x = 0; x < W; x++) {
      final int origX = x * _downscaleFactor;
      final int uvIdx = uvRow + (origX >> 1) * req.uvPixelStride;
      final int yIdx = yRow + origX;
      if (yIdx >= req.yPlane.length || uvIdx >= req.uPlane.length) continue;

      final int yv = req.yPlane[yIdx], uv = req.uPlane[uvIdx], vv = req.vPlane[uvIdx];
      final int v2 = vv - 128, u2 = uv - 128;

      // Obstacle Heuristic: Count dark pixels or highly colored pixels
      // (assuming the floor is relatively bright and desaturated/white).
      if (yv < 120 || u2.abs() + v2.abs() > 50) {
        nonWhitePixelCount++;
      }

      final int r = (yv + ((359 * v2) >> 8)).clamp(0, 255);
      final int g = (yv - ((88 * u2 + 183 * v2) >> 8)).clamp(0, 255);
      final int b = (yv + ((454 * u2) >> 8)).clamp(0, 255);
      final int sidx = y * W + x;

      if (r > req.orangeRMin && (r - g) > req.orangeRGDiff && (r - b) > req.orangeRBDiff) {
        maskOrange[sidx] = 1;
      }
    }
  }

  // Calculate if a massive non-white object is consuming > 65% of the frame
  bool collisionRisk = nonWhitePixelCount > ((W * H) * 0.5);

  for (var zone in tagMaskZones) {
    for (int dy = 0; dy < zone.h; dy++) {
      for (int dx = 0; dx < zone.w; dx++) {
        int ex = zone.x + dx, ey = zone.y + dy;
        if (ex >= 0 && ex < W && ey >= 0 && ey < H) { maskOrange[ey * W + ex] = 0; }
      }
    }
  }

  final roisOrangeUnsorted = _findBoundingBoxes(maskOrange, W, H, req.orangeMinArea, req.orangeMaxArea, req.orangeMinAspect, req.orangeMaxAspect, req.orangeMinFill);
  roisOrangeUnsorted.sort((a, b) => (b.w * b.h).compareTo(a.w * a.h));
  final roisOrange = roisOrangeUnsorted.take(5).toList();

  final List<BallDetection> finalDetections = [];
  final outputBuffer = List.generate(1, (_) => List.generate(100, (_) => List.filled(5, 0.0)));

  void appendBallProjection({
    required double fx, required double fy, required double fw, required double fh, required double cx, required double cy,
    required double calculatedDistanceCm, required double confidence, required bool isYolo, required BallType type,
  }) {
    double? wX, wY;
    if (effectiveRX != null && effectiveRY != null && effectiveRYaw != null) {
      double forwardMeters = calculatedDistanceCm / 100.0;
      double rightMeters = ((cx - fullW / 2.0) * forwardMeters) / req.focalLengthPx;
      double cosYaw = cos(effectiveRYaw), sinYaw = sin(effectiveRYaw);
      wX = effectiveRX + (forwardMeters * cosYaw) + (rightMeters * sinYaw);
      wY = effectiveRY + (forwardMeters * sinYaw) - (rightMeters * cosYaw);
    }
    finalDetections.add(BallDetection(absX1: fx, absY1: fy, absX2: fx + fw, absY2: fy + fh, confidence: confidence, distanceCm: calculatedDistanceCm, isYolo: isYolo, type: type, worldX: wX, worldY: wY));
  }

  void processRois(List<Rect2D> rois, BallType type) {
    final double targetDiamCm = req.orangeDiameterCm;
    final double absoluteMaxPix = ((targetDiamCm * req.focalLengthPx) / 20.0) * 1.5;

    for (final roi in rois) {
      final int fx = roi.x * _downscaleFactor, fy = roi.y * _downscaleFactor;
      final int fw = roi.w * _downscaleFactor, fh = roi.h * _downscaleFactor;

      if (fw > absoluteMaxPix || fh > absoluteMaxPix || fy < ceilingCutoffY) continue;

      final int padding = 80;
      final int px1 = max(0, fx - padding), py1 = max(0, fy - padding);
      final int px2 = min(fullW - 1, fx + fw + padding), py2 = min(fullH - 1, fy + fh + padding);
      final int rW = px2 - px1, rH = py2 - py1;
      bool yoloFound = false;

      if (interpreter != null && rW > 0 && rH > 0) {
        final Int8List tensor = Int8List(_modelInputSize * _modelInputSize * 3);
        final List<int> xMap = List.generate(_modelInputSize, (tx) => (px1 + (tx * rW ~/ _modelInputSize)).clamp(0, fullW - 1));
        final List<int> yMap = List.generate(_modelInputSize, (ty) => (py1 + (ty * rH ~/ _modelInputSize)).clamp(0, fullH - 1));
        int dstIdx = 0;
        for (int ty = 0; ty < _modelInputSize; ty++) {
          final int yRow = yMap[ty] * req.yRowStride, uvRow = (yMap[ty] >> 1) * req.uvRowStride;
          for (int tx = 0; tx < _modelInputSize; tx++) {
            final int yIdx = yRow + xMap[tx], uvIdx = uvRow + (xMap[tx] >> 1) * req.uvPixelStride;
            final int yv = req.yPlane[yIdx], u2 = req.uPlane[uvIdx] - 128, v2 = req.vPlane[uvIdx] - 128;
            tensor[dstIdx++] = (yv + ((359 * v2) >> 8)).clamp(0, 255) - 128;
            tensor[dstIdx++] = (yv - ((88 * u2 + 183 * v2) >> 8)).clamp(0, 255) - 128;
            tensor[dstIdx++] = (yv + ((454 * u2) >> 8)).clamp(0, 255) - 128;
          }
        }
        try {
          interpreter.run(tensor.reshape([1, _modelInputSize, _modelInputSize, 3]), outputBuffer);
          for (int d = 0; d < 100; d++) {
            final conf = outputBuffer[0][d][4];
            if (conf < req.yoloConfThreshold) continue;
            final double boxY1 = py1 + outputBuffer[0][d][1] * rH, boxY2 = py1 + outputBuffer[0][d][3] * rH;
            if (boxY1 < ceilingCutoffY) continue;
            final double boxX1 = px1 + outputBuffer[0][d][0] * rW, boxX2 = px1 + outputBuffer[0][d][2] * rW;
            final double pixDiam = max(boxX2 - boxX1, boxY2 - boxY1);
            double rawDistance = (targetDiamCm * req.focalLengthPx) / pixDiam;

            appendBallProjection(
              fx: boxX1, fy: boxY1, fw: boxX2 - boxX1, fh: boxY2 - boxY1,
              cx: boxX1 + (boxX2 - boxX1) / 2.0, cy: boxY1 + (boxY2 - boxY1) / 2.0,
              calculatedDistanceCm: rawDistance > 16.0 ? sqrt((rawDistance * rawDistance) - (16.0 * 16.0)) : 16.0,
              confidence: conf, isYolo: true, type: type,
            );
            yoloFound = true;
          }
        } catch (e) { debugPrint('TFLite Processing Fallback Activated: $e'); }
      }

      if (!yoloFound) {
        final double pixDiam = max(fw, fh).toDouble();
        double rawDistance = (targetDiamCm * req.focalLengthPx) / pixDiam;
        appendBallProjection(
          fx: fx.toDouble(), fy: fy.toDouble(), fw: fw.toDouble(), fh: fh.toDouble(), cx: fx + fw / 2.0, cy: fy + fh / 2.0,
          calculatedDistanceCm: rawDistance > 16.0 ? sqrt((rawDistance * rawDistance) - (16.0 * 16.0)) : 16.0,
          confidence: 0.1, isYolo: false, type: type,
        );
      }
    }
  }

  processRois(roisOrange, BallType.orange);

  Uint8List? debugBmp;
  if (req.debugCategory != SettingsCategory.none && req.debugCategory != SettingsCategory.direction) {
    debugBmp = _createDebugBmp(maskOrange, W, H, req.debugCategory);
  }

  return FrameResult(finalDetections, debugBmp, rX, rY, rYaw, detectedTags.length, ceilingCutoffY, collisionRisk);
}

List<Rect2D> _findBoundingBoxes(Uint8List mask, int W, int H, int minArea, int maxArea, double minAspect, double maxAspect, double minFill) {
  final visited = Uint8List(W * H);
  final boxes = <Rect2D>[];
  for (int sy = 0; sy < H; sy++) {
    for (int sx = 0; sx < W; sx++) {
      final int sidx = sy * W + sx;
      if (mask[sidx] == 0 || visited[sidx] == 1) continue;

      int minX = sx, maxX = sx, minY = sy, maxY = sy, area = 0;
      final queue = <int>[sidx];
      visited[sidx] = 1;

      while (queue.isNotEmpty) {
        final cur = queue.removeLast();
        final cx = cur % W, cy = cur ~/ W;
        area++;

        if (cx < minX) minX = cx;
        if (cx > maxX) maxX = cx;
        if (cy < minY) minY = cy;
        if (cy > maxY) maxY = cy;

        if (cx > 0 && mask[cur - 1] == 1 && visited[cur - 1] == 0) { visited[cur - 1] = 1; queue.add(cur - 1); }
        if (cx < W - 1 && mask[cur + 1] == 1 && visited[cur + 1] == 0) { visited[cur + 1] = 1; queue.add(cur + 1); }
        if (cy > 0 && mask[cur - W] == 1 && visited[cur - W] == 0) { visited[cur - W] = 1; queue.add(cur - W); }
        if (cy < H - 1 && mask[cur + W] == 1 && visited[cur + W] == 0) { visited[cur + W] = 1; queue.add(cur + W); }
      }

      if (area >= minArea && area <= maxArea) {
        int bw = maxX - minX + 1;
        int bh = maxY - minY + 1;
        double aspect = bw / bh;
        double fillRatio = area / (bw * bh);

        if (aspect >= minAspect && aspect <= maxAspect && fillRatio >= minFill) {
          boxes.add(Rect2D(minX, minY, bw, bh));
        }
      }
    }
  }
  return boxes;
}

Uint8List _createDebugBmp(Uint8List maskOrange, int width, int height, SettingsCategory mode) {
  final int rowSize = width * 4, imageSize = rowSize * height, fileSize = 54 + imageSize;
  final bmp = Uint8List(fileSize);
  final bd = ByteData.view(bmp.buffer);

  bd.setUint8(0, 0x42); bd.setUint8(1, 0x4D); bd.setUint32(2, fileSize, Endian.little); bd.setUint32(10, 54, Endian.little);
  bd.setUint32(14, 40, Endian.little); bd.setInt32(18, width, Endian.little); bd.setInt32(22, -height, Endian.little);
  bd.setUint16(26, 1, Endian.little); bd.setUint16(28, 32, Endian.little); bd.setUint32(34, imageSize, Endian.little);

  int offset = 54;
  for (int i = 0; i < width * height; i++) {
    if (mode == SettingsCategory.orange && maskOrange[i] == 1) {
      bmp[offset++] = 0; bmp[offset++] = 165; bmp[offset++] = 255; bmp[offset++] = 255;
    } else {
      bmp[offset++] = 0; bmp[offset++] = 0; bmp[offset++] = 0; bmp[offset++] = 255;
    }
  }
  return bmp;
}