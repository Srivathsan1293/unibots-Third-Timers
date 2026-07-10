// lib/main.dart

import 'dart:async';
import 'dart:math';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:camera/camera.dart';

import 'Detection.dart';
import 'serial_helper.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await SystemChrome.setPreferredOrientations([
    DeviceOrientation.landscapeRight,
    DeviceOrientation.landscapeLeft,
  ]);
  final cameras = await availableCameras();
  runApp(MaterialApp(
    debugShowCheckedModeBanner: false,
    theme: ThemeData.dark(),
    home: BallDetectorHome(cameras: cameras),
  ));
}

class BallDetectorHome extends StatefulWidget {
  final List<CameraDescription> cameras;
  const BallDetectorHome({super.key, required this.cameras});

  @override
  State<BallDetectorHome> createState() => _BallDetectorHomeState();
}

class _BallDetectorHomeState extends State<BallDetectorHome> {
  CameraController? _cam;
  late DetectionSystem _detectionSystem;
  NavigationOutput? _navState;

  bool _isDetectionActive = false;
  int _startupCountdown = 0;
  Timer? _startupTimer;

  int _fps = 0;
  int _fpsAccum = 0;
  DateTime _lastFpsTs = DateTime.now();
  int _lastCameraTime = 0;
  double _imgW = 1280;
  double _imgH = 720;

  SerialHelper? _serialHelper;

  double _calibPixelDiam = 100.0;
  double _calibRefDistCm = 50.0;

  Future<void> _loadTuningValues() async {
    final prefs = await SharedPreferences.getInstance();
    setState(() {
      _detectionSystem.orangeRMin = prefs.getInt('orangeRMin') ?? 130;
      _detectionSystem.orangeRGDiff = prefs.getInt('orangeRGDiff') ?? 30;
      _detectionSystem.orangeRBDiff = prefs.getInt('orangeRBDiff') ?? 30;
      _detectionSystem.orangeMinArea = prefs.getInt('orangeMinArea') ?? 24;
      _detectionSystem.orangeMaxArea = prefs.getInt('orangeMaxArea') ?? 5000;
      _detectionSystem.orangeMinAspect = prefs.getDouble('orangeMinAspect') ?? 0.4;
      _detectionSystem.orangeMaxAspect = prefs.getDouble('orangeMaxAspect') ?? 2.2;
      _detectionSystem.orangeMinFill = prefs.getDouble('orangeMinFill') ?? 0.25;

      _detectionSystem.camPitchDown = prefs.getDouble('camPitchDown') ?? 0.349;
      _detectionSystem.camHeight = prefs.getDouble('camHeight') ?? 0.179;
      _detectionSystem.offFwd = prefs.getDouble('offFwd') ?? 0.03;
      _detectionSystem.offRight = prefs.getDouble('offRight') ?? 0.0;

      _detectionSystem.focalLengthPx = prefs.getDouble('focalLengthPx') ?? 615.0;
      _detectionSystem.orangeDiameterCm = prefs.getDouble('orangeDiameterCm') ?? 4.0;
      _detectionSystem.tagSizeCm = prefs.getDouble('tagSizeCm') ?? 7.81;
      _detectionSystem.yoloConfThreshold = prefs.getDouble('yoloConfThreshold') ?? 0.50;

      _calibPixelDiam = prefs.getDouble('calibPixelDiam') ?? 100.0;

      _detectionSystem.navDeadzoneLeft = prefs.getDouble('navDeadzoneLeft') ?? 0.20;
      _detectionSystem.navDeadzoneRight = prefs.getDouble('navDeadzoneRight') ?? -0.20;
    });
  }

  Future<void> _saveValue(String key, dynamic value) async {
    final prefs = await SharedPreferences.getInstance();
    if (value is int) {
      await prefs.setInt(key, value);
    } else if (value is double) {
      await prefs.setDouble(key, value);
    }
  }

  @override
  void initState() {
    super.initState();
    _serialHelper = SerialHelper();
    _detectionSystem = DetectionSystem(
      onFrameProcessed: (state) {
        if (mounted) setState(() => _navState = state);
      },
      serialHelper: _serialHelper,
    );
    _loadTuningValues();
    _initCamera();
  }

  Future<void> _initCamera() async {
    if (widget.cameras.isEmpty) return;
    CameraDescription selectedCam = widget.cameras.firstWhere(
          (cam) => cam.lensDirection == CameraLensDirection.back && !cam.name.toLowerCase().contains('wide') && !cam.name.contains('2'),
      orElse: () => widget.cameras.firstWhere((cam) => cam.lensDirection == CameraLensDirection.back, orElse: () => widget.cameras[0]),
    );

    _cam = CameraController(selectedCam, ResolutionPreset.medium, enableAudio: false, imageFormatGroup: ImageFormatGroup.yuv420);
    await _cam!.initialize();

    try { await _cam!.setFocusMode(FocusMode.auto); await _cam!.setExposureMode(ExposureMode.auto); } catch (e) { debugPrint("Setup error: $e"); }

    await _detectionSystem.start();
    _cam!.startImageStream(_onCameraFrame);
    if (mounted) setState(() {});
  }

  void _onCameraFrame(CameraImage image) {
    if (!_isDetectionActive) return;

    final nowMs = DateTime.now().millisecondsSinceEpoch;
    if (nowMs - _lastCameraTime < 33) return;
    _lastCameraTime = nowMs;

    _imgW = image.width.toDouble(); _imgH = image.height.toDouble();
    _fpsAccum++;
    final now = DateTime.now();
    if (now.difference(_lastFpsTs).inMilliseconds >= 1000) {
      if (mounted) setState(() => _fps = _fpsAccum);
      _fpsAccum = 0; _lastFpsTs = now;
    }
    _detectionSystem.processFrame(image);
  }

  Widget _buildSlider(String label, double val, double min, double max, ValueChanged<double> onChanged) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text("$label: ${val.toStringAsFixed(val % 1 == 0 ? 0 : 2)}", style: const TextStyle(color: Colors.white, fontSize: 11)),
          Slider(
            value: val.clamp(min, max), min: min, max: max,
            activeColor: Colors.cyanAccent, inactiveColor: Colors.white24,
            onChanged: onChanged,
          ),
        ],
      ),
    );
  }

  List<Widget> _buildSettingsPanel() {
    switch (_detectionSystem.activeCategory) {
      case SettingsCategory.orange:
        return [
          const Text("Color Bounds", style: TextStyle(color: Colors.orange, fontWeight: FontWeight.bold)),
          _buildSlider("Red Min", _detectionSystem.orangeRMin.toDouble(), 0, 255, (v) { setState(() => _detectionSystem.orangeRMin = v.toInt()); _saveValue('orangeRMin', v.toInt()); }),
          _buildSlider("R-G Diff (Warmth)", _detectionSystem.orangeRGDiff.toDouble(), 0, 100, (v) { setState(() => _detectionSystem.orangeRGDiff = v.toInt()); _saveValue('orangeRGDiff', v.toInt()); }),
          _buildSlider("R-B Diff", _detectionSystem.orangeRBDiff.toDouble(), 0, 100, (v) { setState(() => _detectionSystem.orangeRBDiff = v.toInt()); _saveValue('orangeRBDiff', v.toInt()); }),
          const Divider(),
          const Text("Size Restrictions (px)", style: TextStyle(color: Colors.orange, fontWeight: FontWeight.bold)),
          _buildSlider("Min Area", _detectionSystem.orangeMinArea.toDouble(), 1, 1000, (v) { setState(() => _detectionSystem.orangeMinArea = v.toInt()); _saveValue('orangeMinArea', v.toInt()); }),
          _buildSlider("Max Area", _detectionSystem.orangeMaxArea.toDouble(), 1000, 10000, (v) { setState(() => _detectionSystem.orangeMaxArea = v.toInt()); _saveValue('orangeMaxArea', v.toInt()); }),
          const Divider(),
          const Text("Shape & Shadows", style: TextStyle(color: Colors.orange, fontWeight: FontWeight.bold)),
          _buildSlider("Min Aspect (Roundness)", _detectionSystem.orangeMinAspect, 0.1, 1.0, (v) { setState(() => _detectionSystem.orangeMinAspect = v); _saveValue('orangeMinAspect', v); }),
          _buildSlider("Max Aspect (Shadows)", _detectionSystem.orangeMaxAspect, 1.0, 3.0, (v) { setState(() => _detectionSystem.orangeMaxAspect = v); _saveValue('orangeMaxAspect', v); }),
          _buildSlider("Min Solidity", _detectionSystem.orangeMinFill, 0.1, 1.0, (v) { setState(() => _detectionSystem.orangeMinFill = v); _saveValue('orangeMinFill', v); }),
        ];
      case SettingsCategory.localization:
        return [
          const Text("Camera Position", style: TextStyle(color: Colors.purpleAccent, fontWeight: FontWeight.bold)),
          _buildSlider("Pitch Angle Down (°)", _detectionSystem.camPitchDown * 180 / pi, 0.0, 90.0, (v) { setState(() => _detectionSystem.camPitchDown = v * pi / 180); _saveValue('camPitchDown', v * pi / 180); }),
          _buildSlider("distance from tag (m)", _detectionSystem.offFwd, -0.1, 0.50, (v) { setState(() => _detectionSystem.offFwd = v); _saveValue('offFwd', v); }),
          _buildSlider("Height From Ground (m)", _detectionSystem.camHeight, 0.05, 0.40, (v) { setState(() => _detectionSystem.camHeight = v); _saveValue('camHeight', v); }),
          _buildSlider("Offset Right (m)", _detectionSystem.offRight, -0.1, 0.1, (v) { setState(() => _detectionSystem.offRight = v); _saveValue('offRight', v); }),
          const Divider(),
          const Text("TFLite Model", style: TextStyle(color: Colors.lightGreen, fontWeight: FontWeight.bold)),
          _buildSlider("YOLO Conf Threshold", _detectionSystem.yoloConfThreshold, 0.1, 0.95, (v) { setState(() => _detectionSystem.yoloConfThreshold = v); _saveValue('yoloConfThreshold', v); }),
        ];
      case SettingsCategory.distance:
        double derivedDist = 0.0;
        double realDiam = _detectionSystem.orangeDiameterCm;
        if (_calibPixelDiam > 0) derivedDist = (_detectionSystem.focalLengthPx * realDiam) / _calibPixelDiam;

        return [
          const Text("Real World Exact Sizes", style: TextStyle(color: Colors.greenAccent, fontWeight: FontWeight.bold)),
          _buildSlider("Orange Ball Diam (cm)", _detectionSystem.orangeDiameterCm, 2.0, 8.0, (v) { setState(() => _detectionSystem.orangeDiameterCm = v); _saveValue('orangeDiameterCm', v); }),
          _buildSlider("AprilTag Size (cm)", _detectionSystem.tagSizeCm, 5.0, 15.0, (v) { setState(() => _detectionSystem.tagSizeCm = v); _saveValue('tagSizeCm', v); }),
          const Divider(),
          const Text("Distance Auto-Calibration", style: TextStyle(color: Colors.cyanAccent, fontWeight: FontWeight.bold)),
          const Padding(
            padding: EdgeInsets.symmetric(vertical: 8.0),
            child: Text("Target: Orange Ball", style: TextStyle(color: Colors.orange, fontSize: 13, fontWeight: FontWeight.bold)),
          ),
          _buildSlider("Ref. Distance (cm)", _calibRefDistCm, 10, 200, (v) { setState(() => _calibRefDistCm = v); }),
          _buildSlider("Apparent Size (px)", _calibPixelDiam, 10, 500, (v) { setState(() => _calibPixelDiam = v); _saveValue('calibPixelDiam', v); }),
          const SizedBox(height: 8),
          Row(
            children: [
              Expanded(
                child: ElevatedButton(
                  onPressed: () {
                    double newFocal = (_calibRefDistCm * _calibPixelDiam) / realDiam;
                    setState(() {
                      _detectionSystem.focalLengthPx = newFocal;
                    });
                    _saveValue('focalLengthPx', newFocal);
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(content: Text("Focal Length calibrated to ${newFocal.toStringAsFixed(1)}px")),
                    );
                  },
                  style: ElevatedButton.styleFrom(backgroundColor: Colors.cyan.shade700, foregroundColor: Colors.white),
                  child: const Text("Calculate Focal Length", style: TextStyle(fontSize: 11)),
                ),
              ),
              const SizedBox(width: 8),
              IconButton(
                icon: const Icon(Icons.ads_click, color: Colors.greenAccent),
                tooltip: "Lock to largest detection",
                onPressed: () {
                  if (_navState != null && _navState!.allDetections.isNotEmpty) {
                    var filtered = List<BallDetection>.from(_navState!.allDetections);
                    if (filtered.isNotEmpty) {
                      filtered.sort((a, b) => (b.absX2 - b.absX1).abs().compareTo((a.absX2 - a.absX1).abs()));
                      double largestPx = (filtered.first.absX2 - filtered.first.absX1).abs();
                      setState(() {
                        _calibPixelDiam = largestPx;
                      });
                      _saveValue('calibPixelDiam', largestPx);
                    }
                  }
                },
              )
            ],
          ),
          const Divider(),
          _buildSlider("Manual Focal Length (px)", _detectionSystem.focalLengthPx, 50, 4000, (v) { setState(() => _detectionSystem.focalLengthPx = v); _saveValue('focalLengthPx', v); }),
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 6.0),
            child: Text("Verification Distance: ${derivedDist.toStringAsFixed(1)} cm", style: const TextStyle(color: Colors.white70, fontSize: 11)),
          ),
        ];
      case SettingsCategory.direction:
        return [
          const Text("Heading Deadzone (Straight)", style: TextStyle(color: Colors.cyan, fontWeight: FontWeight.bold)),
          _buildSlider("Turn Left Threshold (rad)", _detectionSystem.navDeadzoneLeft, 0.01, 1.0, (v) { setState(() => _detectionSystem.navDeadzoneLeft = v); _saveValue('navDeadzoneLeft', v); }),
          _buildSlider("Turn Right Threshold (rad)", _detectionSystem.navDeadzoneRight, -1.0, -0.01, (v) { setState(() => _detectionSystem.navDeadzoneRight = v); _saveValue('navDeadzoneRight', v); }),
        ];
      default: return [];
    }
  }

  @override
  Widget build(BuildContext context) {
    if (_cam == null || !_cam!.value.isInitialized) return const Scaffold(body: Center(child: CircularProgressIndicator()));

    String cmd = "Standby"; Color cmdColor = Colors.grey;
    if (_navState != null) {
      switch (_navState!.direction) {
        case NavDirection.forward: cmd = "FORWARD (w)"; cmdColor = Colors.green; break;
        case NavDirection.left: cmd = "LEFT (a)"; cmdColor = Colors.blue; break;
        case NavDirection.right: cmd = "RIGHT (d)"; cmdColor = Colors.red; break;
        case NavDirection.standby: cmd = "STANDBY (0)"; cmdColor = Colors.grey; break;
      }
    }

    return Scaffold(
      backgroundColor: Colors.black,
      body: Stack(
        children: [
          Row(
            children: [
              // Visual Stream Column
              Expanded(
                flex: 1,
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    AspectRatio(
                      aspectRatio: _imgW / _imgH,
                      child: LayoutBuilder(
                        builder: (context, constraints) {
                          final double scaleX = constraints.maxWidth / _imgW;
                          final double scaleY = constraints.maxHeight / _imgH;

                          double centerX = constraints.maxWidth / 2;
                          double leftBarX = centerX - (_detectionSystem.focalLengthPx * tan(_detectionSystem.navDeadzoneLeft)) * scaleX;
                          double rightBarX = centerX - (_detectionSystem.focalLengthPx * tan(_detectionSystem.navDeadzoneRight)) * scaleX;

                          return Stack(
                            children: [
                              CameraPreview(_cam!),

                              if (!_isDetectionActive)
                                Positioned.fill(
                                  child: Container(
                                    color: Colors.black54,
                                    child: Center(
                                      child: Text(
                                        _startupCountdown > 0
                                            ? "Starting Detection in $_startupCountdown..."
                                            : "App Standby\nPress 'Start App' to connect",
                                        textAlign: TextAlign.center,
                                        style: const TextStyle(
                                          color: Colors.white,
                                          fontSize: 24,
                                          fontWeight: FontWeight.bold,
                                          shadows: [Shadow(color: Colors.black, blurRadius: 4)],
                                        ),
                                      ),
                                    ),
                                  ),
                                ),

                              if (_navState?.debugImage != null && _detectionSystem.activeCategory != SettingsCategory.none && _detectionSystem.activeCategory != SettingsCategory.localization)
                                Positioned.fill(
                                  child: Opacity(opacity: 0.85, child: Image.memory(_navState!.debugImage!, gaplessPlayback: true, fit: BoxFit.fill)),
                                ),

                              if (_navState != null)
                                Positioned(
                                  left: 0,
                                  right: 0,
                                  top: _navState!.ceilingCutoffY * scaleY,
                                  child: Container(height: 2.5, color: Colors.redAccent.withValues(alpha: 0.85)),
                                ),

                              // Obstacle Collision UI Flash
                              if (_navState?.collisionRisk == true)
                                Positioned.fill(
                                    child: Container(
                                        decoration: BoxDecoration(
                                            border: Border.all(color: Colors.redAccent, width: 8.0)
                                        ),
                                        child: const Center(
                                            child: Text(
                                                "OBSTACLE EVASION",
                                                style: TextStyle(
                                                    color: Colors.redAccent,
                                                    fontSize: 28,
                                                    fontWeight: FontWeight.bold,
                                                    shadows: [Shadow(color: Colors.black, blurRadius: 6)]
                                                )
                                            )
                                        )
                                    )
                                ),

                              if (_detectionSystem.activeCategory == SettingsCategory.direction) ...[
                                Positioned(left: leftBarX, top: 0, bottom: 0, child: Container(width: 3, color: Colors.blueAccent)),
                                Positioned(left: rightBarX, top: 0, bottom: 0, child: Container(width: 3, color: Colors.redAccent)),
                                Positioned(
                                    left: rightBarX + 5, top: 20,
                                    child: const Text("STRAIGHT ZONE", style: TextStyle(color: Colors.greenAccent, fontWeight: FontWeight.bold))
                                )
                              ],

                              if (_navState != null)
                                ..._navState!.allDetections.map((det) {
                                  bool isTarget = _navState?.activeMapTarget != null && det.worldX != null && det.worldY != null &&
                                      sqrt(pow(det.worldX! - _navState!.activeMapTarget!.x, 2) + pow(det.worldY! - _navState!.activeMapTarget!.y, 2)) < 0.12;

                                  Color boxColor = det.isYolo ? Colors.orange : Colors.deepOrange;
                                  double apparentPixelWidth = (det.absX2 - det.absX1).abs();
                                  return Positioned(
                                    left: det.absX1 * scaleX, top: det.absY1 * scaleY,
                                    width: (det.absX2 - det.absX1).abs() * scaleX, height: (det.absY2 - det.absY1).abs() * scaleY,
                                    child: Stack(
                                      clipBehavior: Clip.none,
                                      children: [
                                        Container(
                                          decoration: BoxDecoration(border: Border.all(color: boxColor, width: isTarget ? 3.0 : 1.5)),
                                        ),
                                        Positioned(
                                          top: -20,
                                          left: -1,
                                          child: Container(
                                            color: boxColor,
                                            padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 2),
                                            child: Text(
                                                '${isTarget ? '★ ' : ''}${apparentPixelWidth.toStringAsFixed(0)}px | ${det.distanceCm.toStringAsFixed(0)}cm',
                                                style: const TextStyle(color: Colors.black, fontSize: 11, fontWeight: FontWeight.bold, height: 1.0)
                                            ),
                                          ),
                                        ),
                                      ],
                                    ),
                                  );
                                })
                            ],
                          );
                        },
                      ),
                    ),

                    const SizedBox(height: 8),

                    if (_navState != null)
                      Container(
                        decoration: BoxDecoration(
                            color: Colors.black,
                            borderRadius: BorderRadius.circular(8),
                            border: Border.all(color: _navState?.activeMapTarget != null ? Colors.greenAccent : Colors.transparent, width: 1.5)
                        ),
                        child: Padding(
                          padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
                          child: Row(
                            mainAxisAlignment: MainAxisAlignment.spaceBetween,
                            children: [
                              Text(cmd, style: TextStyle(color: cmdColor, fontSize: 16, fontWeight: FontWeight.bold, letterSpacing: 1.0)),
                              if (_navState?.activeMapTarget != null)
                                Column(
                                  crossAxisAlignment: CrossAxisAlignment.end, mainAxisSize: MainAxisSize.min,
                                  children: [
                                    Text('Target: ID-${_navState!.activeMapTarget!.id}', style: const TextStyle(color: Colors.greenAccent, fontSize: 12, fontWeight: FontWeight.bold)),
                                    Text('Dist: ${_navState!.distanceCm.toStringAsFixed(1)} cm', style: const TextStyle(color: Colors.white70, fontSize: 11)),
                                  ],
                                ),
                            ],
                          ),
                        ),
                      ),
                  ],
                ),
              ),

              // Digital Twin Map
              Expanded(
                flex: 1,
                child: Container(
                  margin: const EdgeInsets.all(12),
                  decoration: BoxDecoration(color: Colors.grey, border: Border.all(color: Colors.blueGrey.shade800, width: 2), borderRadius: BorderRadius.circular(12)),
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(12),
                    child: Stack(
                      fit: StackFit.expand,
                      children: [
                        Image.asset('assets/arena_map.png', fit: BoxFit.contain),
                        CustomPaint(painter: MapPainter(robotX: _navState?.robotX, robotY: _navState?.robotY, robotYaw: _navState?.robotYaw, balls: _navState?.mapBalls ?? [], activeTarget: _navState?.activeMapTarget)),
                      ],
                    ),
                  ),
                ),
              ),
            ],
          ),

          if (_detectionSystem.activeCategory != SettingsCategory.none)
            Positioned(
              right: 10,
              top: 55, bottom: 80, width: 300,
              child: Container(
                decoration: BoxDecoration(color: Colors.black.withValues(alpha: 0.9), borderRadius: BorderRadius.circular(12), border: Border.all(color: Colors.cyanAccent.withValues(alpha: 0.5))),
                padding: const EdgeInsets.all(12),
                child: ListView(children: _buildSettingsPanel()),
              ),
            ),

          Positioned(
            top: 10, right: 10,
            child: Row(
              children: [
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 12),
                  decoration: BoxDecoration(color: Colors.blueGrey.shade900, borderRadius: BorderRadius.circular(4)),
                  child: DropdownButton<SettingsCategory>(
                    value: _detectionSystem.activeCategory,
                    dropdownColor: Colors.blueGrey.shade900,
                    underline: Container(),
                    icon: const Icon(Icons.settings, color: Colors.white),
                    items: const [
                      DropdownMenuItem(value: SettingsCategory.none, child: Text("Hide Settings", style: TextStyle(color: Colors.white))),
                      DropdownMenuItem(value: SettingsCategory.orange, child: Text("Orange Tuner", style: TextStyle(color: Colors.orange))),
                      DropdownMenuItem(value: SettingsCategory.localization, child: Text("Localisation", style: TextStyle(color: Colors.purpleAccent))),
                      DropdownMenuItem(value: SettingsCategory.distance, child: Text("Distance Sync", style: TextStyle(color: Colors.greenAccent))),
                      DropdownMenuItem(value: SettingsCategory.direction, child: Text("Direction Range", style: TextStyle(color: Colors.cyan))),
                    ],
                    onChanged: (val) { if (val != null) setState(() => _detectionSystem.activeCategory = val); },
                  ),
                ),
                const SizedBox(width: 8),

                ElevatedButton.icon(
                  icon: Icon(
                      (_serialHelper?.isConnected == true || _isDetectionActive || _startupCountdown > 0) ? Icons.stop : Icons.play_arrow,
                      size: 16
                  ),
                  label: Text((_serialHelper?.isConnected == true || _isDetectionActive || _startupCountdown > 0) ? "Stop App" : "Start App"),
                  style: ElevatedButton.styleFrom(
                      backgroundColor: (_serialHelper?.isConnected == true || _isDetectionActive || _startupCountdown > 0)
                          ? Colors.red
                          : Colors.green
                  ),
                  onLongPress: () {
                    setState(() {
                      _isDetectionActive = true;
                      _startupCountdown = 0;
                    });
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text("Offline Tuning Mode Started")),
                    );
                  },
                  onPressed: () async {
                    if (_serialHelper?.isConnected == true || _isDetectionActive || _startupCountdown > 0) {
                      _serialHelper?.disconnect();
                      _startupTimer?.cancel();
                      setState(() {
                        _isDetectionActive = false;
                        _startupCountdown = 0;
                      });
                    } else {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(content: Text("Connecting to Pi over USB Tether...")),
                      );

                      bool success = await _serialHelper!.connectToPi();

                      setState(() {
                        _startupCountdown = 5;
                        _isDetectionActive = false;
                      });

                      _startupTimer?.cancel();
                      _startupTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
                        if (!mounted) {
                          timer.cancel();
                          return;
                        }
                        setState(() {
                          _startupCountdown--;
                          if (_startupCountdown <= 0) {
                            _isDetectionActive = true;
                            timer.cancel();
                          }
                        });
                      });

                      if (!mounted) return;
                      setState(() {});
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: Text(success ? "Connected! Starting in 5 seconds..." : "Connection Failed! Starting anyway..."),
                          backgroundColor: success ? Colors.green : Colors.orange,
                        ),
                      );
                    }
                  },
                ),
              ],
            ),
          ),

          Positioned(
            top: 10, left: 10,
            child: Container(
              padding: const EdgeInsets.all(8), color: Colors.black54,
              child: Text(
                'FPS: $_fps | Res: ${_imgW.toInt()}x${_imgH.toInt()} | Tags: ${_navState?.tagCount ?? 0} | X: ${_navState?.robotX?.toStringAsFixed(2) ?? "-"} | Y: ${_navState?.robotY?.toStringAsFixed(2) ?? "-"} | Yaw: ${((_navState?.robotYaw ?? 0) * 180 / pi).toStringAsFixed(1)}°',
                style: const TextStyle(color: Colors.white, fontSize: 13, fontFamily: 'monospace'),
                overflow: TextOverflow.ellipsis,
                maxLines: 1,
              ),
            ),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    _startupTimer?.cancel();
    _cam?.dispose();
    _detectionSystem.dispose();
    _serialHelper?.disconnect();
    super.dispose();
  }
}

class MapPainter extends CustomPainter {
  final double? robotX, robotY, robotYaw;
  final List<TrackedBall> balls;
  final TrackedBall? activeTarget;
  final double arenaMin = -1.2, arenaMax = 1.2;

  MapPainter({required this.robotX, required this.robotY, required this.robotYaw, required this.balls, this.activeTarget});

  @override
  void paint(Canvas canvas, Size size) {
    double scale(double value) => (value / (arenaMax - arenaMin)) * size.width;
    Offset toCanvas(double wX, double wY) => Offset(((wX - arenaMin) / (arenaMax - arenaMin)) * size.width, (1.0 - ((wY - arenaMin) / (arenaMax - arenaMin))) * size.height);

    for (var b in balls) {
      Offset pos = toCanvas(b.x, b.y);
      bool isTarget = activeTarget != null && b.id == activeTarget!.id;
      canvas.drawCircle(pos, scale(0.025), Paint()..color = isTarget ? Colors.red : Colors.orange..style = PaintingStyle.fill);
      if (isTarget) canvas.drawCircle(pos, scale(0.045), Paint()..color = Colors.redAccent..style = PaintingStyle.stroke..strokeWidth = 2);
    }

    if (robotX != null && robotY != null && robotYaw != null) {
      Offset rPos = toCanvas(robotX!, robotY!);
      double robotRadiusPx = scale(0.05);
      canvas.drawCircle(rPos, robotRadiusPx, Paint()..color = Colors.cyanAccent.withValues(alpha: 0.35)..style = PaintingStyle.fill);
      canvas.drawCircle(rPos, robotRadiusPx, Paint()..color = Colors.cyanAccent..style = PaintingStyle.stroke..strokeWidth = 1.5);
      canvas.drawLine(rPos, Offset(rPos.dx + cos(-robotYaw!) * scale(0.18), rPos.dy + sin(-robotYaw!) * scale(0.18)), Paint()..color = Colors.cyanAccent..strokeWidth = 3);
      canvas.drawCircle(rPos, 3.5, Paint()..color = Colors.white);
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}