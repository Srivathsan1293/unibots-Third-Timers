// lib/apriltag_ffi.dart

import 'dart:ffi';
import 'dart:io';
import 'dart:typed_data';
import 'package:ffi/ffi.dart';
import 'package:vector_math/vector_math_64.dart';
import 'localization_helper.dart'; // Make sure this points to your corrected localization file

final class NativeDetection extends Struct {
  @Int32() external int id;
  @Double() external double tx;
  @Double() external double ty;
  @Double() external double tz;
  @Double() external double r00;
  @Double() external double r01;
  @Double() external double r02;
  @Double() external double r10;
  @Double() external double r11;
  @Double() external double r12;
  @Double() external double r20;
  @Double() external double r21;
  @Double() external double r22;
}

typedef DetectTagsC = Int32 Function(
    Pointer<Uint8> grayBytes, Int32 width, Int32 height,
    Double fx, Double fy, Double cx, Double cy, Double tagSize,
    Pointer<NativeDetection> outDetections, Int32 maxDetections,
    );

typedef DetectTagsDart = int Function(
    Pointer<Uint8> grayBytes, int width, int height,
    double fx, double fy, double cx, double cy, double tagSize,
    Pointer<NativeDetection> outDetections, int maxDetections,
    );

class AprilTagFFI {
  static final DynamicLibrary _nativeLib = Platform.isAndroid
      ? DynamicLibrary.open('libapriltag_native.so')
      : DynamicLibrary.process();

  static final DetectTagsDart _detectTags = _nativeLib
      .lookup<NativeFunction<DetectTagsC>>('detect_tags')
      .asFunction<DetectTagsDart>();

  // Use optional positional parameters so both main app and sandbox work.
  // We default tagSizeMeters to 0.10 (10cm) for the sandbox.
  static List<AprilTagDetection> runDetection(
      Uint8List yPlaneBytes,
      int width,
      int height,
      [double fx = 615.0, double fy = 615.0, double tagSizeMeters = 0.10]) {

    final int maxDets = 10;

    final Pointer<Uint8> bufferPointer = malloc.allocate<Uint8>(yPlaneBytes.length);
    final Pointer<NativeDetection> outDetectionsPointer = malloc.allocate<NativeDetection>(sizeOf<NativeDetection>() * maxDets);

    final Uint8List nativeBuffer = bufferPointer.asTypedList(yPlaneBytes.length);
    nativeBuffer.setAll(0, yPlaneBytes);

    final int count = _detectTags(
      bufferPointer, width, height,
      fx, fy, width / 2.0, height / 2.0, tagSizeMeters,
      outDetectionsPointer, maxDets,
    );

    final List<AprilTagDetection> results = [];
    for (int i = 0; i < count; i++) {
      final NativeDetection det = outDetectionsPointer[i];

      Matrix3 R = Matrix3.zero();
      R.setRow(0, Vector3(det.r00, det.r01, det.r02));
      R.setRow(1, Vector3(det.r10, det.r11, det.r12));
      R.setRow(2, Vector3(det.r20, det.r21, det.r22));

      results.add(AprilTagDetection(
        tagId: det.id,
        t: Vector3(det.tx, det.ty, det.tz),
        R: R,
      ));
    }

    malloc.free(bufferPointer);
    malloc.free(outDetectionsPointer);

    return results;
  }
}