# Project Overview: Unibots Navigation

This is a real-time mobile application for ball detection using Computer Vision (CV) techniques. The core logic involves receiving raw camera frames, performing image processing (Sobel filtering and TFLite inference), clustering detections, and deriving high-level navigation commands based on the cluster's position relative to the screen center.

## 🚨 High-Signal Operational Gotchas & Conventions

### 1. Camera & Data Flow
*   **Frame Source:** The system does not use simple image widgets; it must process raw `CameraImage` planes (YUV format) received via `CameraController.startImageStream`. This is the primary input for CV processing (`main.dart:77`).
*   **Multi-threaded Processing:** All computationally intensive tasks—TFLite inference and filtering/processing—MUST occur in a separate Isolate thread (`lib/DetectionSystem`) to prevent UI jank and frame drops. The main thread only handles receiving results and updating the UI HUD.

### 2. Computer Vision Pipeline (CV) Details
The detection system is complex and relies on sequential processing:
1.  **YUV Planes:** Raw image data must be passed as three separate `Uint8List` planes (Y, U, V). The Y plane is used for luminance/background analysis (Sobel), while the U/V planes are used for color-based masks (Orange ball detection).
2.  **Detection Types:** Two distinct methods detect balls:
    *   **Orange Balls (YOLO):** Uses TFLite inference on cropped ROIs, requiring manual coordinate scaling and tensor preparation (`lib/Detection.dart:317`). The `confidence` score is critical for determining validity.
    *   **Metal Balls (Fallback):** Detects contrast changes using Sobel filters combined with a dilation step to find potential bounding boxes in the Y plane. This method serves as a robust fallback if YOLO fails.
3.  **Coordinate System:** All coordinates passed internally (`absX1`, `absY1`, etc.) are normalized pixel values from the *unscaled* camera stream size. The UI scales these based on the current preview aspect ratio, but the logic must assume the full resolution geometry ($\text{Width}=1280$, $\text{Height}=720$) for calculations like ball diameter and distance.

### 3. Navigation Command Generation
The system does not calculate direction based on individual detections, but by grouping them:
*   **Clustering:** Detections are grouped into `DetectionCluster` instances by checking if their average X-coordinate falls within a horizontal threshold (approx $20\%$ of the image width) relative to previously established groups (`lib/Detection.dart:153`).
*   **Prioritization:** The cluster with the highest cumulative score (Metal=4 points, Orange=3 points) and then the minimum distance is selected as the target.
*   **Command Logic:** Direction (Left/Right/Forward) is determined by comparing the *target cluster's average X-coordinate* against two deadzones defined relative to the assumed 1280 width (`lib/Detection.dart:177`).

## 🛠️ Development Commands & Workflow
*   **Setup:** Always run `flutter pub get` after modifying dependencies in `pubspec.yaml`.
*   **Dependencies:** Key packages include `camera`, `tflite_flutter`, and standard Flutter utilities.
*   **Build/Run:** Use standard Flutter commands (e.g., `flutter run`) but remember that the application relies on platform-specific assets (`assets/model_quantized.tflite`).

## 💡 Areas of Potential Failure
1.  **Coordinate Mismatch:** Any change to the camera resolution or aspect ratio requires updating the hardcoded assumption of `1280` width and `720` height in the navigation logic, as scaling factors are based on this default geometry (`main.dart:165`, `DetectionSystem._calculateNavigation`).
2.  **TFLite Compatibility:** The TFLite inference code is highly dependent on the input shape (`[1, 320, 320, 3]`) and the specific YUV-to-RGB conversion logic used in the isolate thread. If the model changes, this section must be updated first.