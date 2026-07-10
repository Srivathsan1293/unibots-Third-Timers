import cv2
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class BBox:
    """Bounding box for a detected ball."""
    x: int       # top-left corner
    y: int       # top-left corner
    w: int       # width
    h: int       # height
    cx: int      # circle centre x
    cy: int      # circle centre y
    radius: int  # circle radius


class BallDetector(ABC):
    """
    Abstract base class for ball detectors.

    Subclasses implement `_preprocess` to isolate pixels of interest,
    then call the shared `detect` pipeline which returns a list of BBoxes.
    """

    def detect(self, frame: np.ndarray) -> list[BBox]:
        """
        Run the full detection pipeline on a BGR frame.

        Returns a list of BBox objects, one per detected ball.
        """
        processed = self._preprocess(frame)
        circles   = self._find_circles(processed)
        return self._to_bboxes(circles)

    @abstractmethod
    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """
        Convert `frame` into a single-channel image ready for circle detection.
        Only pixels belonging to the target ball type should be non-zero.
        """
        ...

    def _find_circles(self, gray: np.ndarray):
        """Apply HoughCircles to a pre-processed single-channel image."""
        blurred = cv2.GaussianBlur(gray, (11, 11), 2)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp        = 1.2,
            minDist   = 40,
            param1    = 100,
            param2    = 28,
            minRadius = 10,
            maxRadius = 300,
        )
        if circles is not None:
            return np.round(circles[0]).astype(int)
        return []

    @staticmethod
    def _to_bboxes(circles) -> list[BBox]:
        bboxes = []
        for cx, cy, r in circles:
            bboxes.append(BBox(
                x      = int(cx - r),
                y      = int(cy - r),
                w      = int(2 * r),
                h      = int(2 * r),
                cx     = int(cx),
                cy     = int(cy),
                radius = int(r),
            ))
        return bboxes



class OrangeBallDetector(BallDetector):
    """
    Detects orange balls via HSV masking + Hough Circle Transform.

    Pre-processing chain to suppress false positives:
      1. Bilateral filter  — smooths internal texture while keeping the ball
                             boundary sharp (unlike Gaussian, which blurs edges).
      2. HSV colour mask   — isolates orange pixels across both hue bands.
      3. Morphological ops — fills holes, removes speckle inside the mask.
      4. Gaussian blur     — softens remaining edge noise before Hough.

    Hough params are tightened (higher param2) relative to the base class
    so only well-supported circles survive accumulation.
    """

    _ORANGE_LOW_1  = np.array([0,   130,  80])
    _ORANGE_HIGH_1 = np.array([18,  255, 255])
    _ORANGE_LOW_2  = np.array([160, 130,  80])
    _ORANGE_HIGH_2 = np.array([180, 255, 255])

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        # Step 1 — bilateral filter: kills texture noise, preserves the ball edge
        smooth = cv2.bilateralFilter(frame, d=9, sigmaColor=75, sigmaSpace=75)

        # Step 2 — HSV mask
        hsv  = cv2.cvtColor(smooth, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self._ORANGE_LOW_1,  self._ORANGE_HIGH_1),
            cv2.inRange(hsv, self._ORANGE_LOW_2,  self._ORANGE_HIGH_2),
        )

        # Step 3 — morphological clean-up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)

        # Step 4 — masked grayscale + Gaussian blur to smooth residual noise
        gray   = cv2.cvtColor(smooth, cv2.COLOR_BGR2GRAY)
        masked = cv2.bitwise_and(gray, mask)
        return cv2.GaussianBlur(masked, (15, 15), 3)

    def _find_circles(self, gray: np.ndarray):
        """Stricter Hough params than the base class to cut false positives."""
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp        = 1.2,
            minDist   = 50,   # require more separation between candidates
            param1    = 80,   # Canny high threshold
            param2    = 40,   # higher accumulator threshold → fewer, better circles
            minRadius = 10,
            maxRadius = 300,
        )
        if circles is not None:
            return np.round(circles[0]).astype(int)
        return []


class MetalBallDetector(BallDetector):
    """Detects shiny metal balls: circular silhouette with a bright specular
    highlight in the core and darker shaded edges."""
 
    def __init__(self):
        self._shape = None
 
    def _preprocess(self, frame: np.ndarray):
        self._shape = frame.shape
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
 
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        enhanced_gray = clahe.apply(gray)
 
        blurred = cv2.GaussianBlur(enhanced_gray, (7, 7), 1.5)
        return blurred, enhanced_gray
 
    def _find_circles(self, frames):
        blurred, enhanced_gray = frames
        h, w = self._shape[:2]
        min_r = max(1, int(w * 0.015))
        max_r = int(w * 0.15)
 
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT_ALT,
            dp=1.5, minDist=min_r * 2,
            param1=100, param2=0.8,
            minRadius=min_r, maxRadius=max_r,
        )
 
        validated = []
        if circles is not None:
            for cx, cy, r in np.round(circles[0, :]).astype(int):
                # FIX: masks must be single-channel 8-bit (CV_8U), so use the
                # height/width only and force uint8. self._shape is (h, w, 3),
                # and np.zeros defaults to float64, which crashed minMaxLoc.
                core_mask = np.zeros((h, w), dtype=np.uint8)
                cv2.circle(core_mask, (cx, cy), int(r * 0.5), 255, -1)
 
                edge_mask = np.zeros((h, w), dtype=np.uint8)
                cv2.circle(edge_mask, (cx, cy), int(r * 0.95), 255, -1)
                cv2.circle(edge_mask, (cx, cy), int(r * 0.6), 0, -1)
 
                _, max_core_val, _, _ = cv2.minMaxLoc(enhanced_gray, mask=core_mask)
                edge_mean = cv2.mean(enhanced_gray, mask=edge_mask)[0]
 
                # Bright specular spot inside, clearly brighter than the shaded rim.
                if max_core_val > 150 and max_core_val > edge_mean * 1.3:
                    validated.append((cx, cy, r))
        return validated

class CombinedBallDetector:
    """Runs both detectors and resolves overlaps. Returns a dict:
        {"orange": [BBox, ...], "metal": [BBox, ...]}
    Any metal box that intersects an orange box is discarded (orange wins)."""
 
    def __init__(self, orange=None, metal=None):
        # setting params to None fine for when individual ball detectors don't need to be calibrated

        self.orange = orange or OrangeBallDetector()
        self.metal = metal or MetalBallDetector()
 
    def detect(self, frame: np.ndarray) -> dict:
        orange_boxes = self.orange.detect(frame)
        metal_boxes = self.metal.detect(frame)
        metal_boxes = [m for m in metal_boxes
                       if not any(self._intersect(m, o) for o in orange_boxes)]
        
        return {"orange": orange_boxes, "metal": metal_boxes}
 
    @staticmethod
    def _intersect(a: BBox, b: BBox) -> bool:
        """Axis-aligned box overlap test."""
        return (a.x < b.x + b.w and b.x < a.x + a.w and
                a.y < b.y + b.h and b.y < a.y + a.h)
