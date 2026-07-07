#!/usr/bin/env python3
import cv2
import numpy as np


def white_fraction(frame, thresh):
    """Calculates the fraction of pixels that match or exceed the threshold across

    all three BGR channels.
    """
    if frame is None:
        return 0.0
    arr = np.asarray(frame)
    if arr.ndim != 3:
        return 0.0

    # A pixel is considered white floor if B >= thresh, G >= thresh, and R >= thresh
    white_mask = np.all(arr >= thresh, axis=2)
    fraction = float(white_mask.mean())

    # Return both the numeric fraction and the mask itself for visualization
    binary_mask_visual = (white_mask * 255).astype(np.uint8)
    return fraction, binary_mask_visual


def main():
    # Initialize the camera (adjust index if you have multiple cameras connected)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera stream.")
        return

    # Create an interactive GUI window
    window_name = "Collision Avoidance Calibration"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Trackbar callback function (does nothing, values are read directly)
    def nothing(x):
        pass

    # Create dynamic sliders for your FSM parameters
    # Threshold goes from 0-255 (Default initial: 180)
    cv2.createTrackbar("White Thresh", window_name, 180, 255, nothing)
    # Fraction goes from 0-100 to represent percentages (Default initial: 30%)
    cv2.createTrackbar("Min White Frac (%)", window_name, 30, 100, nothing)

    print("\n" + "=" * 50)
    print("COLLISION AVOIDANCE CALIBRATION TOOL")
    print("=" * 50)
    print("1. Clear the floor and check the 'Current White Frac'.")
    print("2. Place an obstacle at your minimum safe stopping distance.")
    print("3. Adjust 'White Thresh' until the floor is white and the obstacle is dark.")
    print("4. Set 'Min White Frac' safely above the obstacle fraction but below the clear floor fraction.")
    print("Press 'q' or 'ESC' to exit and print your calibrated values.\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to grab frame from camera.")
            break

        # Read current positions of the trackbars
        thresh = cv2.getTrackbarPos("White Thresh", window_name)
        min_frac_pct = cv2.getTrackbarPos("Min White Frac (%)", window_name)
        min_frac = min_frac_pct / 100.0

        # Calculate tracking fractions
        current_frac, mask_visual = white_fraction(frame, thresh)
        is_obstacle = current_frac < min_frac

        # Prepare telemetry text overlay
        status_text = "STATUS: OBSTACLE DETECTED! (STOP/TURN)" if is_obstacle else "STATUS: CLEAR PATH (FORWARD)"
        color = (0, 0, 255) if is_obstacle else (0, 255, 0)

        # Overlay statistics onto the raw image feed
        cv2.putText(frame, f"Current White Frac: {current_frac:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Target Min Frac   : {min_frac:.2f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, status_text, (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # Combine the original frame and the binary mask side-by-side for comparison
        mask_3d = cv2.cvtColor(mask_visual, cv2.COLOR_GRAY2BGR)
        combined_view = np.hstack((frame, mask_3d))

        # Show the combined feeds
        cv2.imshow(window_name, combined_view)

        # Handle exiting
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            print("\n" + "=" * 50)
            print("RECOMMENDED PARAMETERS:")
            print("=" * 50)
            print(f"  white_thresh   = {thresh}")
            print(f"  min_white_frac = {min_frac:.2f}")
            print("=" * 50)
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()