"""
Unused
Can resize 10x10 images to 256x256. Only needs to be run once (when changing april tags).
No longer needs to be used.
"""


import glob
import os

from PIL import Image

SRC_DIR = os.path.join(os.path.dirname(__file__), "images/")
# SRC_DIR = r"C:\Users\ariya\Documents\WeBots\unibots_odometry_and_rgb_cam\controllers\ball_detection\webots_env\april_tags"
TARGET = 256                 # power of two; 512 if you want even more crispness
PAD_COLOR = (255, 255, 255)  # white quiet zone


def process(path):
    img = Image.open(path).convert("RGB")
    w, h = img.size
    factor = TARGET // max(w, h)          # largest integer upscale that fits (25 for 10 -> 256)
    if factor < 1:
        factor = 1
    up = img.resize((w * factor, h * factor), Image.NEAREST)   # crisp, uniform cells
    canvas = Image.new("RGB", (TARGET, TARGET), PAD_COLOR)      # power-of-two canvas
    off = ((TARGET - up.width) // 2, (TARGET - up.height) // 2)
    canvas.paste(up, off)
    canvas.save(path)                     # overwrite in place


def main():
    paths = sorted(glob.glob(os.path.join(SRC_DIR, "tag36_11_*.png")))
    if not paths:
        print(f"No tag PNGs found in {SRC_DIR}")
        return
    for p in paths:
        process(p)
    print(f"Processed {len(paths)} tags to {TARGET}x{TARGET} (nearest-neighbor, white-padded).")


if __name__ == "__main__":
    main()