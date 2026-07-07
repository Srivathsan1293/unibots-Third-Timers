"""
pi_fsm_hybrid.py

Combined FSM and Communication Control.
Integrates the advanced behavior states (RETURN, APPROACH_BOX, Obstacle Avoidance) 
with the robust ADB-tunneled network link and verbose serial logging.
"""

import json
import math
import threading
import time
import subprocess
import cv2
import serial
import numpy as np
from detection import PiBallNavigator

GRAVITY = 9.81


def white_fraction(frame, thresh=180):
    """Fraction of pixels that are bright and roughly neutral (white floor).
    frame is an OpenCV BGR uint8 array. A pixel counts as white when all three
    channels are >= thresh. Returns 0.0..1.0; 1.0 means a fully clear white view.
    """
    if frame is None:
        return 1.0
    arr = np.asarray(frame)
    if arr.ndim != 3:
        return 1.0
    white = np.all(arr >= thresh, axis=2)
    return float(white.mean())


# ── physics ────────────────────────────────────────────────────────────────
def compute_launch_speed(distance_m, angle_rad, height_diff_m, g=GRAVITY):
    c = math.cos(angle_rad)
    drop = distance_m * math.tan(angle_rad) - height_diff_m
    denom = 2.0 * c * c * drop
    if denom <= 0:
        return None
    return math.sqrt(g * distance_m * distance_m / denom)


def spring_setting(speed, k=8.0, offset=0.0, lo=0, hi=255):
    if speed is None:
        return None
    return max(lo, min(hi, int(round(offset + k * speed * speed))))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ── transport ──────────────────────────────────────────────────────────────
class DriveESP:
    def __init__(self, port="/dev/ttyUSB_DRIVE", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.last_cmd = None

    def send(self, cmd, current_state):
        if cmd != self.last_cmd:
            print(f"[{current_state.upper()}] -> Sending to DRIVE ESP: '{cmd}'")
            self.ser.write(f"{cmd}\n".encode())
            self.last_cmd = cmd


class LauncherESP:
    def __init__(self, port="/dev/ttyUSB_LAUNCHER", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._last = ""

    def tension(self, setting, current_state):
        print(f"[{current_state.upper()}] -> Sending to LAUNCHER ESP: 'T,{setting}' (Tensioning)")
        self.ser.write(f"T,{setting}\n".encode())

    def release(self, current_state):
        print(f"[{current_state.upper()}] -> Sending to LAUNCHER ESP: 'RELEASE' (Firing)")
        self.ser.write(b"RELEASE\n")

    def _poll(self):
        line = self.ser.readline().decode(errors="ignore").strip()
        if line:
            self._last = line
        return self._last

    def is_ready(self):
        return self._poll() == "READY"

    def is_reset(self):
        return self._poll() == "RESET"


class PhoneLink:
    def __init__(self, host="0.0.0.0", port=5000):
        self._latest = None
        self._lock = threading.Lock()
        self._host, self._port = host, port

    def start(self):
        threading.Thread(target=self._serve, daemon=True).start()

    def _apply_adb_tunnel(self):
        try:
            result = subprocess.run(['adb', 'devices'], capture_output=True, text=True)
            if '\tdevice' in result.stdout:
                subprocess.run(['adb', 'reverse', f'tcp:{self._port}', f'tcp:{self._port}'], check=True)
                return True
            return False
        except FileNotFoundError:
            print("Error: 'adb' utility not found on this system.")
            return False

    def _serve(self):
        import socket
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._host, self._port))
        srv.settimeout(2.0)
        srv.listen(1)
        
        while True:
            if not self._apply_adb_tunnel():
                time.sleep(2)
                continue
                
            try:
                conn, _ = srv.accept()
                conn.settimeout(None) 
                buf = ""
                with conn:
                    while True:
                        data = conn.recv(4096)
                        if not data:
                            break
                        buf += data.decode(errors="ignore")
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            line = line.strip()
                            if line.startswith("NAV,"):
                                parts = line.split(",")
                                if len(parts) >= 5:
                                    try:
                                        msg = {
                                            "dir": parts[1],
                                            "x": float(parts[2]),
                                            "y": float(parts[3]),
                                            "yaw": float(parts[4])
                                        }
                                        with self._lock:
                                            self._latest = msg
                                    except ValueError:
                                        continue
            except socket.timeout:
                pass
            except (ConnectionResetError, BrokenPipeError):
                pass

    def latest(self):
        with self._lock:
            return dict(self._latest) if self._latest else None


# ── FSM ────────────────────────────────────────────────────────────────────
class PiFSM:
    FOLLOW, APPROACH_BOX, ALIGN, RESET_WAIT, RETURN = \
        "follow", "approach_box", "align", "reset_wait", "return"

    def __init__(self, drive, launcher, detector, *,
                 launch_radius=0.70,
                 box_offset_back=0.25,
                 start_samples=4,
                 launch_align_tol=0.10,
                 drive_align_tol=0.35,
                 launch_angle=math.radians(45),
                 net_height_diff=0.0,
                 match_limit=180.0,      # s: hard return to start after this
                 home_tol=0.10,          # m: "parked at start" tolerance
                 white_thresh=180,       # per-channel brightness for "white floor"
                 min_white_frac=0.30,    # below this white fraction -> obstacle ahead
                 avoid_turn="d"):        # which way to turn to avoid an obstacle
        self.drive = drive
        self.launcher = launcher
        self.detector = detector
        self.launch_radius = launch_radius
        self.box_offset_back = box_offset_back
        self.start_samples = start_samples
        self.launch_align_tol = launch_align_tol
        self.drive_align_tol = drive_align_tol
        self.launch_angle = launch_angle
        self.net_height_diff = net_height_diff
        
        self.match_limit = match_limit
        self.home_tol = home_tol
        self.white_thresh = white_thresh
        self.min_white_frac = min_white_frac
        self.avoid_turn = avoid_turn

        self.state = self.FOLLOW
        self.start_pose = None
        self.box = None
        self._samples = []
        self.match_start = None
        self._last_state_print = None

    def _change_state(self, new_state, reason=""):
        print(f"\n>>>> STATE TRANSITION: {self.state.upper()} -> {new_state.upper()} {f'({reason})' if reason else ''} <<<<\n")
        self.state = new_state

    def _accumulate_start(self, x, y, yaw):
        self._samples.append((x, y, math.cos(yaw), math.sin(yaw)))
        if len(self._samples) >= self.start_samples:
            n = len(self._samples)
            sx = sum(s[0] for s in self._samples) / n
            sy = sum(s[1] for s in self._samples) / n
            syaw = math.atan2(sum(s[3] for s in self._samples) / n,
                              sum(s[2] for s in self._samples) / n)
            self.start_pose = (sx, sy, syaw)
            self.box = (sx - self.box_offset_back * math.cos(syaw),
                        sy - self.box_offset_back * math.sin(syaw))
            print(f"[INIT] Map Anchored. Start Pose: {self.start_pose}, Box Target: {self.box}")

    def _dist_box(self, x, y):
        return math.hypot(self.box[0] - x, self.box[1] - y)

    def _begin_launch(self, x, y):
        dist = self._dist_box(x, y)
        speed = compute_launch_speed(dist, self.launch_angle, self.net_height_diff)
        setting = spring_setting(speed)
        print(f"[{self.state.upper()}] Distance to Target box: {dist:.2f}m. Calculated Metric Speed: {speed:.2f}")
        self.launcher.tension(setting, self.state)

    def _err_to_box(self, x, y, yaw):
        return _wrap(math.atan2(self.box[1] - y, self.box[0] - x) - yaw)

    def _obstacle_ahead(self, frame):
        """True when the Pi camera view is not mostly white floor."""
        return white_fraction(frame, self.white_thresh) < self.min_white_frac

    def _drive_toward(self, x, y, yaw, target):
        """Bang-bang straight-line drive: turn toward target, else go forward."""
        err = _wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
        if abs(err) > self.drive_align_tol:
            return self._turn_char(err)
        return "w"

    @staticmethod
    def _turn_char(err):
        # 'a' is assumed to turn CCW (decreasing yaw). Swap if needed.
        return "d" if err > 0 else "a"

    def update(self, phone_msg, pi_frame, now=None):
        pi_cmd, collected = self.detector.get_wasd_command(pi_frame)
        obstacle = self._obstacle_ahead(pi_frame)

        # Remap legacy detection 's' (stop) to real-world '0'
        if pi_cmd == 's':
            pi_cmd = '0'

        x = phone_msg.get("x")
        if self.start_pose is None and x is not None:
            self._accumulate_start(x, phone_msg["y"], phone_msg["yaw"])
            
        if self.match_start is None and self.start_pose is not None and now is not None:
            self.match_start = now
            print(f"[INIT] Match started at {self.match_start}")

        # Periodic baseline state monitor printout
        if self.state != getattr(self, '_last_state_print', None):
            print(f"[FSM STATE ACTIVE] -> {self.state.upper()}")
            self._last_state_print = self.state

        # ---- match deadline: hard override; once in RETURN it never leaves ----
        if (self.match_start is not None and now is not None
                and now - self.match_start > self.match_limit):
            if self.state != self.RETURN:
                self._change_state(self.RETURN, "Match limit reached")

        if self.state == self.RETURN:
            if self.start_pose is None or x is None:
                self.drive.send("a", self.state)                            # spin to find a tag
                return
            y, yaw = phone_msg["y"], phone_msg["yaw"]
            hx, hy = self.start_pose[0], self.start_pose[1]
            if math.hypot(hx - x, hy - y) <= self.home_tol:
                self.drive.send("0", self.state)                            # parked at start
                return
            char = self._drive_toward(x, y, yaw, (hx, hy))
            if char == "w" and obstacle:
                char = self.avoid_turn                                      # avoid on the way home
            self.drive.send(char, self.state)
            return

        # ---- FOLLOW ----
        if self.state == self.FOLLOW:
            if collected is not None and self.box is not None:
                print(f"[{self.state.upper()}] BALL DETECTED AS INTAKED! Short range ball event: '{collected}'")
                self.drive.send("0", self.state)
                
                if self._dist_box(x, phone_msg["y"]) > self.launch_radius:
                    self._change_state(self.APPROACH_BOX, "Out of launch range")
                else:
                    self._begin_launch(x, phone_msg["y"])
                    self._change_state(self.ALIGN, "In launch range, starting orientation lock")
                return
            
            cmd = pi_cmd if pi_cmd != "0" else phone_msg.get("dir", "0")
            if cmd == "0":
                self.drive.send("a", self.state)                            # search: spin in place
            elif cmd == "w" and obstacle:
                self.drive.send(self.avoid_turn, self.state)                # collision avoidance
            else:
                self.drive.send(cmd, self.state)
            return

        if x is None:
            print(f"[{self.state.upper()}] WARNING: Lost Phone Link Telemetry stream!")
            self.drive.send("0", self.state)
            return
            
        y, yaw = phone_msg["y"], phone_msg["yaw"]

        # ---- APPROACH_BOX (bang-bang straight line) ----
        if self.state == self.APPROACH_BOX:
            dist = self._dist_box(x, y)
            if dist <= self.launch_radius:
                self.drive.send("0", self.state)
                self._begin_launch(x, y)
                self._change_state(self.ALIGN, f"Inside target bracket. Dist: {dist:.2f}m")
                return
            
            err = self._err_to_box(x, y, yaw)
            if abs(err) > self.drive_align_tol:
                turn = self._turn_char(err)
                print(f"[{self.state.upper()}] Navigating to Box. Dist: {dist:.2f}m, Heading Err: {err:.2f} rad. Aligning chassis first.")
                self.drive.send(turn, self.state)
            elif obstacle:
                self.drive.send(self.avoid_turn, self.state)         # obstacle: turn instead of forward
            else:
                self.drive.send("w", self.state)
            return

        # ---- ALIGN ----
        if self.state == self.ALIGN:
            err = self._err_to_box(x, y, yaw)
            facing = abs(err) < self.launch_align_tol
            launcher_ready = self.launcher.is_ready()
            
            print(f"[{self.state.upper()}] Targeting precision sweep. Heading Err: {err:.2f}/{self.launch_align_tol} rad. Launcher Tensioned Check: {launcher_ready}", end='\r')
            
            if facing and launcher_ready:
                print(f"\n[{self.state.upper()}] CRITERIA MET. Target alignment locked and Launcher ready.")
                self.drive.send("0", self.state)
                self.launcher.release(self.state)
                self._change_state(self.RESET_WAIT, "Fired payload")
            elif not facing:
                self.drive.send(self._turn_char(err), self.state)
            else:
                self.drive.send("0", self.state)                     # Facing target perfectly, wait for tension feedback
            return

        # ---- RESET_WAIT ----
        if self.state == self.RESET_WAIT:
            self.drive.send("0", self.state)
            if self.launcher.is_reset():
                self._change_state(self.FOLLOW, "Launcher mechanical cycle finished and safe")
            return


def main():
    phone = PhoneLink()
    phone.start()
    
    drive = DriveESP("/dev/ttyUSB_DRIVE")
    launcher = LauncherESP("/dev/ttyUSB_LAUNCHER")
    detector = PiBallNavigator(debug=False)
    
    cap = cv2.VideoCapture(0)
    fsm = PiFSM(drive, launcher, detector)
    
    print("[SYSTEM READY] Verbose print-testing active. Connect phone app...")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        fsm.update(phone.latest() or {}, frame, time.time())


if __name__ == "__main__":
    main()