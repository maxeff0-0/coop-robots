"""
Obstacle detection on the ESP32-CAM stream, sends drive commands to the robot ESP32.

How it works (floor-colour model):
  1. Look at a small patch right in front of the robot (bottom-centre) = "this is floor".
  2. Every pixel in the lower part of the image that looks different from that floor is "obstacle".
  3. Split the lower image into LEFT / CENTER / RIGHT zones and see which are blocked.
  4. Decide: F (go), L / R (turn toward the freer side), S (stop). Send over UDP.

Usage:
  python vision/obstacle_detector.py --selftest                      # no hardware needed
  python vision/obstacle_detector.py --source 0                      # laptop webcam, dry run
  python vision/obstacle_detector.py --source http://CAM_IP/stream   # camera, dry run
  python vision/obstacle_detector.py --source http://CAM_IP/stream --robot-ip ROBOT_IP   # live driving

Keys: c = recalibrate floor (point at CLEAR floor), q = quit
"""
import argparse
import collections
import socket
import sys
import time

import cv2
import numpy as np

W, H = 320, 240
ROI_TOP = 0.45            # ignore the top 45% of the image (walls, horizon)


def floor_model(lab):
    h, w = lab.shape[:2]
    patch = lab[int(h * 0.82):int(h * 0.97), int(w * 0.35):int(w * 0.65)]
    return patch.reshape(-1, 3).mean(axis=0)


def obstacle_mask(lab, mean, thresh):
    diff = lab.astype(np.float32) - mean
    dist = np.sqrt((0.5 * diff[..., 0]) ** 2 + diff[..., 1] ** 2 + diff[..., 2] ** 2)
    mask = (dist > thresh).astype(np.uint8) * 255
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def zone_ratios(mask):
    y0 = int(H * ROI_TOP)
    roi = mask[y0:, :]
    third = W // 3
    zones = [roi[:, :third], roi[:, third:2 * third], roi[:, 2 * third:]]
    return [float(np.count_nonzero(z)) / z.size for z in zones]


def decide(ratios, blocked_at):
    left, center, right = ratios
    if center < blocked_at:
        return "F"
    if left >= blocked_at and right >= blocked_at:
        return "S"
    return "L" if left < right else "R"


def process(frame, mean, thresh, blocked_at):
    frame = cv2.resize(frame, (W, H))
    lab = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2LAB)
    mask = obstacle_mask(lab, mean, thresh)
    ratios = zone_ratios(mask)
    return frame, mask, ratios, decide(ratios, blocked_at)


def draw(frame, mask, ratios, cmd, blocked_at):
    out = frame.copy()
    y0 = int(H * ROI_TOP)
    tint = np.zeros_like(out)
    tint[y0:, :, 2] = mask[y0:, :]               # obstacle pixels in red
    out = cv2.addWeighted(out, 1.0, tint, 0.5, 0)
    third = W // 3
    for i, r in enumerate(ratios):
        x0 = i * third
        col = (0, 0, 255) if r >= blocked_at else (0, 200, 0)
        cv2.rectangle(out, (x0, y0), (x0 + third - 1, H - 1), col, 2)
        cv2.putText(out, f"{r:.2f}", (x0 + 5, y0 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    cv2.rectangle(out, (int(W * .35), int(H * .82)), (int(W * .65), int(H * .97)), (255, 255, 0), 1)
    cv2.putText(out, f"CMD: {cmd}", (5, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return out


def selftest():
    print("Self-test on synthetic frames (no camera needed)...")
    rng = np.random.default_rng(0)

    def floor_frame():
        f = np.full((H, W, 3), (120, 130, 125), np.uint8)
        return np.clip(f + rng.integers(-6, 7, f.shape), 0, 255).astype(np.uint8)

    clear = floor_frame()
    mean = floor_model(cv2.cvtColor(clear, cv2.COLOR_BGR2LAB))
    cases = {
        "clear floor": (clear, "F"),
    }
    f = floor_frame(); f[120:200, 120:200] = (30, 30, 200)
    cases["box in front"] = (f, None)       # expect L or R
    f = floor_frame(); f[120:200, 10:90] = (30, 30, 200)
    cases["box on left"] = (f, "F")      # left zone blocked, centre free -> keep going
    f = floor_frame(); f[110:230, 20:300] = (30, 30, 200)
    cases["wall everywhere"] = (f, "S")

    ok = True
    for name, (img, expect) in cases.items():
        _, _, ratios, cmd = process(img, mean, 25, 0.20)
        print(f"  {name:16s} ratios={[round(r, 2) for r in ratios]} -> {cmd}")
        if name == "clear floor" and cmd != "F": ok = False
        if name == "box in front" and cmd not in ("L", "R"): ok = False
        if name == "wall everywhere" and cmd != "S": ok = False
    print("SELFTEST", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


def open_source(src):
    cap = cv2.VideoCapture(int(src) if str(src).isdigit() else src)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="webcam index or http://CAM_IP/stream")
    ap.add_argument("--robot-ip", default=None, help="robot ESP32 IP; omit for dry run")
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--thresh", type=float, default=25, help="colour difference to count as obstacle")
    ap.add_argument("--blocked", type=float, default=0.20, help="zone fraction that counts as blocked")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if a.robot_ip else None
    print("LIVE DRIVING to", a.robot_ip if a.robot_ip else "off (dry run)")
    cap = open_source(a.source)
    mean, warm, history = None, 0, collections.deque(maxlen=3)
    stable = "S"

    while True:
        ok, frame = cap.read()
        if not ok:
            print("No frame, reconnecting...")
            if sock: sock.sendto(b"S", (a.robot_ip, a.port))
            cap.release(); time.sleep(1); cap = open_source(a.source)
            continue

        small = cv2.resize(frame, (W, H))
        if mean is None or warm < 5:                      # auto-calibrate on first frames
            mean = floor_model(cv2.cvtColor(small, cv2.COLOR_BGR2LAB)); warm += 1

        frame_s, mask, ratios, cmd = process(small, mean, a.thresh, a.blocked)
        history.append(cmd)
        top, count = collections.Counter(history).most_common(1)[0]
        if count >= 2:
            stable = top                                   # majority of last 3 frames
        if sock:
            sock.sendto(stable.encode(), (a.robot_ip, a.port))

        cv2.imshow("obstacle detector (c = recalibrate, q = quit)", draw(frame_s, mask, ratios, stable, a.blocked))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("c"):
            mean = floor_model(cv2.cvtColor(small, cv2.COLOR_BGR2LAB)); print("Recalibrated floor")

    if sock: sock.sendto(b"S", (a.robot_ip, a.port))
    cap.release(); cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
