# ESP32-CAM Live Stream + Obstacle Detection: Setup Guide

## What you're building

```
ESP32-CAM (on robot)  --WiFi video-->  Laptop (Python + OpenCV)
                                            |  detects obstacles, decides F / L / R / S
                                            v  UDP command (1 letter)
ESP32 robot driver  <--------WiFi-----------+
  -> TB6612 -> motors
```

Two separate boards: the **ESP32-CAM** only streams video. The **ESP32** only drives motors. The laptop does the thinking.

## 0. Before you start

- **Use a phone hotspot (2.4 GHz) or your home router.** ESP32 boards do not do 5 GHz. College/VIT WiFi usually blocks devices from talking to each other, so it will not work there.
- Connect the **laptop to the same hotspot/router** as both boards.
- Install Arduino IDE 2.x, Python 3.11, and pip install the requirements (`pip install -r requirements.txt`).
- Arduino IDE: File > Preferences > Additional boards manager URLs, add
  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
  then Tools > Board > Boards Manager > install **esp32 by Espressif** (version 2.0.17 or 3.x).

## 1. Flash the ESP32-CAM

File: `firmware/esp32cam_stream/esp32cam_stream.ino`

1. Edit `WIFI_SSID` and `WIFI_PASS` at the top.
2. Tools > Board > **AI Thinker ESP32-CAM**. Pick the right COM port.
3. **If you have an ESP32-CAM-MB base board:** just plug in USB and upload.
   **If you use an FTDI adapter** (no USB port on the CAM):

| FTDI | ESP32-CAM |
|---|---|
| 5V | 5V |
| GND | GND |
| TX | U0R |
| RX | U0T |
| (jumper) | IO0 to GND, **only while uploading** |

4. Click Upload. When it says "Connecting...", press the small RESET button on the CAM.
5. After upload: **remove the IO0-GND jumper**, press RESET, open Serial Monitor at **115200**.
6. It prints `Stream ready: http://192.168.x.x/stream`. Write that IP down.

**Test:** open `http://<that-ip>/` in your laptop browser. You should see live video. If you do, the hard part is done.

**Power matters:** the CAM draws spikes of 300 mA+. A weak USB port or long thin wires cause "Brownout detector" resets or "Camera init failed". Use a good cable, or 5V from your LM2596 (set to 5V first), with a common ground.

## 2. Flash the robot ESP32

File: `firmware/esp32_robot_udp/esp32_robot_udp.ino`

1. Edit `WIFI_SSID` and `WIFI_PASS`.
2. Board = **ESP32 Dev Module**. Upload.
3. Serial Monitor (115200) prints `Robot ready. IP: 192.168.x.x`. Write that IP down too.

Wiring (same as the motor test):

| TB6612 | ESP32 |
|---|---|
| VCC | 3.3V |
| GND | GND (also battery negative, common ground) |
| VM | motor battery positive (5-6V) |
| AIN1 / AIN2 / PWMA | GPIO 26 / 27 / 25 |
| BIN1 / BIN2 / PWMB | GPIO 14 / 32 / 13 |
| STBY | GPIO 33 |
| A01/A02 | left motors (both in parallel) |
| B01/B02 | right motors (both in parallel) |

Commands the robot understands: `F` forward, `B` back, `L` spin left, `R` spin right, `S` stop.
**Failsafe:** if no command arrives for 0.5 s, the robot stops by itself.
`SPEED` at the top of the sketch is 150 of 255. Keep it low for the first tests.

## 3. Run the detector on the laptop

From the repo root:

```bash
# 0) sanity check, no hardware (should print SELFTEST PASSED)
python vision/obstacle_detector.py --selftest

# 1) camera only, robot does NOT move (dry run)
python vision/obstacle_detector.py --source http://CAM_IP/stream

# 2) live driving
python vision/obstacle_detector.py --source http://CAM_IP/stream --robot-ip ROBOT_IP
```

No ESP32-CAM yet? Test the logic with your laptop webcam: `--source 0`.

**How to use it:**
1. Point the camera at **clear floor** when it starts. It learns the floor colour from the yellow box at the bottom-centre of the image.
2. Put an object (a box, a bottle, a book) in front. Obstacle pixels turn red and the zone box turns red.
3. The command shown at the bottom (F / L / R / S) is what the robot receives.
4. Press `c` to recalibrate on clear floor if lighting changes. Press `q` to quit.

## 4. Test safely (do this order)

1. **Dry run first.** Watch the command change as you move an object in front of the camera.
2. **Wheels lifted** off the table, live driving. Check that L makes the left wheels go back and R the right wheels go back. If a side spins the wrong way, swap that side's motor wires.
3. Put the robot on the floor, with a clear space and your hand near the battery switch.
4. Press `q` or close the terminal and the robot stops within half a second.

## 5. Tuning

| Problem | Fix |
|---|---|
| Sees obstacles that aren't there (shadows, tile patterns) | raise `--thresh` (try 35, 45) |
| Misses obstacles close in colour to the floor | lower `--thresh` (try 18), improve lighting |
| Too jumpy | raise `--blocked` (try 0.30) |
| Reacts too late | lower `--blocked` (try 0.12), or tilt the camera down a bit |
| Lag in the video | keep `FRAMESIZE_QVGA`, put the laptop close to the router/hotspot |

## 6. Troubleshooting

- **Camera init failed 0x105 / 0x20001:** reseat the ribbon cable (the latch must be down), use better power.
- **Brownout detector triggered:** power problem, not code.
- **Upload stuck on "Connecting...":** IO0 not grounded, or you didn't press RESET.
- **Browser shows nothing but IP prints:** laptop on a different network, or the router isolates devices. Use a phone hotspot.
- **Compile error about `pin_sccb_sda`:** rename to `pin_sscb_sda` and `pin_sscb_scl`.
- **cv2 can't open the stream:** check the URL ends with `/stream` and works in the browser first.
- **Robot ignores commands:** check the robot IP, that laptop and robot share WiFi, and the Windows firewall (allow Python on private networks).

## 7. Honest limitations (say this in your review)

- This is a **floor-colour method with a single camera**. It detects things that look *different from the floor*; it does not measure distance.
- It struggles with patterned or shiny floors, strong shadows, and obstacles the same colour as the floor.
- WiFi video adds roughly 100-300 ms of delay, so keep the speed low.
- Planned fix: use the **VL53L0X** distance sensor on the robot as an emergency stop, so the camera handles avoidance and the ToF sensor handles close range.

## 8. How this fits your project

The planner in `src/simulation/` decides global paths for two robots. This detector is the **local obstacle layer**: if something unexpected appears, the robot stops or turns even if the plan didn't expect it. Later, the overhead phone camera + ArUco handles localization.
