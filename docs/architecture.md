# System Architecture

```
Phone camera (overhead)
        |  video over WiFi (DroidCam / IP Webcam)
        v
Laptop: OpenCV + ArUco  -> robot poses
        |
        v
Planner (prioritized space-time A*)  -> waypoints
        |  WiFi (MQTT / UDP)
        v
ESP32 robot 1 / robot 2
   TB6612 -> 2 motors, LM393 encoders, MPU6050 IMU, VL53L0X ToF (emergency stop)
```

Replace this with a proper diagram image (draw.io) and save it as `docs/block_diagram.png`.
