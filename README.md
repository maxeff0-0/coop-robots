# Cooperative Two-Robot Path Planning

Two low-cost ESP32 robots start together, each visits its own goal, and both return to a
parking spot without colliding. A laptop plans paths and tracks robots with an overhead camera.

**Status:** simulation working. Hardware, camera tracking and real-world evaluation in progress.

## Approach
- Grid model of the arena
- Baseline: independent A* per robot (no coordination)
- Proposed: prioritized planning with space-time A* (waits and reservations avoid collisions)
- Metrics: collisions, makespan, total steps

## Quick start
```bash
pip install -r requirements.txt
python src/simulation/multi_robot_sim.py       # GIFs in results/
python src/simulation/run_experiments.py       # results/results.csv
```

## Results
See `results/` (GIFs and `results.csv`). Add a results table here once you have final numbers.

## Hardware
ESP32, TB6612FNG, N20/TT motors, LM393 speed sensors, MPU6050, VL53L0X, HC-SR04,
2x 18650 + LM2596 buck converter, phone as overhead camera.

## Repo layout
```
src/simulation/   planner and experiments
firmware/         ESP32 code
vision/           ArUco tracking (planned)
docs/             architecture, progress log
results/          GIFs and CSV
```

## Roadmap
- [x] Grid simulator with baseline vs cooperative planner
- [ ] One robot driving reliably (encoders + IMU)
- [ ] ArUco localization from phone camera
- [ ] Waypoint following over WiFi
- [ ] Two-robot mission on a taped grid
- [ ] Evaluation, report, demo video

## License
MIT
