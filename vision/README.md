# Vision (planned)

Overhead phone camera -> laptop -> OpenCV ArUco detection -> robot pose (x, y, heading).

Planned steps:
1. Calibrate the camera (checkerboard).
2. Print ArUco markers (10 cm+, matte paper), one per robot.
3. Detect markers, map pixels to arena grid coordinates.
4. Feed poses to the planner.
