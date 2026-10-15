"""Run several layouts and write results/results.csv (baseline vs cooperative).
Run (from repo root):  python src/simulation/run_experiments.py
Add your own layouts to LAYOUTS below.
"""
import csv, os, sys
sys.path.insert(0, os.path.dirname(__file__))
import multi_robot_sim as sim

GAP_WALL = {(4, y) for y in range(8) if y != 3}
LAYOUTS = {
    "gap_wall_crossing": dict(
        obstacles=GAP_WALL,
        robots={"R1": {"start": (0, 2), "goal": (7, 4), "park": (0, 5)},
                "R2": {"start": (0, 4), "goal": (7, 2), "park": (0, 3)}}),
    "open_arena_crossing": dict(
        obstacles=set(),
        robots={"R1": {"start": (0, 0), "goal": (7, 7), "park": (0, 1)},
                "R2": {"start": (0, 7), "goal": (7, 0), "park": (0, 6)}}),
    "two_walls": dict(
        obstacles={(3, y) for y in range(8) if y != 2} | {(5, y) for y in range(8) if y != 5},
        robots={"R1": {"start": (0, 1), "goal": (7, 6), "park": (0, 2)},
                "R2": {"start": (0, 6), "goal": (7, 1), "park": (0, 5)}}),
}

rows = []
for name, cfg in LAYOUTS.items():
    sim.OBSTACLES, sim.ROBOTS = cfg["obstacles"], cfg["robots"]
    for mode, planner in (("baseline", sim.baseline_plan), ("cooperative", sim.prioritized_plan)):
        try:
            m = sim.metrics(planner())
        except RuntimeError:
            m = {"collisions": "no solution", "makespan": "-", "total_steps": "-"}
        rows.append({"layout": name, "planner": mode, **m})
        print(rows[-1])

out = os.path.join(os.path.dirname(__file__), "..", "..", "results")
os.makedirs(out, exist_ok=True)
with open(os.path.join(out, "results.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["layout", "planner", "collisions", "makespan", "total_steps"])
    w.writeheader(); w.writerows(rows)
print("Wrote results/results.csv")
