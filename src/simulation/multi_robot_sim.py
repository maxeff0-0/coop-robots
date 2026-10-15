"""
Two-robot cooperative path planning on a grid.
Mission: both robots start together, each visits its own goal, then parks.
Compares: (A) baseline independent A*  vs  (B) prioritized space-time A*.

Run (from repo root):  python src/simulation/multi_robot_sim.py
"""
import heapq
import os
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------------- Arena config (edit these) ----------------
W, H = 10, 10

# Vertical wall at x=5 with a single gap at y=5, plus 4 blocks on each side
OBSTACLES = (
    {(5, y) for y in range(H) if y != 5}
    | {(2, 2), (2, 3), (2, 7), (2, 8)}
    | {(7, 2), (7, 3), (7, 7), (7, 8)}
)

ROBOTS = {
    "R1": {"start": (0, 4), "goal": (9, 6), "park": (0, 6)},
    "R2": {"start": (0, 6), "goal": (9, 4), "park": (0, 4)},
}
MAX_T = 80
MOVES = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]  # wait + 4 directions


def h(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar_st(start, goal, t0, reserved_v, reserved_e, final_cells):
    """Space-time A*. Returns list of cells starting at time t0 (index 0 = t0)."""
    open_ = [(h(start, goal), 0, start, t0)]
    came = {(start, t0): None}
    g = {(start, t0): 0}
    while open_:
        _, cost, pos, t = heapq.heappop(open_)
        if pos == goal and not any(
            ft <= t + 1 and c == goal for c, ft in final_cells
        ):
            # make sure we aren't sitting in a cell another robot parks in later
            if all(not (c == pos and ft > t) for c, ft in final_cells):
                path, node = [], (pos, t)
                while node:
                    path.append(node[0])
                    node = came[node]
                return path[::-1]
        if t - t0 > MAX_T:
            continue
        for dx, dy in MOVES:
            nxt = (pos[0] + dx, pos[1] + dy)
            nt = t + 1
            if not (0 <= nxt[0] < W and 0 <= nxt[1] < H) or nxt in OBSTACLES:
                continue
            if (nxt, nt) in reserved_v:
                continue
            if (nxt, pos, nt) in reserved_e:  # swap conflict
                continue
            if any(c == nxt and nt >= ft for c, ft in final_cells):
                continue  # another robot already parked there
            key = (nxt, nt)
            ncost = cost + 1
            if key not in g or ncost < g[key]:
                g[key] = ncost
                came[key] = (pos, t)
                heapq.heappush(open_, (ncost + h(nxt, goal), ncost, nxt, nt))
    return None


def plan_mission(r, reserved_v, reserved_e, final_cells):
    """start -> goal -> park, as one continuous timeline."""
    p1 = astar_st(r["start"], r["goal"], 0, reserved_v, reserved_e, final_cells)
    if p1 is None:
        return None
    t = len(p1) - 1
    p2 = astar_st(r["goal"], r["park"], t, reserved_v, reserved_e, final_cells)
    if p2 is None:
        return None
    return p1 + p2[1:]


def prioritized_plan():
    paths, rv, re, finals = {}, set(), set(), []
    for name, r in ROBOTS.items():  # priority = dict order
        path = plan_mission(r, rv, re, finals)
        if path is None:
            raise RuntimeError(f"No path for {name}")
        paths[name] = path
        for t, c in enumerate(path):
            rv.add((c, t))
            if t > 0:
                re.add((path[t - 1], c, t))  # reverse edge of what others may do
        finals.append((path[-1], len(path) - 1))
    return paths


def baseline_plan():
    """Each robot plans alone (no knowledge of the other)."""
    return {n: plan_mission(r, set(), set(), []) for n, r in ROBOTS.items()}


def count_collisions(paths):
    """Vertex + swap conflicts. Finished robots stay parked at last cell."""
    names = list(paths)
    T = max(len(p) for p in paths.values())
    at = lambda p, t: p[min(t, len(p) - 1)]
    n = 0
    for t in range(T):
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = paths[names[i]], paths[names[j]]
                if at(a, t) == at(b, t):
                    n += 1
                elif t > 0 and at(a, t) == at(b, t - 1) and at(b, t) == at(a, t - 1):
                    n += 1
    return n


def metrics(paths):
    return {
        "collisions": count_collisions(paths),
        "makespan": max(len(p) - 1 for p in paths.values()),
        "total_steps": sum(len(p) - 1 for p in paths.values()),
    }


def animate(paths, title, save=None):
    fig, ax = plt.subplots(figsize=(5, 5))
    T = max(len(p) for p in paths.values())
    colors = {"R1": "tab:blue", "R2": "tab:red"}

    def draw(t):
        ax.clear()
        ax.set_xlim(-0.5, W - 0.5); ax.set_ylim(-0.5, H - 0.5)
        ax.set_xticks(range(W)); ax.set_yticks(range(H)); ax.grid(True)
        ax.set_aspect("equal")
        for o in OBSTACLES:
            ax.add_patch(plt.Rectangle((o[0] - .5, o[1] - .5), 1, 1, color="k"))
        for n, r in ROBOTS.items():
            ax.plot(*r["goal"], marker="*", ms=16, color=colors[n], alpha=.4)
            ax.plot(*r["park"], marker="s", ms=14, color=colors[n], alpha=.3)
            p = paths[n]
            xs, ys = zip(*p[: min(t, len(p) - 1) + 1])
            ax.plot(xs, ys, "-", color=colors[n], alpha=.5)
            c = p[min(t, len(p) - 1)]
            ax.plot(*c, "o", ms=18, color=colors[n])
            ax.text(*c, n, color="w", ha="center", va="center", fontsize=7)
        ax.set_title(f"{title}   t={t}")

    ani = FuncAnimation(fig, draw, frames=T + 3, interval=400, repeat=False)
    if save:
        ani.save(save, writer="pillow", fps=3)
    return ani


if __name__ == "__main__":
    base, coop = baseline_plan(), prioritized_plan()
    print("Baseline (independent A*):  ", metrics(base))
    print("Cooperative (space-time A*):", metrics(coop))
    for n in coop:
        print(n, "path:", coop[n])
    out = os.path.join(os.path.dirname(__file__), "..", "..", "results")
    os.makedirs(out, exist_ok=True)
    animate(base, "BASELINE", save=os.path.join(out, "baseline.gif"))
    animate(coop, "COOPERATIVE", save=os.path.join(out, "cooperative.gif"))
    print("Saved results/baseline.gif and results/cooperative.gif")
