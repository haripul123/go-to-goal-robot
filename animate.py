import math
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from robot import DT, go_to_goal

START = (0.0, 0.0, math.pi)   # x, y, facing left (away from the goal)
GOAL = (3.0, 2.0)
ROBOT_RADIUS = 0.15

path = go_to_goal(*START, *GOAL)
xs, ys = zip(*path)

# Work out which way the robot faces at each frame from how it moved
headings = [START[2]]
for (x0, y0), (x1, y1) in zip(path, path[1:]):
    headings.append(math.atan2(y1 - y0, x1 - x0))

fig, ax = plt.subplots(figsize=(6, 5))
ax.set_xlim(-1, 4)
ax.set_ylim(-1, 3)
ax.set_aspect("equal")
ax.grid(True)
ax.plot(*GOAL, "r*", markersize=15, label="Goal")
ax.legend(loc="upper left")

(trail,) = ax.plot([], [], "b-", linewidth=1)
body = plt.Circle((0, 0), ROBOT_RADIUS, color="tab:blue")
ax.add_patch(body)
(nose,) = ax.plot([], [], "k-", linewidth=2)   # line showing which way it faces


def update(i):
    x, y, heading = xs[i], ys[i], headings[i]
    trail.set_data(xs[: i + 1], ys[: i + 1])
    body.center = (x, y)
    nose.set_data(
        [x, x + ROBOT_RADIUS * 1.5 * math.cos(heading)],
        [y, y + ROBOT_RADIUS * 1.5 * math.sin(heading)],
    )
    ax.set_title(f"t = {i * DT:.1f} s")
    return trail, body, nose


anim = FuncAnimation(fig, update, frames=len(path), interval=DT * 1000)

if "--gif" in sys.argv:
    anim.save("demo.gif", writer=PillowWriter(fps=20))
    print("Saved demo.gif")
else:
    plt.show()