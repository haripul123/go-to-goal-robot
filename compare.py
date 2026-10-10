import math

import matplotlib.pyplot as plt

from robot import DT, GOAL_TOLERANCE, go_to_goal

start = (0.0, 0.0, math.pi / 2)  # x, y, facing up
goal = (3.0, 2.0)

plt.figure(figsize=(7, 6))

for k_turn in [0.3, 0.4, 0.5]:
    path = go_to_goal(*(0.0,0.0,math.pi), *goal, k_turn=k_turn, max_steps=600)
    xs, ys = zip(*path)
    end_distance = math.dist(path[-1], goal)
    if end_distance < GOAL_TOLERANCE:
        result = f"arrived in {len(path) * DT:.1f} s"
    else:
        result = "never arrived!"
    plt.plot(xs, ys, label=f"K_TURN = {k_turn}: {result}")

plt.plot(start[0], start[1], "go", markersize=10, label="Start")
plt.plot(goal[0], goal[1], "r*", markersize=15, label="Goal")
plt.axis("equal")
plt.grid(True)
plt.legend(fontsize=8)
plt.title("Effect of the turning gain K_TURN")
plt.show()