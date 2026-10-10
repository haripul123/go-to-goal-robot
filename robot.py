import math

import matplotlib.pyplot as plt

DT = 0.05              # time step (s)
MAX_SPEED = 1.0        # m/s
MAX_TURN_RATE = 2.0    # rad/s
K_SPEED = 1.0          # how strongly the robot speeds up when far away
K_TURN = 4.0           # how strongly the robot turns towards the goal
GOAL_TOLERANCE = 0.05  # m: close enough counts as arrived


def wrap_angle(angle):
    """Keep an angle between -pi and +pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def step(x, y, theta, v, omega, dt=DT):
    """Move the robot forward by one time step."""
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta = wrap_angle(theta + omega * dt)
    return x, y, theta


def go_to_goal(x, y, theta, goal_x, goal_y, k_turn=K_TURN, max_steps=2000):
    """Drive the robot to the goal. Returns the list of (x, y) positions."""
    path = [(x, y)]
    for _ in range(max_steps):
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)
        if distance < GOAL_TOLERANCE:
            break

        heading_error = wrap_angle(math.atan2(dy, dx) - theta)
        v = min(K_SPEED * distance, MAX_SPEED)
        omega = max(-MAX_TURN_RATE, min(k_turn * heading_error, MAX_TURN_RATE))

        x, y, theta = step(x, y, theta, v, omega)
        path.append((x, y))
    return path


if __name__ == "__main__":
    start = (0.0, 0.0, math.pi / 2)   # x, y, facing up
    goal = (3.0, 2.0)

    path = go_to_goal(*start, *goal)
    xs, ys = zip(*path)

    plt.figure(figsize=(6, 5))
    plt.plot(xs, ys, label="Robot path")
    plt.plot(start[0], start[1], "go", markersize=10, label="Start")
    plt.plot(goal[0], goal[1], "r*", markersize=15, label="Goal")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title(f"Reached goal in {len(path) * DT:.1f} s")
    plt.show()