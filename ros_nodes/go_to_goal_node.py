import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node

DT = 0.05              # time step (s): 20 updates per second
MAX_SPEED = 0.5        # m/s
MAX_TURN_RATE = 2.0    # rad/s
K_SPEED = 1.0
K_TURN = 4.0
GOAL_TOLERANCE = 0.05  # m


def wrap_angle(angle):
    """Keep an angle between -pi and +pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


class GoToGoalNode(Node):
    def __init__(self):
        super().__init__("go_to_goal")  # the node's name in ROS

        # Simulated robot state: start at (0, 0) facing up
        self.x, self.y, self.theta = 0.0, 0.0, math.pi / 2
        self.goal_x, self.goal_y = 3.0, 2.0
        self.arrived = False

        # Publishers (broadcasters) and a subscriber (listener)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pos_pub = self.create_publisher(Point, "/robot/position", 10)
        self.create_subscription(Point, "/goal", self.on_goal, 10)

        # Call self.update every DT seconds
        self.create_timer(DT, self.update)
        self.get_logger().info(f"Driving to goal ({self.goal_x}, {self.goal_y})")

    def on_goal(self, msg):
        """Runs automatically whenever a message arrives on /goal."""
        self.goal_x, self.goal_y = msg.x, msg.y
        self.arrived = False
        self.get_logger().info(f"New goal: ({msg.x:.2f}, {msg.y:.2f})")

    def update(self):
        """Runs 20 times per second: steer, move, publish."""
        dx, dy = self.goal_x - self.x, self.goal_y - self.y
        distance = math.hypot(dx, dy)
        cmd = Twist()  # all zeros = stop

        if distance < GOAL_TOLERANCE:
            if not self.arrived:
                self.get_logger().info(f"Arrived at ({self.x:.2f}, {self.y:.2f})")
                self.arrived = True
        else:
            heading_error = wrap_angle(math.atan2(dy, dx) - self.theta)
            cmd.linear.x = min(K_SPEED * distance, MAX_SPEED)
            cmd.angular.z = max(-MAX_TURN_RATE, min(K_TURN * heading_error, MAX_TURN_RATE))

            # Simulate the robot moving (a real robot's motors would do this)
            self.x += cmd.linear.x * math.cos(self.theta) * DT
            self.y += cmd.linear.x * math.sin(self.theta) * DT
            self.theta = wrap_angle(self.theta + cmd.angular.z * DT)

        self.cmd_pub.publish(cmd)
        self.pos_pub.publish(Point(x=self.x, y=self.y, z=0.0))


def main():
    rclpy.init()
    node = GoToGoalNode()
    try:
        rclpy.spin(node)  # keep running, handling timers and messages
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()