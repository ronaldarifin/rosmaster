import numpy as np
import heapq

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class Trajectory(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Subscriber to the /map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Subscriber to the /goal_pose topic
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publisher for the /trajectory topic
        self.trajectory_publisher = self.create_publisher(
            Path,
            '/trajectory',
            10
        )

        # Internal storage for map and goal
        self.current_map = None
        self.goal_pose = None

        self.get_logger().info("TrajectoryPlanner node initialized.")

    def map_callback(self, map_msg: OccupancyGrid):
        """
        Callback function for the /map topic. Stores the received map
        and triggers trajectory planning if a goal is available.

        :param map_msg: The OccupancyGrid message from the /map topic.
        """
        self.get_logger().info("Received a map message.")
        self.current_map = map_msg

        if self.goal_pose:
            self.plan_and_publish_trajectory()

    def goal_callback(self, goal_msg: PoseStamped):
        """
        Callback function for the /goal_pose topic. Stores the received goal
        and triggers trajectory planning if a map is available.

        :param goal_msg: The PoseStamped message from the /goal_pose topic.
        """
        self.get_logger().info("Received a goal pose.")
        self.goal_pose = goal_msg

        if self.current_map:
            self.plan_and_publish_trajectory()

    def plan_and_publish_trajectory(self):
        """
        Plans a trajectory based on the current map and goal, and publishes it.
        """
        if not self.current_map or not self.goal_pose:
            self.get_logger().warn("Map or goal pose is missing. Cannot plan trajectory.")
            return

        # Generate the trajectory
        trajectory = self.generate_trajectory(self.current_map, self.goal_pose)

        # Publish the trajectory
        self.trajectory_publisher.publish(trajectory)
        self.get_logger().info("Published trajectory.")

    def generate_trajectory(self, map_msg: OccupancyGrid, goal_pose: PoseStamped) -> Path:
        """
        Generates a trajectory using the A* algorithm based on the input map and goal.
        :param map_msg: The OccupancyGrid message.
        :param goal_pose: The PoseStamped message representing the goal.
        :return: A nav_msgs/Path message.
        """
        # Parse OccupancyGrid into a 2D numpy array
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position

        grid = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
        grid = np.where(grid > 0, 1, 0)  # Convert to binary occupancy grid

        # Transform start and goal into grid coordinates
        start_x = int(-origin.x / resolution)
        start_y = int(-origin.y / resolution)
        goal_x = int((goal_pose.pose.position.x - origin.x) / resolution)
        goal_y = int((goal_pose.pose.position.y - origin.y) / resolution)

        # Plan the path
        path_points = self._astar((start_y, start_x), (goal_y, goal_x), grid)

        if not path_points:
            self.get_logger().warn("A* algorithm could not find a path.")
            return Path()

        # Convert path to ROS Path message
        path = Path()
        path.header = map_msg.header

        for (y, x) in path_points:
            pose = PoseStamped()
            pose.header = map_msg.header
            pose.pose.position.x = x * resolution + origin.x
            pose.pose.position.y = y * resolution + origin.y
            path.poses.append(pose)

        return path

    def _astar(start, goal, grid):
        """
        A* pathfinding algorithm implementation.
        :param start: (x, y) tuple for the start position.
        :param goal: (x, y) tuple for the goal position.
        :param grid: 2D numpy array representing the occupancy grid (0: free, 1: obstacle).
        :return: List of (x, y) tuples representing the path from start to goal.
        """
        def heuristic(a, b):
            # Euclidean distance as heuristic
            return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if (0 <= neighbor[0] < grid.shape[0] and
                        0 <= neighbor[1] < grid.shape[1] and
                        grid[neighbor[0], neighbor[1]] == 0):
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found



def main(args=None):
    rclpy.init(args=args)
    node = Trajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TrajectoryPlanner node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
