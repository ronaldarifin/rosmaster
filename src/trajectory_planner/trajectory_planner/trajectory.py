from slam import SLAM

class RobotState:
    def __init__(self):
        self.position = {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 0,
        }

        self.velocity = {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 0,
        }

        self.acceleration = {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 0,
        }

    def set_position(self, x:float = None, y:float = None, z:float = None, w:float = None):
        if x is not None:
            self.position["x"] = x
        if y is not None:
            self.position["y"] = y
        if z is not None:
            self.position["z"] = z
        if w is not None:
            self.position["w"] = w    
        
    def set_velocity(self, x:float = None, y:float = None, z:float = None, w:float = None):
        if x is not None:
            self.velocity["x"] = x
        if y is not None:
            self.velocity["y"] = y
        if z is not None:
            self.velocity["z"] = z
        if w is not None:
            self.velocity["w"] = w     

    def set_acceleration(self, x:float = None, y:float = None, z:float = None, w:float = None):
        if x is not None:
            self.acceleration["x"] = x
        if y is not None:
            self.acceleration["y"] = y
        if z is not None:
            self.acceleration["z"] = z
        if w is not None:
            self.acceleration["w"] = w 


class TrajectoryPlanner:
    """
    Base class for trajectory planning. Defines the interface and common functionality
    for trajectory planners.
    """
    def __init__(self, map: SLAM):
        self.trajectory: list[RobotState] = []
        self.map = map

    def generate_trajectory(self, start:RobotState, goal:RobotState):
        """
        Plan a trajectory from start to goal.

        This method should be implemented by subclasses.
        """
        raise NotImplementedError("The plan method must be implemented by subclasses.")

    def get_trajectory(self):
        """
        Get the planned trajectory.
        """
        return self.trajectory

class TrivialTrajectory(TrajectoryPlanner):
    """
    A simple implementation of a trajectory planner that moves directly from start to goal.
    """
    def __init__(self):
        pass

    def generate_trajectory(self, start, goal):
        """
        Plan a trajectory that goes directly from start to goal.
        """
        self.trajectory = [start, goal]

class AStarTrajectory(TrajectoryPlanner):
    """
    A trajectory planner implementation using the A* search algorithm.
    """
    def __init__(self):
        pass

    def generate_trajectory(self, start, goal):
        """
        Plan a trajectory using the A* search algorithm.
        """
        self.trajectory = self._a_star_search(start, goal, self.map)

# Example usage
if __name__ == "__main__":
    start = [0, 0]
    goal = [5, 5]

    # Using TrivialTrajectory
    trivial_planner = TrivialTrajectory()
    trivial_planner.plan(start, goal)
    print("Trivial Trajectory:", trivial_planner.get_trajectory())

    # Using AStarTrajectory
    grid_map = [[0] * 10 for _ in range(10)]  # Example grid map
    a_star_planner = AStarTrajectory(grid_map)
    a_star_planner.plan(start, goal)
    print("A* Trajectory:", a_star_planner.get_trajectory())
