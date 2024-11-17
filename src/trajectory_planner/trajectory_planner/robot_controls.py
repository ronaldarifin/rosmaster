import numpy as np

from trajectory import TrajectoryPlanner

import resource.SunriseRobotLib.SunriseRobotLib as srl

class RobotController:
    def __init__(self, robot: srl.SunriseRobot):
        self.robot = robot
        self.position = np.array([0.0, 0.0, 0.0])

    def move_robot(self, trajectory: TrajectoryPlanner):
        pass
        
