import numpy as np
import time

from trajectory import TrajectoryPlanner, AStarTrajectory, TrivialTrajectory

import resource.SunriseRobotLib.SunriseRobotLib as srl

class Motion:
    def __init__(self, vel_x, vel_y, vel_w, time):
        self.velocity = np.array([vel_x, vel_y, vel_w])
        self.time = time

class RobotController:
    def __init__(self, robot: srl.SunriseRobot):
        self.robot = robot
        self.position = np.array([0.0, 0.0, 0.0])

    def move_robot(self, trajectory: TrajectoryPlanner):
        motion: Motion = self.get_velocity(trajectory)
        self.robot.set_car_motion(*motion.velocity)
        time.sleep(motion.time)

    def get_velocity(self, trajectory: TrajectoryPlanner):
        pass
        
