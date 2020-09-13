import time
import numpy as np

from typing import List

from src.robot import Robot


class Engine:
    XLIM = (-0.75, 0.75)
    YLIM = (-0.65, 0.65)

    def __init__(self, robots: List[Robot]):
        self.timeref = time.time()
        self.robots = robots

    def set_time(self):
        self.timeref = time.time()

    def get_delta(self):
        new_time = time.time()
        delta = new_time - self.timeref
        self.timeref = new_time
        return delta

    def update(self, callback):
        delta = self.get_delta()
        for robot in self.robots:
            left, right = callback(robot.id)
            robot.set_state(left, right, delta)
            # Keep robot in bound
            robot.x = min(max(robot.x, Engine.XLIM[0]), Engine.XLIM[1])
            robot.y = min(max(robot.y, Engine.YLIM[0]), Engine.YLIM[1])
