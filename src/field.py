import time
import numpy as np
import matplotlib.pyplot as plt

from typing import List

from src.robot import Robot


class Field:
    XLIM = (-0.75, 0.75)
    YLIM = (-0.65, 0.65)

    def __init__(self, robots: List[Robot]):
        self.fig = plt.figure()

        self.ax = plt.axes(xlim=Field.XLIM, ylim=Field.YLIM)
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        self.robot_plot = {}
        self.robots = {}
        for i in range(len(robots)):
            self.robots[i] = robots[i]
            self.robot_plot[i], = self.ax.plot([], [], 'bh' if robots[i].team == 'blue' else 'yH')

    def plot_robots(self):
        for i in range(len(self.robots)):
            self.robot_plot[i].set_data(*self.robots[i].get_pos())
