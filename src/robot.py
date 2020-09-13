import numpy as np


def limit(r: float, lim=1.0):
    if r > lim:
        return lim
    elif r < -lim:
        return -lim
    return r


class Robot:
    """Class of robot state
        - x, y: robot position in meters
        - vx, vy: robot speed in meters per second
        - angle: robot angle with x-axis in rad (-pi, pi)
        - vangle: robot angle velocity in rad per second
    """

    def __init__(self, id=0, team='blue', x=0.0, y=0.0, angle=0.0, vx=0.0, vy=0.0, vangle=0.0):
        self.id = id
        self.team = team
        self.x = x
        self.y = y
        self.angle = angle
        self.vx = vx
        self.vy = vy
        self.vangle = vangle
        self.mass = 0.5
        self.width = 0.075

    def set_pos(self, x: float, y: float):
        self.x = x
        self.y = y

    def set_speed(self, vx: float, vy: float):
        self.vx = vx
        self.vy = vy

    def get_pos(self) -> np.array:
        return np.array((self.x, self.y), dtype=float)

    def get_speed(self) -> np.array:
        return np.array((self.vx, self.vy), dtype=float)

    def get_velocity(self) -> float:
        return ((self.vx*self.vx)+(self.vy*self.vy))**0.5

    def set_state(self, left: float, right: float, delta: float):
        """Set a robot state in a engine callback
            - left: left motor rotation speed in m/s
            - right: right motor rotation speed in m/s
            Speed is limited to [-1.0, 1.0]
        """
        # Set new velocities based on motor speed
        left = limit(left)
        right = limit(right)
        vx = (left + right)/2.0 * np.cos(self.angle)
        vy = (left + right)/2.0 * np.sin(self.angle)
        self.set_speed(vx, vy)
        self.vangle = (-left + right)/self.width
        # Set new position and angle based on velocities and delta time
        x, y = self.get_pos() + delta * self.get_speed()
        angle = self.angle + delta * self.vangle
        self.set_pos(x, y)
        # limit to -pi, pi
        self.angle = np.arctan2(np.sin(angle), np.cos(angle))
