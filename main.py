import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from src.robot import Robot
from src.engine import Engine
from src.field import Field

robot1 = Robot()
eng = Engine([robot1])
field = Field([robot1])


def foo(n):
    return 0.5, 0.35


def update(n):
    eng.update(foo)
    field.plot_robots()


if __name__ == '__main__':
    eng.set_time()
    animation = FuncAnimation(field.fig, update, interval=16)
    plt.show()
