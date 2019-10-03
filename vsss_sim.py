'''
    Script for IEEE-VSSS control simulation
    Control model is PID based

'''

from scipy import integrate
import numpy as np
import matplotlib.pyplot as plt


def rought_signal(x):
    if x >= 0:
        return 1
    else:
        return -1

class robotController:

    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.left_error = 0.0
        self.right_error = 0.0
        self.left_previous_error = 0.0
        self.right_previous_error = 0.0
        self.left_error_integrative = 0.0
        self.right_error_integrative = 0.0
        self.left_error_derivative = 0.0
        self.right_error_derivative = 0.0
        self.time = [0.0]

    def setError(self, x, y, ux, uy, xref, yref):
        self.left_error = rought_signal(ux*(xref-x) + uy*(yref-y)) * (np.sqrt((xref-x)**2 + (yref-y)**2) - (ux*(yref-y) - uy*(xref-x)))
        self.right_error = rought_signal(ux*(xref-x) + uy*(yref-y)) * (np.sqrt((xref-x)**2 + (yref-y)**2) + (ux*(yref-y) - uy*(xref-x)))

    def pushTime(self, t):
        ''' Called after set both errors '''
        if self.time[-1] != t:
            self.time.append(t)
        if len(self.time) >= 2:
            self.left_error_integrative += self.left_error * (self.time[-1] - self.time[-2])
            self.right_error_integrative += self.right_error * (self.time[-1] - self.time[-2])
            self.left_error_derivative = (
                self.left_error - self.left_previous_error)/(self.time[-1] - self.time[-2])
            self.left_previous_error = self.left_error
            self.right_error_derivative = (
                self.right_error - self.right_previous_error)/(self.time[-1] - self.time[-2])
            self.right_previous_error = self.right_error
            self.time.pop(0)

    # This is the controller itself
    def leftCorrection(self):
        return (self.kp * self.left_error + self.ki * self.left_error_integrative + self.kd * self.left_error_derivative)

    def rightCorrection(self):
        return (self.kp * self.right_error + self.ki * self.right_error_integrative + self.kd * self.right_error_derivative)



def increment(v, t, controller, setpoint):
    ''' Returns dv/dt = f(v,t) '''
    xref, yref = setpoint
    [x, y, ux, uy] = v
    robot_width = 7.5
    controller.setError(x, y, ux, uy, xref, yref)
    controller.pushTime(t)
    lmotor = controller.leftCorrection()
    rmotor = controller.rightCorrection()
    xdot = ux*(lmotor + rmotor)/2.0
    ydot = uy*(lmotor + rmotor)/2.0
    uxdot = uy*(lmotor - rmotor)/robot_width
    uydot = -ux*(lmotor - rmotor)/robot_width
    return [xdot, ydot, uxdot, uydot]


###     Simulation parameters   ###
Ts = 0.05
Tf = 50.0
t = np.linspace(0, Tf, 1+int(Tf/Ts))
v0 = [10, 10, np.cos(60*np.pi/180), np.sin(60*np.pi/180)]
ref = (100, 100)
robot = robotController(kp=0.2)
v = integrate.odeint(increment, v0, t, args=(robot, (100, 100), ))

###     Simulation results      ###
ax_pos = plt.subplot(321)
ax_pos.plot(t, v[:, 0], 'b', t, v[:, 1], 'r')
ax_pos.set_ylabel('Position')
ax_pos.grid(True)

ax_u = plt.subplot(323)
ax_u.plot(t, v[:, 2], 'b', t, v[:, 3], 'r')
ax_u.set_ylabel('Orientation Vector')
ax_u.grid(True)

ax_theta = plt.subplot(325)
ax_theta.plot(t, (180/np.pi)*np.arctan2(v[:,3],v[:,2]))
ax_theta.set_ylabel('Orientation (°)')
ax_theta.set_xlabel('Time (s)')
ax_theta.grid(True)

ax_field = plt.subplot(122)
step = int(1/Ts)
ax_field.quiver(v[::step,0], v[::step,1], v[::step,2], v[::step,3] )
ax_field.set_xlim(0, 150)
ax_field.set_ylim(0, 130)
ax_field.set_title('Field Trajectory (cm)')
ax_field.grid(True)

plt.show()