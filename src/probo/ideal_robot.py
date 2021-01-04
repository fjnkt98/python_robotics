#!/usr/bin/env python3

# import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math


class IdealRobot():
    """IdealRobot class
        The robot without noise

        Attributes:
            pose (numpy.ndarray): pose of robot
            r (double): robot radius (constant value)
            color (string): robot color
            agent (Agent): Agent class instance for robot
            poses (List): Trajectory list of the robot movement
    """

    def __init__(self, pose, agent=None, color="black"):
        self.pose = pose
        self.r = 0.2
        self.color = color
        self.agent = agent
        self.poses = [pose]

    @classmethod
    def state_transition(cls, nu, omega, time, pose):
        if math.fabs(omega) < 1e-10:
            return pose + np.array([
                nu * math.cos(pose[2]),
                nu * math.sin(pose[2]),
                omega]) * time
        else:
            return pose + np.array([
                nu/omega*(math.sin(pose[2] + omega*time) - math.sin(pose[2])),
                nu/omega*(-math.cos(pose[2] + omega*time) + math.cos(pose[2])),
                omega * time
                ])

    def draw(self, ax, elements):
        """draw robot

            Args:
                ax (matplotlib.axes._subplots.AxesSubplot): axis object
        """
        x, y, theta = self.pose

        x_nose = x + self.r * math.cos(theta)
        y_nose = y + self.r * math.sin(theta)

        elements += ax.plot([x, x_nose], [y, y_nose], color=self.color)

        c = patches.Circle(
                xy=(x, y),
                radius=self.r,
                fill=False,
                color=self.color)

        elements.append(ax.add_patch(c))

        self.poses.append(self.pose)
        elements += ax.plot(
                [e[0] for e in self.poses],
                [e[1] for e in self.poses],
                linewidth=0.5,
                color="black"
                )

    def one_step(self, time_interval):
        if not self.agent:
            return

        nu, omega = self.agent.decision()
        self.pose = self.state_transition(nu, omega, time_interval, self.pose)
