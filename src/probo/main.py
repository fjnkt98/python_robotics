#!/usr/bin/env python3

import numpy as np
import math

import world
import ideal_robot
import agent
import landmark
import map


if __name__ == "__main__":
    world = world.World(time_span=10.0, time_interval=0.1, debug=False)

    strait_agent = agent.Agent(0.2, 0.0)
    circle_agent = agent.Agent(0.2, 10.0/180*math.pi)

    robot1 = ideal_robot.IdealRobot(
            np.array([2, 3, math.pi/6]).T,
            agent=strait_agent)
    robot2 = ideal_robot.IdealRobot(
            np.array([-2, -1, math.pi/5*6]).T,
            agent=circle_agent,
            color="red")
    robot3 = ideal_robot.IdealRobot(
            np.array([0, 0, 0]).T,
            color="blue"
            )

    world.append(robot1)
    world.append(robot2)
    world.append(robot3)

    m = map.Map()
    m.append_landmark(landmark.Landmark(2, -2))
    m.append_landmark(landmark.Landmark(-1, -3))
    m.append_landmark(landmark.Landmark(3, 3))
    world.append(m)

    world.draw()
