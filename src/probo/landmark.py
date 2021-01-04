#!/usr/bin/env python3

import numpy as np


class Landmark:
    def __init__(self, x, y):
        self.position = np.array([x, y]).T
        self.id = None

    def draw(self, ax, elements):
        c = ax.scatter(
                self.position[0],
                self.position[1],
                s=100,
                marker="*",
                label="landmarks",
                color="orange")
        elements.append(c)
        elements.append(
                ax.text(
                    self.position[0],
                    self.position[1],
                    "id:" + str(self.id),
                    fontsize=10)
                )
