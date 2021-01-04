#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.animation as anm


class World():
    """The world in which robots operate
        Attributes:
            objects (list): The objects belonging to the world.
            debug (bool): Control whether to debug or not.
    """

    def __init__(self, time_span, time_interval, debug=False):
        self.objects = []
        self.debug = debug
        self.time_span = time_span
        self.time_interval = time_interval

    def append(self, obj):
        self.objects.append(obj)

    def draw(self):
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111)

        ax.set_aspect("equal")
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_xlabel("X", fontsize=20)
        ax.set_ylabel("Y", fontsize=20)

        elements = []

        if self.debug:
            for i in range(1000):
                self.one_step(i, elements, ax)
        else:
            self.animation = anm.FuncAnimation(
                    fig,
                    self.one_step,
                    fargs=(elements, ax),
                    frames=int(self.time_span/self.time_interval) + 1,
                    interval=int(self.time_interval*1000),
                    repeat=False)
        plt.show()

    def one_step(self, i, elements, ax):
        while elements:
            elements.pop().remove()

        time_string = "t = %.2f[s]" % (self.time_interval*i)
        elements.append(ax.text(-4.4, 4.5, time_string, fontsize=10))

        for obj in self.objects:
            obj.draw(ax, elements)

            if hasattr(obj, "one_step"):
                obj.one_step(self.time_interval)


if __name__ == "__main__":
    world = World(debug=False)

    world.draw()
