#!/usr/bin/env python3

class Map:
    def __init__(self):
        self.landmarks = []

    def append_landmark(self, landmark):
        landmark.id = len(self.landmarks)
        self.landmarks.append(landmark)

    def draw(self, ax, elements):
        for lm in self.landmarks:
            lm.draw(ax, elements)
