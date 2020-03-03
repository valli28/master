import numpy as np

class Ray():
    def __init__(self, pos, direction):
        self.pos = np.array([pos[0], pos[1]])
        self.dir = np.linalg.norm(np.array(direction - pos)) # Direction unit vector


    def cast(self, wall):
        print("I'm trying to cast")
