import time
from src.Boid import Boid

class Loop:
    def __init__(self, boids, delay=0.1):
        self.boids = boids
        self.delay = delay

    def run(self):
        while True:
            grid = {}
            for boid in self.boids:
                boid_grid_pos = boid.get_grid_pos()
                grid.setdefault(boid_grid_pos, []).append(boid)

            for boid in self.boids:
                boid.update(self.boids, grid)
            time.sleep(self.delay)