from random import random
from src.main import GRID_X1, GRID_Y1, GRID_X2, GRID_Y2

class Boid:
    def __init__(self, position, velocity, max_velocity, range_of_view, size, strength, random_factor, boids):
        self.position = position
        self.velocity = velocity
        self.max_velocity = max_velocity
        self.range_of_view = range_of_view
        self.size = size
        self.strength = strength
        self.random_factor = random_factor
        boids.append(self)

    def get_grid_pos(self):
        gridx = self.position['x'] // self.range_of_view
        gridy = self.position['y'] // self.range_of_view
        return (gridx, gridy)        

    def detect_neighbors(self, boids, grid):
        neighbors = []
        boid_grid_pos = self.get_grid_pos()
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor_grid_pos = (boid_grid_pos[0] + dx, boid_grid_pos[1] + dy)
                if neighbor_grid_pos in grid:
                    for boid in grid[neighbor_grid_pos]:
                        if boid is not self:
                            distance = ((self.position['x'] - boid.position['x']) ** 2 + 
                                        (self.position['y'] - boid.position['y']) ** 2) ** 0.5
                            if distance <= self.range_of_view:
                                neighbors.append(boid)
        
        return neighbors
    
    def adapt_velocity(self, neighbors):
        if not neighbors:
            return
        
        avg_velocity = {'x': 0, 'y': 0}
        for neighbor in neighbors:
            avg_velocity['x'] += neighbor.velocity['x']
            avg_velocity['y'] += neighbor.velocity['y']
        
        avg_velocity['x'] /= len(neighbors)
        avg_velocity['y'] /= len(neighbors)
        
        self.velocity['x'] += (avg_velocity['x'] - self.velocity['x']) * self.strength
        self.velocity['y'] += (avg_velocity['y'] - self.velocity['y']) * self.strength

        if (self.velocity['x'] ** 2 + self.velocity['y'] ** 2) ** 0.5 > self.max_velocity:
            scale = self.max_velocity / ((self.velocity['x'] ** 2 + self.velocity['y'] ** 2) ** 0.5)
            self.velocity['x'] *= scale
            self.velocity['y'] *= scale
    
    def avoid_collisions(self, neighbors):
        for neighbor in neighbors:
            distance = ((self.position['x'] - neighbor.position['x']) ** 2 + 
                        (self.position['y'] - neighbor.position['y']) ** 2) ** 0.5
            if distance < self.size + neighbor.size:
                self.velocity['x'] += (self.position['x'] - neighbor.position['x']) * self.strength
                self.velocity['y'] += (self.position['y'] - neighbor.position['y']) * self.strength

        if (self.velocity['x'] ** 2 + self.velocity['y'] ** 2) ** 0.5 > self.max_velocity:
            scale = self.max_velocity / ((self.velocity['x'] ** 2 + self.velocity['y'] ** 2) ** 0.5)
            self.velocity['x'] *= scale
            self.velocity['y'] *= scale

    def move(self):
        self.position['x'] += self.velocity['x']
        self.position['y'] += self.velocity['y']

        if self.position['x'] > GRID_X1:
            self.position['x'] = GRID_X2
        elif self.position['x'] < GRID_X2:
            self.position['x'] = GRID_X1

        if self.position['y'] > GRID_Y1:
            self.position['y'] = GRID_Y2
        elif self.position['y'] < GRID_Y2:
            self.position['y'] = GRID_Y1
    
    def apply_randomness(self):
        self.velocity['x'] += (random() - 0.5) * self.random_factor
        self.velocity['y'] += (random() - 0.5) * self.random_factor

    def update(self, boids, grid):
        neighbors = self.detect_neighbors(boids, grid)
        self.adapt_velocity(neighbors)
        self.avoid_collisions(neighbors)
        self.apply_randomness()
        self.move()