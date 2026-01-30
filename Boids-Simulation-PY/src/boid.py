"""
Structure of Arrays (SoA) boid system with spatial hashing for maximum performance.
All boid data stored in NumPy arrays, updated via Numba-compiled functions.
"""
import numpy as np
import math
from numba import njit, prange

# Constants
WORLD_WIDTH = 200.0
WORLD_HEIGHT = 200.0
GRID_X1 = 100.0
GRID_Y1 = 100.0
GRID_X2 = -100.0
GRID_Y2 = -100.0


@njit(cache=True)
def wrapped_distance_sq(x1, y1, x2, y2, world_w, world_h):
    """Calculate squared wrapped distance (faster, no sqrt)."""
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    
    if dx > world_w * 0.5:
        dx = world_w - dx
    if dy > world_h * 0.5:
        dy = world_h - dy
    
    return dx * dx + dy * dy


@njit(cache=True)
def build_spatial_hash(positions_x, positions_y, cell_size, grid_x2, grid_y2, 
                       num_cells_x, num_cells_y):
    """
    Build spatial hash grid using counting sort approach.
    Returns:
        cell_counts: number of boids in each cell
        cell_starts: start index in sorted_indices for each cell
        sorted_indices: boid indices sorted by cell
    """
    n = len(positions_x)
    total_cells = num_cells_x * num_cells_y
    
    # Count boids per cell
    cell_counts = np.zeros(total_cells, dtype=np.int32)
    boid_cells = np.empty(n, dtype=np.int32)
    
    for i in range(n):
        cx = int((positions_x[i] - grid_x2) / cell_size)
        cy = int((positions_y[i] - grid_y2) / cell_size)
        # Clamp to valid range
        cx = max(0, min(cx, num_cells_x - 1))
        cy = max(0, min(cy, num_cells_y - 1))
        cell_idx = cy * num_cells_x + cx
        boid_cells[i] = cell_idx
        cell_counts[cell_idx] += 1
    
    # Compute start indices (prefix sum)
    cell_starts = np.empty(total_cells + 1, dtype=np.int32)
    cell_starts[0] = 0
    for i in range(total_cells):
        cell_starts[i + 1] = cell_starts[i] + cell_counts[i]
    
    # Sort boid indices by cell (counting sort)
    sorted_indices = np.empty(n, dtype=np.int32)
    cell_offsets = np.zeros(total_cells, dtype=np.int32)
    
    for i in range(n):
        cell_idx = boid_cells[i]
        pos = cell_starts[cell_idx] + cell_offsets[cell_idx]
        sorted_indices[pos] = i
        cell_offsets[cell_idx] += 1
    
    return cell_starts, sorted_indices, boid_cells


@njit(cache=True, parallel=True)
def update_all_boids(
    positions_x, positions_y,
    velocities_x, velocities_y,
    sizes,
    max_velocity, min_velocity, range_of_view,
    strength, repulsion_factor,
    slow_factor, confusion_factor, distance_factor,
    world_w, world_h,
    grid_x1, grid_y1, grid_x2, grid_y2,
    cell_starts, sorted_indices, boid_cells,
    num_cells_x, num_cells_y, cell_size
):
    """
    Update all boids using spatial hashing for O(n*k) complexity.
    """
    n = len(positions_x)
    rov_sq = range_of_view * range_of_view
    half_w = world_w * 0.5
    half_h = world_h * 0.5
    total_cells = num_cells_x * num_cells_y
    
    # Output velocities
    new_vx = np.empty(n)
    new_vy = np.empty(n)
    
    for i in prange(n):
        px = positions_x[i]
        py = positions_y[i]
        vx = velocities_x[i]
        vy = velocities_y[i]
        size = sizes[i]
        
        # Get this boid's cell
        cell_idx = boid_cells[i]
        cell_x = cell_idx % num_cells_x
        cell_y = cell_idx // num_cells_x
        
        # Find neighbors from adjacent cells only
        neighbor_count = 0
        avg_vx = 0.0
        avg_vy = 0.0
        avg_dx = 0.0
        avg_dy = 0.0
        sep_vx = 0.0
        sep_vy = 0.0
        
        # Check 3x3 neighborhood of cells (with wrapping)
        for dcx in range(-1, 2):
            for dcy in range(-1, 2):
                # Wrap cell coordinates
                ncx = (cell_x + dcx) % num_cells_x
                ncy = (cell_y + dcy) % num_cells_y
                neighbor_cell = ncy * num_cells_x + ncx
                
                # Iterate through boids in this cell
                start = cell_starts[neighbor_cell]
                end = cell_starts[neighbor_cell + 1]
                
                for idx in range(start, end):
                    j = sorted_indices[idx]
                    if i == j:
                        continue
                    
                    dist_sq = wrapped_distance_sq(
                        px, py,
                        positions_x[j], positions_y[j],
                        world_w, world_h
                    )
                    
                    if dist_sq <= rov_sq:
                        neighbor_count += 1
                        
                        # Alignment
                        avg_vx += velocities_x[j]
                        avg_vy += velocities_y[j]
                        
                        # Cohesion
                        dx = positions_x[j] - px
                        dy = positions_y[j] - py
                        
                        if dx > half_w:
                            dx -= world_w
                        elif dx < -half_w:
                            dx += world_w
                        if dy > half_h:
                            dy -= world_h
                        elif dy < -half_h:
                            dy += world_h
                        
                        avg_dx += dx
                        avg_dy += dy
                        
                        # Separation (sqrt needed to normalize direction vector)
                        collision_dist = size + sizes[j]
                        collision_dist_sq = collision_dist * collision_dist
                        
                        if dist_sq < collision_dist_sq and dist_sq > 0:
                            dist = math.sqrt(dist_sq)
                            sep_dx = px - positions_x[j]
                            sep_dy = py - positions_y[j]
                            if sep_dx > half_w:
                                sep_dx -= world_w
                            elif sep_dx < -half_w:
                                sep_dx += world_w
                            if sep_dy > half_h:
                                sep_dy -= world_h
                            elif sep_dy < -half_h:
                                sep_dy += world_h
                            sep_vx += (sep_dx / dist) * repulsion_factor
                            sep_vy += (sep_dy / dist) * repulsion_factor
        
        if neighbor_count > 0:
            confusion = 1.0 / (1.0 + neighbor_count * confusion_factor)
            
            # Alignment
            avg_vx /= neighbor_count
            avg_vy /= neighbor_count
            alignment_strength = strength * confusion
            vx += (avg_vx - vx) * alignment_strength
            vy += (avg_vy - vy) * alignment_strength
            
            # Cohesion
            avg_dx /= neighbor_count
            avg_dy /= neighbor_count
            dist_sq = avg_dx * avg_dx + avg_dy * avg_dy
            distance_attenuation = 1.0 / (1.0 + dist_sq * distance_factor)
            cohesion_strength = strength * 0.5 * distance_attenuation * confusion
            vx += avg_dx * cohesion_strength
            vy += avg_dy * cohesion_strength
            
            # Separation
            vx += sep_vx
            vy += sep_vy
        else:
            # Slow down when alone (sqrt needed for speed ratio)
            vel_sq = vx * vx + vy * vy
            if vel_sq > min_velocity * min_velocity:
                speed = math.sqrt(vel_sq)
                new_speed = speed - slow_factor
                if new_speed < min_velocity:
                    new_speed = min_velocity
                scale = new_speed / speed
                vx *= scale
                vy *= scale
        
        # Limit velocity (sqrt needed for scaling ratio)
        vel_sq = vx * vx + vy * vy
        max_vel_sq = max_velocity * max_velocity
        min_vel_sq = (min_velocity * 0.8) * (min_velocity * 0.8)
        
        if vel_sq > max_vel_sq:
            scale = max_velocity / math.sqrt(vel_sq)
            vx *= scale
            vy *= scale
        elif vel_sq < min_vel_sq and vel_sq > 0:
            scale = (min_velocity * 0.8) / math.sqrt(vel_sq)
            vx *= scale
            vy *= scale
        
        new_vx[i] = vx
        new_vy[i] = vy
    
    return new_vx, new_vy


@njit(cache=True, parallel=True)
def move_and_wrap(positions_x, positions_y, velocities_x, velocities_y,
                  grid_x1, grid_y1, grid_x2, grid_y2):
    """Apply velocity and wrap positions."""
    n = len(positions_x)
    
    for i in prange(n):
        # Move
        positions_x[i] += velocities_x[i]
        positions_y[i] += velocities_y[i]
        
        # Wrap
        if positions_x[i] > grid_x1:
            positions_x[i] = grid_x2
        elif positions_x[i] < grid_x2:
            positions_x[i] = grid_x1
        
        if positions_y[i] > grid_y1:
            positions_y[i] = grid_y2
        elif positions_y[i] < grid_y2:
            positions_y[i] = grid_y1


@njit(cache=True, parallel=True)
def move_and_bounce(positions_x, positions_y, velocities_x, velocities_y,
                    grid_x1, grid_y1, grid_x2, grid_y2):
    """Apply velocity and bounce off walls."""
    n = len(positions_x)
    
    for i in prange(n):
        # Move
        positions_x[i] += velocities_x[i]
        positions_y[i] += velocities_y[i]
        
        # Bounce off walls
        if positions_x[i] > grid_x1:
            positions_x[i] = grid_x1
            velocities_x[i] = -velocities_x[i]
        elif positions_x[i] < grid_x2:
            positions_x[i] = grid_x2
            velocities_x[i] = -velocities_x[i]
        
        if positions_y[i] > grid_y1:
            positions_y[i] = grid_y1
            velocities_y[i] = -velocities_y[i]
        elif positions_y[i] < grid_y2:
            positions_y[i] = grid_y2
            velocities_y[i] = -velocities_y[i]


class BoidSystem:
    """
    Manages all boids using Structure of Arrays with spatial hashing for performance.
    """
    def __init__(self):
        self.max_count = 5000  # Pre-allocate for max boids
        
        # Structure of Arrays
        self.positions_x = np.zeros(self.max_count, dtype=np.float64)
        self.positions_y = np.zeros(self.max_count, dtype=np.float64)
        self.velocities_x = np.zeros(self.max_count, dtype=np.float64)
        self.velocities_y = np.zeros(self.max_count, dtype=np.float64)
        self.sizes = np.zeros(self.max_count, dtype=np.float64)
        
        self.count = 0
        
        # Shared parameters (same for all boids)
        self.max_velocity = 1.5
        self.min_velocity = 0.5
        self.range_of_view = 3.0
        self.strength = 0.15
        self.repulsion_factor = 0.03
        self.random_factor = 0.25
        self.slow_factor = 1.0
        self.confusion_factor = 0.20
        self.distance_factor = 0.05
        self.default_size = 7.0
        self.wrap_mode = True
        
        # Spatial hash grid parameters
        self._update_grid_params()
    
    def _update_grid_params(self):
        """Update spatial hash grid parameters based on range_of_view."""
        # Cell size should be >= range_of_view so we only need to check 3x3 cells
        self.cell_size = max(self.range_of_view, 1.0)
        self.num_cells_x = max(1, int(np.ceil(WORLD_WIDTH / self.cell_size)))
        self.num_cells_y = max(1, int(np.ceil(WORLD_HEIGHT / self.cell_size)))
    
    def add_boids(self, n):
        """Add multiple boids at once."""
        space = self.max_count - self.count
        to_add = min(n, space)
        
        if to_add > 0:
            start = self.count
            end = start + to_add
            self.positions_x[start:end] = np.random.uniform(GRID_X2, GRID_X1, to_add)
            self.positions_y[start:end] = np.random.uniform(GRID_Y2, GRID_Y1, to_add)
            self.velocities_x[start:end] = np.random.uniform(-0.1, 0.1, to_add)
            self.velocities_y[start:end] = np.random.uniform(-0.1, 0.1, to_add)
            self.sizes[start:end] = self.default_size
            self.count += to_add
    
    def remove_boids(self, n):
        """Remove boids from the end."""
        self.count = max(0, self.count - n)
    
    def set_count(self, target_count):
        """Set the boid count to a specific value."""
        target_count = max(0, min(target_count, self.max_count))
        if target_count > self.count:
            self.add_boids(target_count - self.count)
        elif target_count < self.count:
            self.count = target_count
    
    def update(self):
        """Update all boids using spatial hashing."""
        if self.count == 0:
            return
        
        # Get active slices
        n = self.count
        px = self.positions_x[:n]
        py = self.positions_y[:n]
        vx = self.velocities_x[:n]
        vy = self.velocities_y[:n]
        sizes = self.sizes[:n]
        
        # Build spatial hash grid (Numba compiled)
        cell_starts, sorted_indices, boid_cells = build_spatial_hash(
            px, py, self.cell_size, GRID_X2, GRID_Y2,
            self.num_cells_x, self.num_cells_y
        )
        
        # Compute new velocities with spatial hashing (Numba compiled, parallel)
        new_vx, new_vy = update_all_boids(
            px, py, vx, vy, sizes,
            self.max_velocity, self.min_velocity, self.range_of_view,
            self.strength, self.repulsion_factor,
            self.slow_factor, self.confusion_factor, self.distance_factor,
            WORLD_WIDTH, WORLD_HEIGHT,
            GRID_X1, GRID_Y1, GRID_X2, GRID_Y2,
            cell_starts, sorted_indices, boid_cells,
            self.num_cells_x, self.num_cells_y, self.cell_size
        )
        
        # Apply new velocities
        self.velocities_x[:n] = new_vx
        self.velocities_y[:n] = new_vy
        
        # Add randomness (done outside Numba for proper random)
        self.velocities_x[:n] += (np.random.random(n) - 0.5) * self.random_factor
        self.velocities_y[:n] += (np.random.random(n) - 0.5) * self.random_factor
        
        # Move and wrap/bounce
        if self.wrap_mode:
            move_and_wrap(
                self.positions_x[:n], self.positions_y[:n],
                self.velocities_x[:n], self.velocities_y[:n],
                GRID_X1, GRID_Y1, GRID_X2, GRID_Y2
            )
        else:
            move_and_bounce(
                self.positions_x[:n], self.positions_y[:n],
                self.velocities_x[:n], self.velocities_y[:n],
                GRID_X1, GRID_Y1, GRID_X2, GRID_Y2
            )
    
    def get_boid_data(self):
        """Return boid data for broadcasting."""
        n = self.count
        return [
            {
                'x': float(self.positions_x[i]),
                'y': float(self.positions_y[i]),
                'vx': float(self.velocities_x[i]),
                'vy': float(self.velocities_y[i]),
                'size': float(self.sizes[i])
            }
            for i in range(n)
        ]
    
    def update_param(self, param, value):
        """Update a simulation parameter."""
        if hasattr(self, param):
            setattr(self, param, value)
            # Update grid if range_of_view changed
            if param == 'range_of_view':
                self._update_grid_params()
            # Update all existing boids sizes if default_size changed
            if param == 'default_size':
                self.sizes[:self.count] = value
    
    def reset(self):
        """Reset the boid system to initial state with same number of boids."""
        current_count = self.count
        # Clear all arrays
        self.positions_x[:] = 0
        self.positions_y[:] = 0
        self.velocities_x[:] = 0
        self.velocities_y[:] = 0
        self.sizes[:] = 0
        self.count = 0
        # Re-add boids at random positions
        if current_count > 0:
            self.add_boids(current_count)
