import threading
import time
import random
from src.app import app, socketio, simulation_state, broadcast_boids
from src.Boid import Boid
from src.main import GRID_X1, GRID_Y1, GRID_X2, GRID_Y2

def run_simulation(boids):
    frame = 0
    
    while True:
        if not simulation_state['paused']:
            with simulation_state['lock']:
                grid = {}
                for boid in boids:
                    boid_grid_pos = boid.get_grid_pos()
                    grid.setdefault(boid_grid_pos, []).append(boid)
                
                for boid in boids:
                    boid.update(boids, grid)
                
                simulation_state['frame'] = frame
            
            broadcast_boids()
            frame += 1
        
        time.sleep(0.01)

def start_server():
    boids = []
    count = 1000
    for _ in range(count):
        position = {
            'x': random.uniform(GRID_X2, GRID_X1),
            'y': random.uniform(GRID_Y2, GRID_Y1)
        }
        velocity = {
            'x': random.uniform(-0.1, 0.1),
            'y': random.uniform(-0.1, 0.1)
        }
        Boid(
            position=position,
            velocity=velocity,
            max_velocity=1.0,
            min_velocity=0.5,
            range_of_view=10,
            size=2,
            strength=0.01,
            repulsion_factor=0.01,
            random_factor=0.05,
            slow_factor=0.0005,
            boids=boids,
        )
    
    simulation_state['boids'] = boids
    
    sim_thread = threading.Thread(target=run_simulation, args=(boids,), daemon=True)
    sim_thread.start()
    
    print("Starting Boids Simulation Server...")
    print("Open http://localhost:5000 in your browser")
    socketio.run(app, debug=False, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)

if __name__ == "__main__":
    start_server()
