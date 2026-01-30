import threading
import time
from src.app import app, socketio, simulation_state, broadcast_boids
from src.boid import BoidSystem

def run_simulation(boid_system):
    frame = 0
    
    while True:
        if not simulation_state['paused']:
            with simulation_state['lock']:
                boid_system.update()
                simulation_state['frame'] = frame
            
            broadcast_boids()
            frame += 1
        
        time.sleep(0.01)

def start_server():
    boid_system = BoidSystem()
    boid_system.add_boids(750)
    
    simulation_state['boid_system'] = boid_system
    
    sim_thread = threading.Thread(target=run_simulation, args=(boid_system,), daemon=True)
    sim_thread.start()
    
    print("Starting Boids Simulation Server...")
    print("Open http://localhost:5000 in your browser")
    socketio.run(
        app,
        debug=False,
        host='0.0.0.0',
        port=5000,
        allow_unsafe_werkzeug=True
    )

if __name__ == "__main__":
    start_server()
