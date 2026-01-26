import os
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

template_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'templates'))
app = Flask(__name__, template_folder=template_dir)
app.config['SECRET_KEY'] = 'boids-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

simulation_state = {
    'boid_system': None,
    'paused': False,
    'frame': 0,
    'lock': threading.Lock()
}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/pause', methods=['POST'])
def api_pause():
    """HTTP endpoint to pause the simulation"""
    simulation_state['paused'] = True
    socketio.emit('simulation_paused', {'status': 'paused'})
    return {'status': 'paused'}

@app.route('/api/resume', methods=['POST'])
def api_resume():
    """HTTP endpoint to resume the simulation"""
    simulation_state['paused'] = False
    socketio.emit('simulation_resumed', {'status': 'running'})
    return {'status': 'running'}

@app.route('/api/reset', methods=['POST'])
def api_reset():
    """HTTP endpoint to reset the simulation"""
    with simulation_state['lock']:
        boid_system = simulation_state.get('boid_system')
        if boid_system:
            simulation_state['frame'] = 0
            boid_system.reset()
            socketio.emit('simulation_reset', {
                'status': 'reset',
                'boid_count': boid_system.count,
                'frame': simulation_state['frame']
            })
    return {'status': 'reset'}

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    boid_system = simulation_state.get('boid_system')
    if boid_system:
        emit('init', {
            'boid_count': boid_system.count,
            'grid_x1': 100.0,
            'grid_y1': 100.0,
            'grid_x2': -100.0,
            'grid_y2': -100.0,
            'max_velocity': boid_system.max_velocity,
            'min_velocity': boid_system.min_velocity
        })
    else:
        emit('init', {'boid_count': 0})

@socketio.on('pause')
def handle_pause():
    simulation_state['paused'] = not simulation_state['paused']
    socketio.emit('paused', simulation_state['paused'], to=None)

@socketio.on('update_param')
def handle_update_param(data):
    """Update a parameter on the boid system"""
    param_name = data.get('param')
    param_value = data.get('value')
    
    with simulation_state['lock']:
        boid_system = simulation_state.get('boid_system')
        if boid_system:
            boid_system.update_param(param_name, param_value)

@socketio.on('set_boid_count')
def handle_set_boid_count(data):
    """Add or remove boids to reach target count"""
    target_count = data.get('count', 0)
    
    with simulation_state['lock']:
        boid_system = simulation_state.get('boid_system')
        if boid_system:
            boid_system.set_count(target_count)

@socketio.on('pause_simulation')
def handle_pause_simulation():
    """Pause the simulation"""
    simulation_state['paused'] = True
    socketio.emit('simulation_paused', {'status': 'paused'}, to=None)

@socketio.on('resume_simulation')
def handle_resume_simulation():
    """Resume the simulation"""
    simulation_state['paused'] = False
    socketio.emit('simulation_resumed', {'status': 'running'}, to=None)

@socketio.on('reset_simulation')
def handle_reset_simulation():
    """Reset the simulation to initial state"""
    with simulation_state['lock']:
        boid_system = simulation_state.get('boid_system')
        if boid_system:
            # Reset frame counter
            simulation_state['frame'] = 0
            # Reset boid system
            boid_system.reset()
            # Emit reset confirmation
            socketio.emit('simulation_reset', {
                'status': 'reset',
                'boid_count': boid_system.count,
                'frame': simulation_state['frame']
            }, to=None)

def broadcast_boids():
    with simulation_state['lock']:
        boid_system = simulation_state.get('boid_system')
        if not boid_system:
            return
        
        n = boid_system.count
        boids_array = []
        for i in range(n):
            boids_array.append([
                boid_system.positions_x[i],
                boid_system.positions_y[i],
                boid_system.velocities_x[i],
                boid_system.velocities_y[i],
                boid_system.sizes[i]
            ])
        
        socketio.server.emit('boids', {
            'b': boids_array,
            'f': simulation_state['frame'],
            'p': simulation_state['paused']
        })

if __name__ == '__main__':
    socketio.run(app, debug=False, host='0.0.0.0', port=5000)
