import os
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from src.main import GRID_X1, GRID_Y1, GRID_X2, GRID_Y2

template_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'templates'))
app = Flask(__name__, template_folder=template_dir)
app.config['SECRET_KEY'] = 'boids-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

simulation_state = {
    'boids': [],
    'paused': False,
    'frame': 0,
    'lock': threading.Lock()
}

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('init', {'boid_count': len(simulation_state['boids'])})

@socketio.on('pause')
def handle_pause():
    simulation_state['paused'] = not simulation_state['paused']
    socketio.emit('paused', simulation_state['paused'], to=None)

def broadcast_boids():
    with simulation_state['lock']:
        boids_array = []
        for boid in simulation_state['boids']:
            boids_array.append([
                boid.position['x'],
                boid.position['y'],
                boid.velocity['x'],
                boid.velocity['y'],
                boid.size
            ])
        
        socketio.server.emit('boids', {
            'b': boids_array,
            'f': simulation_state['frame'],
            'p': simulation_state['paused']
        })

if __name__ == '__main__':
    socketio.run(app, debug=False, host='0.0.0.0', port=5000)
