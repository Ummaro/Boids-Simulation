import BoidSystem from './Boid.js';

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
let lastUpdateTime = Date.now();
let frameCount = 0, fps = 0;

let gridX1 = 100, gridY1 = 100, gridX2 = -100, gridY2 = -100;

// Initialize boid system
const boidSystem = new BoidSystem();

let isPaused = false;
let minVelocity = boidSystem.minVelocity;
let maxVelocity = boidSystem.maxVelocity;

function getColorForVelocity(velocity) {
    const clamped = Math.max(minVelocity, Math.min(maxVelocity, velocity));
    const normalized = (maxVelocity > minVelocity) ? (clamped - minVelocity) / (maxVelocity - minVelocity) : 0;
    
    const coldR = 100, coldG = 200, coldB = 255;
    const warmR = 255, warmG = 100, warmB = 50;
    
    const r = Math.round(coldR + (warmR - coldR) * normalized);
    const g = Math.round(coldG + (warmG - coldG) * normalized);
    const b = Math.round(coldB + (warmB - coldB) * normalized);
    
    return `rgb(${r}, ${g}, ${b})`;
}

function resizeCanvas() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
}
resizeCanvas();
window.addEventListener('resize', resizeCanvas);

function worldToScreen(x, y) {
    const sx = ((x - gridX2) / (gridX1 - gridX2)) * canvas.width;
    const sy = canvas.height - ((y - gridY2) / (gridY1 - gridY2)) * canvas.height;
    return { sx, sy };
}

function drawArrow(fromX, fromY, toX, toY, size) {
    const headlen = size * 2;
    const angle = Math.atan2(toY - fromY, toX - fromX);
    
    ctx.beginPath();
    ctx.moveTo(fromX, fromY);
    ctx.lineTo(toX, toY);
    ctx.stroke();
    
    ctx.beginPath();
    ctx.moveTo(toX, toY);
    ctx.lineTo(toX - headlen * Math.cos(angle - Math.PI / 6), toY - headlen * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(toX - headlen * Math.cos(angle + Math.PI / 6), toY - headlen * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.fill();
}

function draw() {
    ctx.fillStyle = 'rgb(26, 26, 46)';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    ctx.lineWidth = 2;
    
    const boids = boidSystem.getBoidData();
    
    for (const boid of boids) {
        const screen = worldToScreen(boid.x, boid.y);
        const actualVelocity = Math.sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
        const size = boid.size || 3;
        
        const scaleFactor = size / 3;
        const vx = boid.vx * 5 * scaleFactor;
        const vy = -boid.vy * 5 * scaleFactor;
        
        const color = getColorForVelocity(actualVelocity);
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        
        if (actualVelocity > 0.1) {
            drawArrow(screen.sx, screen.sy, screen.sx + vx, screen.sy + vy, size);
        } else {
            ctx.beginPath();
            ctx.arc(screen.sx, screen.sy, size, 0, Math.PI * 2);
            ctx.fill();
        }
    }
    
    document.getElementById('frame').textContent = frameCount;
    document.getElementById('boid-count').textContent = boidSystem.count;
    
    frameCount++;
    const now = Date.now();
    if (now - lastUpdateTime >= 1000) {
        fps = frameCount;
        frameCount = 0;
        lastUpdateTime = now;
        document.getElementById('fps').textContent = fps;
    }
}

function animate() {
    if (!isPaused) {
        boidSystem.update();
    }
    draw();
    requestAnimationFrame(animate);
}

// Pause button
document.getElementById('pause-btn').addEventListener('click', () => {
    isPaused = !isPaused;
    document.getElementById('pause-btn').textContent = isPaused ? 'Resume' : 'Pause';
});

// Parameter sliders
const params = ['strength', 'range_of_view', 'random_factor', 'max_velocity', 'min_velocity', 'repulsion_factor', 'slow_factor', 'confusion_factor', 'distance_factor', 'default_size'];
const paramLabels = {
    'strength': 'strength-val',
    'range_of_view': 'range-val',
    'random_factor': 'random-val',
    'max_velocity': 'maxvel-val',
    'min_velocity': 'minvel-val',
    'repulsion_factor': 'repulsion-val',
    'slow_factor': 'slow-val',
    'confusion_factor': 'confusion-val',
    'distance_factor': 'distance-val',
    'default_size': 'size-val'
};

params.forEach(param => {
    const slider = document.getElementById(param);
    const label = document.getElementById(paramLabels[param]);
    
    slider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        
        // Convert snake_case to camelCase for boidSystem
        const camelParam = param.replace(/_([a-z])/g, (g) => g[1].toUpperCase());
        
        label.textContent = param === 'range_of_view' ? value : value.toFixed(2);
        boidSystem.updateParam(camelParam, value);
        
        if (param === 'max_velocity') maxVelocity = value;
        if (param === 'min_velocity') minVelocity = value;
    });
});

// Boid count slider
const boidCountSlider = document.getElementById('boid_count');
const boidCountLabel = document.getElementById('boidcount-val');
boidCountSlider.addEventListener('input', (e) => {
    const count = parseInt(e.target.value);
    boidCountLabel.textContent = count;
    boidSystem.setCount(count);
});

// Wrap mode checkbox
const wrapModeCheckbox = document.getElementById('wrap_mode');
wrapModeCheckbox.addEventListener('change', (e) => {
    boidSystem.updateParam('wrapMode', e.target.checked);
});

// Initialize with some boids
boidSystem.addBoids(750);
boidCountSlider.value = 750;
boidCountLabel.textContent = '750';

// Set initial slider values
document.getElementById('strength').value = boidSystem.strength;
document.getElementById('strength-val').textContent = boidSystem.strength.toFixed(2);

document.getElementById('range_of_view').value = boidSystem.rangeOfView;
document.getElementById('range-val').textContent = boidSystem.rangeOfView.toFixed(2);

document.getElementById('max_velocity').value = boidSystem.maxVelocity;
document.getElementById('maxvel-val').textContent = boidSystem.maxVelocity.toFixed(2);

document.getElementById('min_velocity').value = boidSystem.minVelocity;
document.getElementById('minvel-val').textContent = boidSystem.minVelocity.toFixed(2);

document.getElementById('random_factor').value = boidSystem.randomFactor;
document.getElementById('random-val').textContent = boidSystem.randomFactor.toFixed(2);

document.getElementById('repulsion_factor').value = boidSystem.repulsionFactor;
document.getElementById('repulsion-val').textContent = boidSystem.repulsionFactor.toFixed(2);

document.getElementById('slow_factor').value = boidSystem.slowFactor;
document.getElementById('slow-val').textContent = boidSystem.slowFactor.toFixed(2);

document.getElementById('confusion_factor').value = boidSystem.confusionFactor;
document.getElementById('confusion-val').textContent = boidSystem.confusionFactor.toFixed(2);

document.getElementById('distance_factor').value = boidSystem.distanceFactor;
document.getElementById('distance-val').textContent = boidSystem.distanceFactor.toFixed(2);

document.getElementById('default_size').value = boidSystem.defaultSize;
document.getElementById('size-val').textContent = boidSystem.defaultSize.toFixed(2);

animate();
