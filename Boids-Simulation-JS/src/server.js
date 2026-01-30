import express from 'express';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const projectRoot = path.dirname(__dirname);

const app = express();
const PORT = process.env.PORT || 3000;

// Serve static files from project root
app.use(express.static(projectRoot));

// Main route
app.get('/', (req, res) => {
  res.sendFile(path.join(projectRoot, 'index.html'));
});

// Start server
app.listen(PORT, () => {
  console.log(`
╔═══════════════════════════════════════════════════════╗
║          🎯 Boids Simulation Server Started           ║
╠═══════════════════════════════════════════════════════╣
║                                                       ║
║  Server running on: http://localhost:${PORT}${' '.repeat(15 - PORT.toString().length)}
║                                                       ║
║  Access from remote: http://<your-ip>:${PORT}${' '.repeat(15 - PORT.toString().length)}
║                                                       ║
║  Press Ctrl+C to stop                                ║
║                                                       ║
╚═══════════════════════════════════════════════════════╝
  `);
});
