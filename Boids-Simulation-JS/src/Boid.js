/**
 * Structure of Arrays (SoA) boid system with spatial hashing
 * All boid data stored in parallel arrays for maximum performance
 */

const WORLD_WIDTH = 200.0;
const WORLD_HEIGHT = 200.0;
const GRID_X1 = 100.0;
const GRID_Y1 = 100.0;
const GRID_X2 = -100.0;
const GRID_Y2 = -100.0;

/**
 * Calculate squared wrapped distance between two points
 * @param {number} x1
 * @param {number} y1
 * @param {number} x2
 * @param {number} y2
 * @param {number} worldW
 * @param {number} worldH
 * @returns {number}
 */
function wrappedDistanceSq(x1, y1, x2, y2, worldW, worldH) {
  let dx = Math.abs(x1 - x2);
  let dy = Math.abs(y1 - y2);

  if (dx > worldW * 0.5) {
    dx = worldW - dx;
  }
  if (dy > worldH * 0.5) {
    dy = worldH - dy;
  }

  return dx * dx + dy * dy;
}

/**
 * Build spatial hash grid
 * @returns {Object} { cellStarts, sortedIndices, boidCells }
 */
function buildSpatialHash(
  positionsX,
  positionsY,
  cellSize,
  gridX2,
  gridY2,
  numCellsX,
  numCellsY,
  count
) {
  const totalCells = numCellsX * numCellsY;
  const cellCounts = new Int32Array(totalCells);
  const boidCells = new Int32Array(count);

  // Count boids per cell
  for (let i = 0; i < count; i++) {
    let cx = Math.floor((positionsX[i] - gridX2) / cellSize);
    let cy = Math.floor((positionsY[i] - gridY2) / cellSize);

    // Clamp to valid range
    cx = Math.max(0, Math.min(cx, numCellsX - 1));
    cy = Math.max(0, Math.min(cy, numCellsY - 1));

    const cellIdx = cy * numCellsX + cx;
    boidCells[i] = cellIdx;
    cellCounts[cellIdx]++;
  }

  // Compute start indices (prefix sum)
  const cellStarts = new Int32Array(totalCells + 1);
  cellStarts[0] = 0;
  for (let i = 0; i < totalCells; i++) {
    cellStarts[i + 1] = cellStarts[i] + cellCounts[i];
  }

  // Sort boid indices by cell
  const sortedIndices = new Int32Array(count);
  const cellOffsets = new Int32Array(totalCells);

  for (let i = 0; i < count; i++) {
    const cellIdx = boidCells[i];
    const pos = cellStarts[cellIdx] + cellOffsets[cellIdx];
    sortedIndices[pos] = i;
    cellOffsets[cellIdx]++;
  }

  return { cellStarts, sortedIndices, boidCells };
}

class BoidSystem {
  constructor() {
    this.maxCount = 5000;

    // Structure of Arrays
    this.positionsX = new Float64Array(this.maxCount);
    this.positionsY = new Float64Array(this.maxCount);
    this.velocitiesX = new Float64Array(this.maxCount);
    this.velocitiesY = new Float64Array(this.maxCount);
    this.sizes = new Float64Array(this.maxCount);

    this.count = 0;

    // Shared parameters
    this.maxVelocity = 1.5;
    this.minVelocity = 0.5;
    this.rangeOfView = 3.0;
    this.strength = 0.15;
    this.repulsionFactor = 0.03;
    this.randomFactor = 0.25;
    this.slowFactor = 1.0;
    this.confusionFactor = 0.2;
    this.distanceFactor = 0.05;
    this.defaultSize = 7.0;
    this.wrapMode = true;

    // Spatial hash grid parameters
    this.updateGridParams();
  }

  updateGridParams() {
    this.cellSize = Math.max(this.rangeOfView, 1.0);
    this.numCellsX = Math.max(1, Math.ceil(WORLD_WIDTH / this.cellSize));
    this.numCellsY = Math.max(1, Math.ceil(WORLD_HEIGHT / this.cellSize));
  }

  addBoids(n) {
    const space = this.maxCount - this.count;
    const toAdd = Math.min(n, space);

    if (toAdd > 0) {
      const start = this.count;
      const end = start + toAdd;

      for (let i = start; i < end; i++) {
        this.positionsX[i] = GRID_X2 + Math.random() * WORLD_WIDTH;
        this.positionsY[i] = GRID_Y2 + Math.random() * WORLD_HEIGHT;
        this.velocitiesX[i] = (Math.random() - 0.5) * 0.2;
        this.velocitiesY[i] = (Math.random() - 0.5) * 0.2;
        this.sizes[i] = this.defaultSize;
      }

      this.count += toAdd;
    }
  }

  removeBoids(n) {
    this.count = Math.max(0, this.count - n);
  }

  setCount(targetCount) {
    targetCount = Math.max(0, Math.min(targetCount, this.maxCount));
    if (targetCount > this.count) {
      this.addBoids(targetCount - this.count);
    } else if (targetCount < this.count) {
      this.count = targetCount;
    }
  }

  update() {
    if (this.count === 0) {
      return;
    }

    const n = this.count;
    const rovSq = this.rangeOfView * this.rangeOfView;
    const halfW = WORLD_WIDTH * 0.5;
    const halfH = WORLD_HEIGHT * 0.5;

    // Build spatial hash grid
    const { cellStarts, sortedIndices, boidCells } = buildSpatialHash(
      this.positionsX,
      this.positionsY,
      this.cellSize,
      GRID_X2,
      GRID_Y2,
      this.numCellsX,
      this.numCellsY,
      n
    );

    // Update velocities using spatial hashing
    const newVx = new Float64Array(n);
    const newVy = new Float64Array(n);

    for (let i = 0; i < n; i++) {
      const px = this.positionsX[i];
      const py = this.positionsY[i];
      let vx = this.velocitiesX[i];
      let vy = this.velocitiesY[i];
      const size = this.sizes[i];

      const cellIdx = boidCells[i];
      const cellX = cellIdx % this.numCellsX;
      const cellY = Math.floor(cellIdx / this.numCellsX);

      let neighborCount = 0;
      let avgVx = 0.0;
      let avgVy = 0.0;
      let avgDx = 0.0;
      let avgDy = 0.0;
      let sepVx = 0.0;
      let sepVy = 0.0;

      // Check 3x3 neighborhood of cells
      for (let dcx = -1; dcx <= 1; dcx++) {
        for (let dcy = -1; dcy <= 1; dcy++) {
          const ncx = (cellX + dcx + this.numCellsX) % this.numCellsX;
          const ncy = (cellY + dcy + this.numCellsY) % this.numCellsY;
          const neighborCell = ncy * this.numCellsX + ncx;

          const start = cellStarts[neighborCell];
          const end = cellStarts[neighborCell + 1];

          for (let idx = start; idx < end; idx++) {
            const j = sortedIndices[idx];
            if (i === j) {
              continue;
            }

            const distSq = wrappedDistanceSq(
              px,
              py,
              this.positionsX[j],
              this.positionsY[j],
              WORLD_WIDTH,
              WORLD_HEIGHT
            );

            if (distSq <= rovSq) {
              neighborCount++;

              // Alignment
              avgVx += this.velocitiesX[j];
              avgVy += this.velocitiesY[j];

              // Cohesion
              let dx = this.positionsX[j] - px;
              let dy = this.positionsY[j] - py;

              if (dx > halfW) {
                dx -= WORLD_WIDTH;
              } else if (dx < -halfW) {
                dx += WORLD_WIDTH;
              }
              if (dy > halfH) {
                dy -= WORLD_HEIGHT;
              } else if (dy < -halfH) {
                dy += WORLD_HEIGHT;
              }

              avgDx += dx;
              avgDy += dy;

              // Separation
              const collisionDist = size + this.sizes[j];
              const collisionDistSq = collisionDist * collisionDist;

              if (distSq < collisionDistSq && distSq > 0) {
                const dist = Math.sqrt(distSq);
                let sepDx = px - this.positionsX[j];
                let sepDy = py - this.positionsY[j];

                if (sepDx > halfW) {
                  sepDx -= WORLD_WIDTH;
                } else if (sepDx < -halfW) {
                  sepDx += WORLD_WIDTH;
                }
                if (sepDy > halfH) {
                  sepDy -= WORLD_HEIGHT;
                } else if (sepDy < -halfH) {
                  sepDy += WORLD_HEIGHT;
                }

                sepVx += (sepDx / dist) * this.repulsionFactor;
                sepVy += (sepDy / dist) * this.repulsionFactor;
              }
            }
          }
        }
      }

      if (neighborCount > 0) {
        const confusion =
          1.0 / (1.0 + neighborCount * this.confusionFactor);

        // Alignment
        avgVx /= neighborCount;
        avgVy /= neighborCount;
        const alignmentStrength = this.strength * confusion;
        vx += (avgVx - vx) * alignmentStrength;
        vy += (avgVy - vy) * alignmentStrength;

        // Cohesion
        avgDx /= neighborCount;
        avgDy /= neighborCount;
        const distSq = avgDx * avgDx + avgDy * avgDy;
        const distanceAttenuation =
          1.0 / (1.0 + distSq * this.distanceFactor);
        const cohesionStrength =
          this.strength * 0.5 * distanceAttenuation * confusion;
        vx += avgDx * cohesionStrength;
        vy += avgDy * cohesionStrength;

        // Separation
        vx += sepVx;
        vy += sepVy;
      } else {
        // Slow down when alone
        const velSq = vx * vx + vy * vy;
        if (velSq > this.minVelocity * this.minVelocity) {
          const speed = Math.sqrt(velSq);
          let newSpeed = speed - this.slowFactor;
          if (newSpeed < this.minVelocity) {
            newSpeed = this.minVelocity;
          }
          const scale = newSpeed / speed;
          vx *= scale;
          vy *= scale;
        }
      }

      // Limit velocity
      const velSq = vx * vx + vy * vy;
      const maxVelSq = this.maxVelocity * this.maxVelocity;
      const minVelSq =
        this.minVelocity * 0.8 * (this.minVelocity * 0.8);

      if (velSq > maxVelSq) {
        const scale = this.maxVelocity / Math.sqrt(velSq);
        vx *= scale;
        vy *= scale;
      } else if (velSq < minVelSq && velSq > 0) {
        const scale = (this.minVelocity * 0.8) / Math.sqrt(velSq);
        vx *= scale;
        vy *= scale;
      }

      newVx[i] = vx;
      newVy[i] = vy;
    }

    // Apply new velocities and randomness
    for (let i = 0; i < n; i++) {
      this.velocitiesX[i] =
        newVx[i] + (Math.random() - 0.5) * this.randomFactor;
      this.velocitiesY[i] =
        newVy[i] + (Math.random() - 0.5) * this.randomFactor;
    }

    // Move and wrap/bounce
    if (this.wrapMode) {
      this.moveAndWrap(n);
    } else {
      this.moveAndBounce(n);
    }
  }

  moveAndWrap(n) {
    for (let i = 0; i < n; i++) {
      this.positionsX[i] += this.velocitiesX[i];
      this.positionsY[i] += this.velocitiesY[i];

      // Wrap
      if (this.positionsX[i] > GRID_X1) {
        this.positionsX[i] = GRID_X2;
      } else if (this.positionsX[i] < GRID_X2) {
        this.positionsX[i] = GRID_X1;
      }

      if (this.positionsY[i] > GRID_Y1) {
        this.positionsY[i] = GRID_Y2;
      } else if (this.positionsY[i] < GRID_Y2) {
        this.positionsY[i] = GRID_Y1;
      }
    }
  }

  moveAndBounce(n) {
    for (let i = 0; i < n; i++) {
      this.positionsX[i] += this.velocitiesX[i];
      this.positionsY[i] += this.velocitiesY[i];

      // Bounce off walls
      if (this.positionsX[i] > GRID_X1) {
        this.positionsX[i] = GRID_X1;
        this.velocitiesX[i] = -this.velocitiesX[i];
      } else if (this.positionsX[i] < GRID_X2) {
        this.positionsX[i] = GRID_X2;
        this.velocitiesX[i] = -this.velocitiesX[i];
      }

      if (this.positionsY[i] > GRID_Y1) {
        this.positionsY[i] = GRID_Y1;
        this.velocitiesY[i] = -this.velocitiesY[i];
      } else if (this.positionsY[i] < GRID_Y2) {
        this.positionsY[i] = GRID_Y2;
        this.velocitiesY[i] = -this.velocitiesY[i];
      }
    }
  }

  getBoidData() {
    const data = [];
    for (let i = 0; i < this.count; i++) {
      data.push({
        x: this.positionsX[i],
        y: this.positionsY[i],
        vx: this.velocitiesX[i],
        vy: this.velocitiesY[i],
        size: this.sizes[i],
      });
    }
    return data;
  }

  updateParam(param, value) {
    if (param in this) {
      this[param] = value;

      // Update grid if range_of_view changed
      if (param === "rangeOfView") {
        this.updateGridParams();
      }

      // Update all existing boids sizes if default_size changed
      if (param === "defaultSize") {
        for (let i = 0; i < this.count; i++) {
          this.sizes[i] = value;
        }
      }
    }
  }

  reset() {
    const currentCount = this.count;

    // Clear all arrays
    this.positionsX.fill(0);
    this.positionsY.fill(0);
    this.velocitiesX.fill(0);
    this.velocitiesY.fill(0);
    this.sizes.fill(0);
    this.count = 0;

    // Re-add boids at random positions
    if (currentCount > 0) {
      this.addBoids(currentCount);
    }
  }
}

export default BoidSystem;
