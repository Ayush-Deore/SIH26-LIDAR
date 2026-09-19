/**
 * Uniform, rolling 2.5D elevation map.
 *
 * Input points use the LiDAR frame: +x right, +y forward, and z elevation.
 * Cells are stored in the world frame and displayed in a vehicle-centred window.
 * `cellSize` is expressed in the simulator's world units because this 2D project
 * does not yet define a pixels-to-metres calibration.
 */
class UniformElevationMap {
    constructor(options = {}) {
        this.cellSize = options.cellSize ?? 10;
        this.maxRange = options.maxRange ?? 180;
        this.cells = new Map();
        this.frame = 0;
    }

    update(points, vehiclePose) {
        this.frame += 1;

        for (const point of points) {
            if (point.range > this.maxRange) continue;

            const worldPoint = CoordinateFrames.vehicleToWorld(point, vehiclePose);
            const cellX = Math.floor(worldPoint.x / this.cellSize);
            const cellY = Math.floor(worldPoint.y / this.cellSize);
            const key = `${cellX},${cellY}`;
            const cell = this.cells.get(key) ?? {
                cellX,
                cellY,
                heightSum: 0,
                pointCount: 0,
                semanticVotes: {}
            };

            cell.heightSum += point.z;
            cell.pointCount += 1;
            this.#addSemanticVote(cell, point.class, point.confidence);
            cell.lastSeenFrame = this.frame;
            this.cells.set(key, cell);
        }

        this.#removeDistantCells(vehiclePose);
    }

    getCells() {
        return Array.from(this.cells.values(), cell => {
            const semantic = this.#dominantSemantic(cell);
            return {
                worldX: cell.cellX * this.cellSize,
                worldY: cell.cellY * this.cellSize,
                size: this.cellSize,
                height: cell.heightSum / cell.pointCount,
                pointCount: cell.pointCount,
                class: semantic.class,
                confidence: semantic.confidence,
                lastSeenFrame: cell.lastSeenFrame
            };
        });
    }

    #addSemanticVote(cell, semanticClass, confidence) {
        const className = semanticClass ?? SemanticClass.UNKNOWN;
        const vote = cell.semanticVotes[className] ?? { count: 0, confidenceSum: 0 };
        vote.count += 1;
        vote.confidenceSum += Number.isFinite(confidence) ? confidence : 0;
        cell.semanticVotes[className] = vote;
    }

    #dominantSemantic(cell) {
        let winner = { class: SemanticClass.UNKNOWN, count: 0, confidenceSum: 0 };
        for (const [className, vote] of Object.entries(cell.semanticVotes)) {
            if (vote.count > winner.count ||
                (vote.count === winner.count && vote.confidenceSum > winner.confidenceSum)) {
                winner = { class: className, ...vote };
            }
        }
        return {
            class: winner.class,
            confidence: winner.count === 0 ? 0 : winner.confidenceSum / winner.count
        };
    }

    #removeDistantCells(vehiclePose) {
        for (const [key, cell] of this.cells) {
            const centreX = (cell.cellX + 0.5) * this.cellSize;
            const centreY = (cell.cellY + 0.5) * this.cellSize;
            if (Math.hypot(centreX - vehiclePose.x, centreY - vehiclePose.y) > this.maxRange) {
                this.cells.delete(key);
            }
        }
    }
}

class ElevationMapVisualizer {
    static draw(ctx, elevationMap, vehiclePose) {
        const { width, height } = ctx.canvas;
        const scale = Math.min(width, height) / (elevationMap.maxRange * 2.2);
        const centreX = width / 2;
        const centreY = height / 2;
        const cells = elevationMap.getCells();

        ctx.clearRect(0, 0, width, height);
        this.#drawRangeGrid(ctx, elevationMap.maxRange, scale, centreX, centreY);

        for (const cell of cells) {
            ctx.fillStyle = this.#cellColor(cell);
            ctx.fillRect(
                centreX + (cell.worldX - vehiclePose.x) * scale,
                centreY + (cell.worldY - vehiclePose.y) * scale,
                cell.size * scale,
                cell.size * scale
            );
        }

        this.#drawVehicle(ctx, centreX, centreY, vehiclePose.angle);
        ctx.fillStyle = "#e8f1f5";
        ctx.font = "12px Arial";
        ctx.fillText("Semantic 2.5D rolling map", 10, 20);
        ctx.fillText(`${cells.length} world cells · ${elevationMap.cellSize} unit cells`, 10, 38);
        ctx.fillStyle = "#4edb7a";
        ctx.fillText("terrain", 10, 57);
        ctx.fillStyle = "#ff9f43";
        ctx.fillText("static", 65, 57);
        ctx.fillStyle = "#f15b9a";
        ctx.fillText("dynamic", 115, 57);
        ctx.fillStyle = "#e8f1f5";
        ctx.fillText("world-aligned · mean z + dominant class", 10, height - 12);
    }

    static #cellColor(cell) {
        switch (cell.class) {
            case SemanticClass.DRIVABLE_TERRAIN: return "#4edb7a";
            case SemanticClass.STATIC_OBSTACLE: return "#ff9f43";
            case SemanticClass.DYNAMIC_OBJECT: return "#f15b9a";
            default: return this.#heightColor(cell.height);
        }
    }

    static #heightColor(height) {
        // Centre zero elevation on a neutral cyan. Higher and lower terrain are
        // intentionally visible once a 3D LiDAR source becomes available.
        const normalized = Math.max(-1, Math.min(1, height / 20));
        const hue = 195 - normalized * 120;
        const lightness = 48 + normalized * 14;
        return `hsl(${hue}, 75%, ${lightness}%)`;
    }

    static #drawRangeGrid(ctx, maxRange, scale, centreX, centreY) {
        ctx.strokeStyle = "rgba(136, 174, 190, 0.35)";
        ctx.lineWidth = 1;
        for (let distance = 50; distance <= maxRange; distance += 50) {
            ctx.beginPath();
            ctx.arc(centreX, centreY, distance * scale, 0, Math.PI * 2);
            ctx.stroke();
        }
        ctx.beginPath();
        ctx.moveTo(centreX, 0);
        ctx.lineTo(centreX, ctx.canvas.height);
        ctx.moveTo(0, centreY);
        ctx.lineTo(ctx.canvas.width, centreY);
        ctx.stroke();
    }

    static #drawVehicle(ctx, centreX, centreY, angle) {
        ctx.save();
        ctx.translate(centreX, centreY);
        ctx.rotate(-angle);
        ctx.fillStyle = "#ffcf4a";
        ctx.beginPath();
        ctx.moveTo(0, -8);
        ctx.lineTo(6, 7);
        ctx.lineTo(-6, 7);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    }
}
