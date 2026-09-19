const OccupancyState = Object.freeze({
    UNKNOWN: "UNKNOWN",
    FREE: "FREE",
    OCCUPIED: "OCCUPIED"
});

/**
 * Vehicle-relative, foveated semantic occupancy map.
 * Resolution is selected by radial distance on every update, preventing gaps or
 * overlap between the nested distance bands in the local map window.
 */
class AdaptiveSemanticMap {
    constructor(options = {}) {
        this.maxRange = options.maxRange ?? 180;
        this.bands = options.bands ?? [
            { maxRange: 60, resolution: 5 },
            { maxRange: 120, resolution: 10 },
            { maxRange: 180, resolution: 20 }
        ];
        this.cells = new Map();
    }

    update(semanticPoints, observations) {
        this.cells.clear();

        for (const point of semanticPoints) {
            const cell = this.#getOrCreateCell(point.x, point.y);
            if (!cell) continue;
            cell.heightSum += point.z;
            cell.pointCount += 1;
            this.#addSemanticVote(cell, point.class, point.confidence);
            if (point.class === SemanticClass.DRIVABLE_TERRAIN) {
                cell.freeVotes += 1;
            } else if (point.class !== SemanticClass.UNKNOWN) {
                cell.occupiedVotes += 3;
            }
        }

        for (const observation of observations) {
            this.#markFreeRay(observation);
        }
    }

    getCells() {
        return Array.from(this.cells.values(), cell => {
            const semantic = this.#dominantSemantic(cell);
            return {
                x: cell.cellX * cell.resolution,
                y: cell.cellY * cell.resolution,
                size: cell.resolution,
                height: cell.pointCount === 0 ? 0 : cell.heightSum / cell.pointCount,
                pointCount: cell.pointCount,
                class: semantic.class,
                confidence: semantic.confidence,
                occupancy: this.#occupancy(cell),
                resolution: cell.resolution
            };
        });
    }

    getLaneRisk(laneCenter, vehiclePose, road, lookAhead) {
        const target = CoordinateFrames.worldToVehicle(
            { x: laneCenter, y: vehiclePose.y, z: 0 },
            vehiclePose
        );
        let nearest = Infinity;
        const laneHalfWidth = road.width / road.laneCount / 2;
        for (const cell of this.getCells()) {
            if (cell.occupancy !== OccupancyState.OCCUPIED) continue;
            const centreX = cell.x + cell.size / 2;
            const centreY = cell.y + cell.size / 2;
            if (centreY > 0 && centreY < lookAhead && Math.abs(centreX - target.x) < laneHalfWidth) {
                nearest = Math.min(nearest, centreY);
            }
        }
        return nearest;
    }

    #markFreeRay(observation) {
        const maxFreeRange = observation.hit
            ? Math.max(0, observation.range - 2)
            : this.maxRange;
        if (maxFreeRange < 2) return;

        const directionX = observation.x / observation.range;
        const directionY = observation.y / observation.range;
        for (let distance = 2; distance < maxFreeRange;) {
            const cell = this.#getOrCreateCell(directionX * distance, directionY * distance);
            if (!cell) break;
            cell.freeVotes += 1;
            distance += cell.resolution;
        }
    }

    #getOrCreateCell(x, y) {
        const range = Math.hypot(x, y);
        const band = this.bands.find(candidate => range <= candidate.maxRange);
        if (!band) return null;
        const cellX = Math.floor(x / band.resolution);
        const cellY = Math.floor(y / band.resolution);
        const key = `${band.resolution}:${cellX},${cellY}`;
        let cell = this.cells.get(key);
        if (!cell) {
            cell = {
                cellX,
                cellY,
                resolution: band.resolution,
                heightSum: 0,
                pointCount: 0,
                semanticVotes: {},
                freeVotes: 0,
                occupiedVotes: 0
            };
            this.cells.set(key, cell);
        }
        return cell;
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

    #occupancy(cell) {
        if (cell.occupiedVotes > 0) return OccupancyState.OCCUPIED;
        if (cell.freeVotes > 0) return OccupancyState.FREE;
        return OccupancyState.UNKNOWN;
    }
}

class AdaptiveMapVisualizer {
    static draw(ctx, adaptiveMap, tracks, metrics) {
        const { width, height } = ctx.canvas;
        const scale = Math.min(width, height) / (adaptiveMap.maxRange * 2.2);
        const centreX = width / 2;
        const centreY = height / 2;
        const cells = adaptiveMap.getCells();

        ctx.clearRect(0, 0, width, height);
        this.#drawRangeGrid(ctx, adaptiveMap, scale, centreX, centreY);
        for (const cell of cells) {
            const color = this.#cellColor(cell);
            ctx.fillStyle = color;
            ctx.fillRect(
                centreX + cell.x * scale,
                centreY - (cell.y + cell.size) * scale,
                cell.size * scale,
                cell.size * scale
            );
        }

        for (const track of tracks) {
            const point = CoordinateFrames.worldToVehicle(track.position, metrics.vehiclePose);
            if (point.y < -10 || point.y > adaptiveMap.maxRange) continue;
            ctx.strokeStyle = "#f15b9a";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(centreX + point.x * scale, centreY - point.y * scale, 6, 0, Math.PI * 2);
            ctx.stroke();
        }

        ctx.fillStyle = "#ffcf4a";
        ctx.fillRect(centreX - 4, centreY - 6, 8, 12);
        ctx.font = "12px Arial";
        ctx.fillStyle = "#e8f1f5";
        ctx.fillText("Adaptive semantic occupancy", 10, 20);
        ctx.fillText(`${cells.length} cells · ${tracks.length} tracks`, 10, 38);
        ctx.fillText(`${metrics.latencyMs.toFixed(1)} ms update`, 10, 56);
        ctx.fillStyle = "#4edb7a";
        ctx.fillText("free", 10, 75);
        ctx.fillStyle = "#ff9f43";
        ctx.fillText("static", 48, 75);
        ctx.fillStyle = "#f15b9a";
        ctx.fillText("dynamic", 102, 75);
    }

    static #cellColor(cell) {
        if (cell.occupancy === OccupancyState.UNKNOWN) return "rgba(110, 130, 145, 0.18)";
        if (cell.occupancy === OccupancyState.FREE) return "rgba(78, 219, 122, 0.42)";
        switch (cell.class) {
            case SemanticClass.DYNAMIC_OBJECT: return "rgba(241, 91, 154, 0.92)";
            case SemanticClass.STATIC_OBSTACLE: return "rgba(255, 159, 67, 0.92)";
            default: return "rgba(241, 91, 154, 0.78)";
        }
    }

    static #drawRangeGrid(ctx, adaptiveMap, scale, centreX, centreY) {
        ctx.strokeStyle = "rgba(136, 174, 190, 0.35)";
        for (const band of adaptiveMap.bands) {
            ctx.beginPath();
            ctx.arc(centreX, centreY, band.maxRange * scale, 0, Math.PI * 2);
            ctx.stroke();
        }
        ctx.beginPath();
        ctx.moveTo(centreX, 0);
        ctx.lineTo(centreX, ctx.canvas.height);
        ctx.moveTo(0, centreY);
        ctx.lineTo(ctx.canvas.width, centreY);
        ctx.stroke();
    }
}
