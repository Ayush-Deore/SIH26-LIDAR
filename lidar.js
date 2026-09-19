/**
 * Planar LiDAR simulator for the existing top-down driving world.
 *
 * Coordinate frames:
 * - World: the canvas coordinate system used by Car and Road.
 * - LiDAR: origin at the vehicle centre; +x points right and +y points forward.
 *
 * The source simulation has no vertical geometry, therefore every measured point
 * has z = 0. This module intentionally produces a 2D point cloud for Milestone 1.
 */
class Lidar {
    constructor(car, options = {}) {
        this.car = car;
        this.rayCount = options.rayCount ?? 91;
        this.maxRange = options.maxRange ?? 180;
        this.fieldOfView = options.fieldOfView ?? Math.PI * 2;
        this.points = [];
        this.rays = [];
        this.observations = [];
    }

    update(roadBorders, traffic) {
        this.points = [];
        this.rays = [];
        this.observations = [];

        for (let i = 0; i < this.rayCount; i++) {
            const ray = this.#castRay(i);
            this.rays.push(ray);
            const hit = this.#closestHit(ray, roadBorders, traffic);
            if (hit) {
                const point = this.#toVehiclePoint(hit);
                this.points.push(point);
                this.observations.push({ ...point, hit: true });
            } else {
                const endpoint = this.#toVehiclePoint({
                    x: ray[1].x,
                    y: ray[1].y,
                    z: 0,
                    simulatorLabel: SemanticClass.UNKNOWN
                });
                this.observations.push({ ...endpoint, hit: false });
            }
        }

        this.#addGroundReturns(roadBorders);
    }

    #castRay(index) {
        const fraction = this.rayCount === 1 ? 0.5 : index / (this.rayCount - 1);
        const rayAngle = lerp(
            this.fieldOfView / 2,
            -this.fieldOfView / 2,
            fraction
        ) + this.car.angle;
        const start = { x: this.car.x, y: this.car.y };
        const end = {
            x: this.car.x - Math.sin(rayAngle) * this.maxRange,
            y: this.car.y - Math.cos(rayAngle) * this.maxRange
        };
        return [start, end];
    }

    #closestHit(ray, roadBorders, traffic) {
        const hits = [];

        for (const border of roadBorders) {
            const hit = getIntersection(ray[0], ray[1], border[0], border[1]);
            if (hit) hits.push({ ...hit, simulatorLabel: SemanticClass.STATIC_OBSTACLE });
        }

        for (const vehicle of traffic) {
            const polygon = vehicle.polygon;
            for (let i = 0; i < polygon.length; i++) {
                const hit = getIntersection(
                    ray[0],
                    ray[1],
                    polygon[i],
                    polygon[(i + 1) % polygon.length]
                );
                if (hit) hits.push({ ...hit, simulatorLabel: SemanticClass.DYNAMIC_OBJECT });
            }
        }

        if (hits.length === 0) return null;
        return hits.reduce((closest, hit) =>
            hit.offset < closest.offset ? hit : closest
        );
    }

    #addGroundReturns(roadBorders) {
        // Downward-facing channels are approximated against the simulator's flat
        // road plane. This is the 2D simulator equivalent of ground returns.
        const leftRoadEdge = Math.min(roadBorders[0][0].x, roadBorders[0][1].x);
        const rightRoadEdge = Math.max(roadBorders[1][0].x, roadBorders[1][1].x);
        const ranges = [25, 65, 110, 155];

        for (let i = 0; i < this.rays.length; i += 4) {
            const ray = this.rays[i];
            const directionX = (ray[1].x - ray[0].x) / this.maxRange;
            const directionY = (ray[1].y - ray[0].y) / this.maxRange;
            for (const range of ranges) {
                const worldPoint = {
                    x: this.car.x + directionX * range,
                    y: this.car.y + directionY * range,
                    z: 0,
                    simulatorLabel: SemanticClass.DRIVABLE_TERRAIN
                };
                if (worldPoint.x >= leftRoadEdge && worldPoint.x <= rightRoadEdge) {
                    this.points.push(this.#toVehiclePoint(worldPoint));
                }
            }
        }
    }

    #toVehiclePoint(worldPoint) {
        const point = CoordinateFrames.worldToVehicle(worldPoint, this.car);

        return {
            x: point.x,
            y: point.y,
            z: 0,
            range: Math.hypot(point.x, point.y),
            simulatorLabel: worldPoint.simulatorLabel
        };
    }
}

class LidarVisualizer {
    static draw(ctx, lidar) {
        const { width, height } = ctx.canvas;
        const scale = Math.min(width, height) / (lidar.maxRange * 2.2);
        const centreX = width / 2;
        const centreY = height / 2;

        ctx.clearRect(0, 0, width, height);
        this.#drawGrid(ctx, lidar.maxRange, scale, centreX, centreY);

        ctx.fillStyle = "#58d8ff";
        for (const point of lidar.points) {
            ctx.beginPath();
            ctx.arc(
                centreX + point.x * scale,
                centreY - point.y * scale,
                2,
                0,
                Math.PI * 2
            );
            ctx.fill();
        }

        ctx.fillStyle = "#ffcf4a";
        ctx.fillRect(centreX - 4, centreY - 6, 8, 12);
        ctx.fillStyle = "#e8f1f5";
        ctx.font = "12px Arial";
        ctx.fillText("Planar LiDAR", 10, 20);
        ctx.fillText(`${lidar.points.length} returns`, 10, 38);
        ctx.fillText("+Y forward", 10, height - 12);
    }

    static #drawGrid(ctx, maxRange, scale, centreX, centreY) {
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
}
