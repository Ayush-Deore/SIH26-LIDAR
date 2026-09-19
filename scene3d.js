/**
 * Software 3D renderer for the existing 2D driving simulation.
 * It projects the same world geometry through a camera that follows behind the
 * ego vehicle, so simulation, LiDAR, and collision logic keep one source of truth.
 */
class Scene3DRenderer {
    static draw(ctx, road, egoCar, traffic) {
        const camera = {
            car: egoCar,
            width: ctx.canvas.width,
            distance: 105,
            height: 30,
            horizon: ctx.canvas.height * 0.34,
            focalLength: ctx.canvas.height * 0.58
        };

        this.#drawSky(ctx);
        this.#drawRoad(ctx, road, camera);

        const vehicles = [...traffic, egoCar]
            .map(car => ({ car, local: CoordinateFrames.worldToVehicle(car, egoCar) }))
            .filter(entry => entry.local.y + camera.distance > 5 && entry.local.y < 650)
            .sort((a, b) => b.local.y - a.local.y);

        for (const { car } of vehicles) {
            this.#drawVehicle(ctx, car, car === egoCar ? "#2f8cff" : "#ed4d4d", camera);
        }
    }

    static #drawSky(ctx) {
        const gradient = ctx.createLinearGradient(0, 0, 0, ctx.canvas.height);
        gradient.addColorStop(0, "#72bde8");
        gradient.addColorStop(0.34, "#d9eff8");
        gradient.addColorStop(0.35, "#38434a");
        gradient.addColorStop(1, "#1a2024");
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, ctx.canvas.width, ctx.canvas.height);
    }

    static #drawRoad(ctx, road, camera) {
        // Keep the near road edge in front of the follow camera.
        const nearY = camera.car.y + 60;
        const farY = camera.car.y - 700;
        this.#fillWorldPolygon(ctx, [
            { x: road.left, y: nearY, z: 0 },
            { x: road.right, y: nearY, z: 0 },
            { x: road.right, y: farY, z: 0 },
            { x: road.left, y: farY, z: 0 }
        ], camera, "#4c565c");

        for (let lane = 1; lane < road.laneCount; lane++) {
            const x = road.left + (road.width * lane) / road.laneCount;
            for (let y = farY; y < nearY; y += 54) {
                this.#fillWorldPolygon(ctx, [
                    { x: x - 2, y, z: 0.2 },
                    { x: x + 2, y, z: 0.2 },
                    { x: x + 2, y: y + 25, z: 0.2 },
                    { x: x - 2, y: y + 25, z: 0.2 }
                ], camera, "#e9edf0");
            }
        }

        for (const x of [road.left, road.right]) {
            this.#fillWorldPolygon(ctx, [
                { x: x - 3, y: nearY, z: 0.3 },
                { x: x + 3, y: nearY, z: 0.3 },
                { x: x + 3, y: farY, z: 0.3 },
                { x: x - 3, y: farY, z: 0.3 }
            ], camera, "#f7d34e");
        }
    }

    static #drawVehicle(ctx, car, color, camera) {
        const base = car.polygon.map(point => ({ x: point.x, y: point.y, z: 0 }));
        const bodyTop = car.polygon.map(point => ({ x: point.x, y: point.y, z: 7 }));
        const sideColor = this.#shade(color, -42);

        for (let i = 0; i < base.length; i++) {
            this.#fillWorldPolygon(ctx, [
                base[i],
                base[(i + 1) % base.length],
                bodyTop[(i + 1) % bodyTop.length],
                bodyTop[i]
            ], camera, sideColor);
        }
        this.#fillWorldPolygon(ctx, bodyTop, camera, color);

        const cabinBase = this.#carRectangle(car, 0.34, -0.23, 0.34, 0.24, 7);
        const cabinTop = this.#carRectangle(car, 0.25, -0.16, 0.25, 0.17, 16);
        const glassColor = car === camera.car ? "#9ed8ff" : "#ffd1c8";
        for (let i = 0; i < cabinBase.length; i++) {
            this.#fillWorldPolygon(ctx, [
                cabinBase[i],
                cabinBase[(i + 1) % cabinBase.length],
                cabinTop[(i + 1) % cabinTop.length],
                cabinTop[i]
            ], camera, glassColor);
        }
        this.#fillWorldPolygon(ctx, cabinTop, camera, this.#shade(glassColor, 12));

        //this.#drawWheels(ctx, car, camera);
        //this.#drawLights(ctx, car, camera);
    }

    static #drawWheels(ctx, car, camera) {
        for (const lateral of [-0.53, 0.53]) {
            for (const forward of [-0.28, 0.28]) {
                const wheel = this.#carRectangle(car, 0.12, forward - 0.1, 0.12, forward + 0.1, 4, lateral);
                this.#fillWorldPolygon(ctx, wheel, camera, "#11161a");
            }
        }
    }

    static #drawLights(ctx, car, camera) {
        const rearLights = [
            this.#carRectangle(car, 0.11, -0.5, 0.11, -0.42, 8, -0.25),
            this.#carRectangle(car, 0.11, -0.5, 0.11, -0.42, 8, 0.25)
        ];
        for (const light of rearLights) this.#fillWorldPolygon(ctx, light, camera, "#ff5252");
    }

    static #carRectangle(car, halfWidth, rear, front, forwardEnd, z, lateralOffset = 0) {
        const forwardStart = rear;
        const forwardFinish = forwardEnd;
        return [
            this.#carPoint(car, lateralOffset - halfWidth, forwardStart, z),
            this.#carPoint(car, lateralOffset + halfWidth, forwardStart, z),
            this.#carPoint(car, lateralOffset + front, forwardFinish, z),
            this.#carPoint(car, lateralOffset - front, forwardFinish, z)
        ];
    }

    static #carPoint(car, lateralFraction, forwardFraction, z) {
        const lateral = lateralFraction * car.width;
        const forward = forwardFraction * car.height;
        return {
            x: car.x + lateral * Math.cos(car.angle) - forward * Math.sin(car.angle),
            y: car.y - lateral * Math.sin(car.angle) - forward * Math.cos(car.angle),
            z
        };
    }

    static #fillWorldPolygon(ctx, worldPoints, camera, color) {
        const projected = worldPoints.map(point => this.#project(point, camera));
        if (projected.some(point => point === null)) return;

        ctx.beginPath();
        ctx.moveTo(projected[0].x, projected[0].y);
        for (let i = 1; i < projected.length; i++) ctx.lineTo(projected[i].x, projected[i].y);
        ctx.closePath();
        ctx.fillStyle = color;
        ctx.fill();
    }

    static #project(point, camera) {
        const local = CoordinateFrames.worldToVehicle(point, camera.car);
        const depth = local.y + camera.distance;
        if (depth <= 5) return null;
        return {
            x: camera.width / 2 + (local.x * camera.focalLength) / depth,
            y: camera.horizon + ((camera.height - (point.z ?? 0)) * camera.focalLength) / depth
        };
    }

    static #shade(hexColor, amount) {
        const value = parseInt(hexColor.slice(1), 16);
        const channel = shift => Math.max(0, Math.min(255, (value >> shift & 255) + amount));
        return `rgb(${channel(16)}, ${channel(8)}, ${channel(0)})`;
    }
}
