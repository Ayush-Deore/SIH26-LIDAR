/**
 * Coordinate transforms used by perception and mapping.
 *
 * World frame: the simulation canvas; +x is right and +y is down.
 * Vehicle/LiDAR frame: origin is the vehicle centre; +x is vehicle-right and
 * +y is vehicle-forward. The vehicle's `angle` is measured in world radians.
 */
class CoordinateFrames {
    static worldToVehicle(point, pose) {
        const dx = point.x - pose.x;
        const dy = point.y - pose.y;
        const cos = Math.cos(pose.angle);
        const sin = Math.sin(pose.angle);

        return {
            x: dx * cos - dy * sin,
            y: -(dx * sin + dy * cos),
            z: point.z ?? 0
        };
    }

    static vehicleToWorld(point, pose) {
        const cos = Math.cos(pose.angle);
        const sin = Math.sin(pose.angle);

        return {
            x: pose.x + point.x * cos - point.y * sin,
            y: pose.y - point.x * sin - point.y * cos,
            z: point.z ?? 0
        };
    }
}
