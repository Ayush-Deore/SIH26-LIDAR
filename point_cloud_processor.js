const SemanticClass = Object.freeze({
    DRIVABLE_TERRAIN: "DRIVABLE_TERRAIN",
    STATIC_OBSTACLE: "STATIC_OBSTACLE",
    DYNAMIC_OBJECT: "DYNAMIC_OBJECT",
    UNKNOWN: "UNKNOWN"
});

/**
 * Converts raw LiDAR returns into a clean semantic point cloud.
 *
 * This first implementation uses simulator-provided return provenance as ground
 * truth. It is intentionally isolated here so a learned classifier can replace
 * `#classify` later without changing the mapper or visualizer.
 */
class PointCloudProcessor {
    constructor(options = {}) {
        this.minRange = options.minRange ?? 2;
        this.maxRange = options.maxRange ?? 180;
    }

    process(rawPoints) {
        return rawPoints
            .filter(point => this.#isValid(point))
            .map(point => {
                const semantic = this.#classify(point);
                return {
                    x: point.x,
                    y: point.y,
                    z: point.z,
                    range: point.range,
                    class: semantic.class,
                    confidence: semantic.confidence
                };
            });
    }

    #isValid(point) {
        return Number.isFinite(point.x) &&
            Number.isFinite(point.y) &&
            Number.isFinite(point.z) &&
            Number.isFinite(point.range) &&
            point.range >= this.minRange &&
            point.range <= this.maxRange;
    }

    #classify(point) {
        switch (point.simulatorLabel) {
            case SemanticClass.DRIVABLE_TERRAIN:
                return { class: SemanticClass.DRIVABLE_TERRAIN, confidence: 0.99 };
            case SemanticClass.STATIC_OBSTACLE:
                return { class: SemanticClass.STATIC_OBSTACLE, confidence: 0.98 };
            case SemanticClass.DYNAMIC_OBJECT:
                return { class: SemanticClass.DYNAMIC_OBJECT, confidence: 0.99 };
            default:
                return { class: SemanticClass.UNKNOWN, confidence: 0.1 };
        }
    }
}
