/** Associates clusters of dynamic semantic returns across frames. */
class DynamicObjectTracker {
    constructor(options = {}) {
        this.clusterRadius = options.clusterRadius ?? 28;
        this.matchDistance = options.matchDistance ?? 45;
        this.maxAgeMs = options.maxAgeMs ?? 700;
        this.tracks = [];
        this.nextId = 1;
    }

    update(semanticPoints, vehiclePose, timestampMs) {
        const clusters = this.#clusterDynamicPoints(semanticPoints)
            .map(cluster => CoordinateFrames.vehicleToWorld(cluster, vehiclePose));
        const unmatchedTracks = new Set(this.tracks);

        for (const position of clusters) {
            let nearest = null;
            let nearestDistance = this.matchDistance;
            for (const track of unmatchedTracks) {
                const distance = Math.hypot(track.position.x - position.x, track.position.y - position.y);
                if (distance < nearestDistance) {
                    nearest = track;
                    nearestDistance = distance;
                }
            }

            if (nearest) {
                const deltaSeconds = Math.max((timestampMs - nearest.lastSeenMs) / 1000, 0.001);
                nearest.velocity = {
                    x: (position.x - nearest.position.x) / deltaSeconds,
                    y: (position.y - nearest.position.y) / deltaSeconds
                };
                nearest.position = position;
                nearest.lastSeenMs = timestampMs;
                unmatchedTracks.delete(nearest);
            } else {
                this.tracks.push({
                    id: this.nextId++,
                    position,
                    velocity: { x: 0, y: 0 },
                    lastSeenMs: timestampMs
                });
            }
        }

        this.tracks = this.tracks.filter(track => timestampMs - track.lastSeenMs <= this.maxAgeMs);
        return this.tracks;
    }

    #clusterDynamicPoints(points) {
        const clusters = [];
        for (const point of points) {
            if (point.class !== SemanticClass.DYNAMIC_OBJECT) continue;
            let cluster = clusters.find(candidate =>
                Math.hypot(candidate.x - point.x, candidate.y - point.y) < this.clusterRadius
            );
            if (!cluster) {
                cluster = { x: point.x, y: point.y, z: point.z, count: 0 };
                clusters.push(cluster);
            }
            cluster.count += 1;
            cluster.x += (point.x - cluster.x) / cluster.count;
            cluster.y += (point.y - cluster.y) / cluster.count;
            cluster.z += (point.z - cluster.z) / cluster.count;
        }
        return clusters;
    }
}
