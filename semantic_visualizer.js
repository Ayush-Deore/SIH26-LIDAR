class SemanticPointCloudVisualizer {
    static draw(ctx, points, maxRange) {
        const { width, height } = ctx.canvas;
        const scale = Math.min(width, height) / (maxRange * 2.2);
        const centreX = width / 2;
        const centreY = height / 2;

        ctx.clearRect(0, 0, width, height);
        this.#drawRangeGrid(ctx, maxRange, scale, centreX, centreY);

        for (const point of points) {
            ctx.fillStyle = this.#colorFor(point.class);
            ctx.beginPath();
            ctx.arc(
                centreX + point.x * scale,
                centreY - point.y * scale,
                point.class === SemanticClass.DYNAMIC_OBJECT ? 2.5 : 2,
                0,
                Math.PI * 2
            );
            ctx.fill();
        }

        ctx.fillStyle = "#ffcf4a";
        ctx.fillRect(centreX - 4, centreY - 6, 8, 12);
        ctx.font = "12px Arial";
        ctx.fillStyle = "#e8f1f5";
        ctx.fillText("Semantic point cloud", 10, 20);
        ctx.fillStyle = "#4edb7a";
        ctx.fillText("● terrain", 10, 39);
        ctx.fillStyle = "#ff9f43";
        ctx.fillText("● static", 10, 57);
        ctx.fillStyle = "#f15b9a";
        ctx.fillText("● dynamic", 10, 75);
    }

    static #colorFor(semanticClass) {
        switch (semanticClass) {
            case SemanticClass.DRIVABLE_TERRAIN: return "#4edb7a";
            case SemanticClass.STATIC_OBSTACLE: return "#ff9f43";
            case SemanticClass.DYNAMIC_OBJECT: return "#f15b9a";
            default: return "#a4b0be";
        }
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
}
