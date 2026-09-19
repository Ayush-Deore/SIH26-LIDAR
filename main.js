
const carCanvas = document.getElementById("carCanvas");
carCanvas.width = 500;
const lidarCanvas = document.getElementById("lidarCanvas");
lidarCanvas.width = 220;
const semanticCanvas = document.getElementById("semanticCanvas");
semanticCanvas.width = 220;
const mapCanvas = document.getElementById("mapCanvas");
mapCanvas.width = 220;
const networkCanvas = document.getElementById("networkCanvas");
networkCanvas.width = 300;

const carImage = new Image();
carImage.src = "car_topview.svg"; // Path to your SVG image

const carCtx = carCanvas.getContext("2d");
const lidarCtx = lidarCanvas.getContext("2d");
const semanticCtx = semanticCanvas.getContext("2d");
const mapCtx = mapCanvas.getContext("2d");
const networkCtx = networkCanvas.getContext("2d");

// Keep simulation dimensions independent from the wider 3D camera viewport.
const road = new Road(carCanvas.width / 2, 180);



const N = 1;
const cars = generateCars(N);

let bestCar = cars[0];
const brainStorageKey = "adaptive-lidar-best-brain-v1";
const storedBrain = localStorage.getItem(brainStorageKey);
const initialBrain = storedBrain ? JSON.parse(storedBrain) : savedBrain;
for (let i = 0; i < cars.length; i++) {
    cars[i].brain = JSON.parse(JSON.stringify(initialBrain));
    if (i !== 0 && storedBrain) {
        NeuralNetwork.mutate(cars[i].brain, 0.03);
    }
}
const lidar = new Lidar(bestCar);
const pointCloudProcessor = new PointCloudProcessor({ maxRange: lidar.maxRange });
const adaptiveMap = new AdaptiveSemanticMap({ maxRange: lidar.maxRange });
const dynamicTracker = new DynamicObjectTracker();
let perceptionLatencyMs = 0;
const elevationMap = new UniformElevationMap({
    cellSize: 10,
    maxRange: lidar.maxRange
});
const traffic = [
    new Car(road.getLaneCenter(1), -100, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(0), -300, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(2), -300, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(0), -500, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(1), -500, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(1), -700, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(2), -700, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(0), -900, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(2), -900, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(1), -1100, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(2), -1100, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(0), -1300, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(2), -1300, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(1), -1500, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(0), -1700, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(2), -1900, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(1), -2100, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(2), -2100, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(0), -2300, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(1), -2500, 30, 50, "DUMMY", 2),
    new Car(road.getLaneCenter(1), -2500, 30, 50, "DUMMY", 2),



    
];

animate();

function save() {
    localStorage.setItem(brainStorageKey,
        JSON.stringify(bestCar.brain));
    
    
}

function discard() {
    localStorage.removeItem(brainStorageKey);
}

function generateCars(N) {
    const cars = [];
    for (let i = 1; i <= N; i++){
        cars.push(new Car(road.getLaneCenter(1), 100, 30, 50, "AI"));
    }
    return cars;
}

function animate(time) {
    
    for (let i = 0; i < traffic.length; i++){
        traffic[i].update(road.borders,[]);
    }

    for (let i = 0; i < cars.length; i++) {
        cars[i].update(road.borders, traffic);
    }

     bestCar = cars.find(
        c => c.y == Math.min(
            ...cars.map(c => c.y)
        ));

    carCanvas.height = window.innerHeight;
    lidarCanvas.height = window.innerHeight;
    semanticCanvas.height = window.innerHeight;
    mapCanvas.height = window.innerHeight;
    networkCanvas.height = window.innerHeight;

    lidar.update(road.borders, traffic);
    const perceptionStart = performance.now();
    const semanticPoints = pointCloudProcessor.process(lidar.points);
    elevationMap.update(semanticPoints, bestCar);
    adaptiveMap.update(semanticPoints, lidar.observations);
    const tracks = dynamicTracker.update(semanticPoints, bestCar, performance.now());
    perceptionLatencyMs = performance.now() - perceptionStart;

    Scene3DRenderer.draw(carCtx, road, bestCar, traffic);

    LidarVisualizer.draw(lidarCtx, lidar);
    SemanticPointCloudVisualizer.draw(semanticCtx, semanticPoints, lidar.maxRange);
    AdaptiveMapVisualizer.draw(mapCtx, adaptiveMap, tracks, {
        vehiclePose: bestCar,
        latencyMs: perceptionLatencyMs
    });

    networkCtx.lineDashOffset = -time / 50;
    Visualizer.drawNetwork(networkCtx, bestCar.brain);
    requestAnimationFrame(animate);
};
