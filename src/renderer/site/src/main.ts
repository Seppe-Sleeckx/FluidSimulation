import { WebGPUContext } from "./WebgpuContext";
import { Camera } from "./Camera";
import { InputController } from "./ImputController";
import { ParticleRenderer } from "./ParticleRenderer";
import { BoundsRenderer } from "./BoundsRenderer.ts"
import fluidSolverModule from "./fluid_solver.js";
import * as SolverConfigUtils from "./fluid_solver_utils.ts"
import GUI from "lil-gui";

const canvas = document.getElementById("simulation") as HTMLCanvasElement;

canvas.width = canvas.clientWidth * window.devicePixelRatio;
canvas.height = canvas.clientHeight * window.devicePixelRatio;

const gpu = new WebGPUContext(canvas);
await gpu.initialize(); //create and initialize context



const camera = new Camera();
const inputController = new InputController(canvas, camera); //create camera and bind it with our inputcontroller

//===============
//Flip solver
//===============
const Module = await fluidSolverModule(); //get WASM instance

//create function "wrappers", ptr's == number
const create = Module.cwrap('Create', 'number', ['number']);
const destroy = Module.cwrap('Destroy', null, ['number']);
const initialize = Module.cwrap('Initialize', null, ['number']);
const simulate = Module.cwrap('Simulate', null, ['number', 'number']);
const getParticleCount = Module.cwrap('GetParticleCount', 'number', ['number']);
const getParticlePositions = Module.cwrap('GetParticlePositions', 'number', ['number']);
const getGridX = Module.cwrap('GetGridXDimension', 'number', ['number']);
const getGridY = Module.cwrap('GetGridYDimension', 'number', ['number']);
const getGridZ = Module.cwrap('GetGridZDimension', 'number', ['number']);
const writeMeasurementsToFile = Module.cwrap('WriteMeasurementsToFile', null, ['number']);


const config: SolverConfigUtils.SolverConfig = {
    gridX: 50,
    gridY: 20,
    gridZ: 20,
    numParticles: 5000,
    particleRadius: 0.5,
    alphaPic: 0.05,
    useAdaptiveMixing: false,
}

const depthTexture = gpu.device.createTexture({
    size: [canvas.width, canvas.height],
    format: "depth24plus", //24 bit depth buffer
    usage: GPUTextureUsage.RENDER_ATTACHMENT //allow this texture to be used in a render pass
});


const fixedDt = 1 / 60;
let simAccumulator = 0;
let accumulatedTime = 0;
const maxSimTime = 30; //in seconds
let lastTime = performance.now();

//================
//Simulation state
//================
let solverHandle = 0;
let particleRenderer: ParticleRenderer | null = null;
let boundsRenderer: BoundsRenderer | null = null;

function startSimulation() {
    if (solverHandle)
        destroy(solverHandle);
    particleRenderer?.destroy();
    boundsRenderer?.Destroy();

    const solverConfigPtr = SolverConfigUtils.createSolverConfig(Module, config);
    solverHandle = create(solverConfigPtr);
    SolverConfigUtils.freeSolverConfig(Module, solverConfigPtr); //free the memory after creating solver

    initialize(solverHandle);

    particleRenderer = new ParticleRenderer(gpu, camera, getParticleCount(solverHandle));
    particleRenderer.initialize(); //setup renderer

    boundsRenderer = new BoundsRenderer(gpu, camera, getGridX(solverHandle), getGridY(solverHandle), getGridZ(solverHandle));
    boundsRenderer.Initialize();
    camera.lookAtBounds(getGridX(solverHandle), getGridY(solverHandle), getGridZ(solverHandle));

    simAccumulator = 0;
    accumulatedTime = 0;
    lastTime = performance.now();
}

//================
//UI
//================
const stats = { fps: 0 };

const gui = new GUI({ title: "FLIP Simulation" });
gui.add(stats, "fps").name("FPS").listen().disable();

const configFolder = gui.addFolder("New simulation");
configFolder.add(config, "gridX", 1).step(1);
configFolder.add(config, "gridY", 1).step(1);
configFolder.add(config, "gridZ", 1).step(1);
configFolder.add(config, "numParticles", 1).step(1);
configFolder.add(config, "particleRadius", 0.05, 2, 0.01);
configFolder.add(config, "alphaPic", 0, 1, 0.01);
configFolder.add(config, "useAdaptiveMixing");
configFolder.add({ start: startSimulation }, "start").name("Start simulation");

let fpsFrames = 0;
let fpsLastUpdate = performance.now();

startSimulation();

function draw() {
    Update();
    const canvasTexture = gpu.context.getCurrentTexture();
    const renderPassDescriptor: GPURenderPassDescriptor = {
        colorAttachments: [{
            view: canvasTexture.createView(),
            clearValue: [0.2, 0.2, 0.2, 1],
            loadOp: "clear",
            storeOp: "store"
        }],
        depthStencilAttachment: {
                view: depthTexture.createView(),
                depthClearValue: 1.0,
                depthLoadOp: "clear",
                depthStoreOp: "store"
            },
        };
    const encoder = gpu.device.createCommandEncoder();
    const pass = encoder.beginRenderPass(renderPassDescriptor);

    boundsRenderer?.Draw(pass);
    particleRenderer?.Draw(pass);

    pass.end();
    const commandBuffer = encoder.finish();
    gpu.device.queue.submit([commandBuffer]);
    requestAnimationFrame(draw);
}

function Update()
{
    const now = performance.now();
    const dt = (now - lastTime) / 1000; //in seconds
    lastTime = now;

    //fps counter
    fpsFrames++;
    if (now - fpsLastUpdate >= 500) {
        stats.fps = Math.round(fpsFrames * 1000 / (now - fpsLastUpdate));
        fpsFrames = 0;
        fpsLastUpdate = now;
    }

    //camera movement
    inputController.update(dt);


    if (solverHandle && particleRenderer && accumulatedTime < maxSimTime) {
        simAccumulator = Math.min(simAccumulator + dt, fixedDt); //never queue more than one physics step
        if (simAccumulator >= fixedDt && accumulatedTime < maxSimTime) {
            simulate(solverHandle, fixedDt); // physics step
            simAccumulator -= fixedDt;
            accumulatedTime += fixedDt;
        }

        if(accumulatedTime > maxSimTime)
        {
            writeMeasurementsToFile(solverHandle);
            console.log("Simulation finished: measurements written to measurements.csv");
            const data = Module.FS.readFile("measurements.csv", { encoding: "utf8" }); //get file from virtual file system
            const blob = new Blob([data], { type: "text/csv" });
            const url = URL.createObjectURL(blob);

            const a = document.createElement("a");
            a.href = url;
            a.download = "measurements.csv";
            a.click();

            URL.revokeObjectURL(url);
        }

        //push particle data to GPU
        const ptr = getParticlePositions(solverHandle);
        const positions = new Float32Array(Module.HEAPF32.buffer, ptr, getParticleCount(solverHandle) * 3); //Module.HeapF32.buffer is WASM memory, zero copy

        const gpuPositions = new Float32Array(getParticleCount(solverHandle) * 4);
        for (let i = 0; i < getParticleCount(solverHandle); i++) {
            gpuPositions[i * 4 + 0] = positions[i * 3 + 0];
            gpuPositions[i * 4 + 1] = positions[i * 3 + 1];
            gpuPositions[i * 4 + 2] = positions[i * 3 + 2];
            gpuPositions[i * 4 + 3] = 1.0; //w -> 1.0 for positions

            //console.log(`Particle ${i}: x=${positions[i * 3 + 0]}, y=${positions[i * 3 + 1]}, z=${positions[i * 3 + 2]}`); //DEBUG
        }
        particleRenderer.updateParticles(gpuPositions);
    }
}

draw();
