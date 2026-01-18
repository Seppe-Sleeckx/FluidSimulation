import { WebGPUContext } from "./WebgpuContext";
import { Camera } from "./Camera";
import { InputController } from "./ImputController";
import { ParticleRenderer } from "./ParticleRenderer";
import { BoundsRenderer } from "./BoundsRenderer.ts"
import fluidSolverModule from "./fluid_solver.js";
import * as SolverConfigUtils from "./fluid_solver_utils.ts"

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


const config: SolverConfigUtils.SolverConfig = {
    gridX: 20,
    gridY: 10,
    gridZ: 20,
    numParticles: 1000,
    particleRadius: 0.5,
    alphaPic: 0.95
}
const solverConfigPtr = SolverConfigUtils.createSolverConfig(Module, config); //temp, change later
const solverHandle = create(solverConfigPtr);
SolverConfigUtils.freeSolverConfig(Module, solverConfigPtr); //free the memory after creating our solver

initialize(solverHandle);

//================
//Renderers
//================
const particleRenderer = new ParticleRenderer(gpu, camera, getParticleCount(solverHandle));
particleRenderer.initialize(); //setup renderer for x amount fo particles

const boundsRenderer = new BoundsRenderer(gpu, camera, getGridX(solverHandle), getGridY(solverHandle), getGridZ(solverHandle));
boundsRenderer.Initialize();


const depthTexture = gpu.device.createTexture({
    size: [canvas.width, canvas.height],
    format: "depth24plus", //24 bit depth buffer
    usage: GPUTextureUsage.RENDER_ATTACHMENT //allow this texture to be used in a render pass
});


let lastTime = performance.now();
function draw() {
    const now = performance.now();
    const dt = (now - lastTime) / 1000; //in seconds
    lastTime = now;

    //camera movement
    inputController.update(dt);


    //Physics (move to update later)
    simulate(solverHandle, dt); //physics step
    //push particle data to GPU
    const ptr = getParticlePositions(solverHandle);
    const positions = new Float32Array(Module.HEAPF32.buffer, ptr, getParticleCount(solverHandle) * 3); //Module.HeapF32.buffer is WASM memory, zero copy

    const gpuPositions = new Float32Array(getParticleCount(solverHandle) * 4);
    for (let i = 0; i < getParticleCount(solverHandle); i++) {
        gpuPositions[i * 4 + 0] = positions[i * 3 + 0];
        gpuPositions[i * 4 + 1] = positions[i * 3 + 1];
        gpuPositions[i * 4 + 2] = positions[i * 3 + 2];
        gpuPositions[i * 4 + 3] = 1.0; //w -> 1.0 for positions
    }

    particleRenderer.updateParticles(gpuPositions);


    //Rendering
    const canvasTexture = gpu.context.getCurrentTexture();
    const renderPassDescriptor: GPURenderPassDescriptor = {
        colorAttachments: [{
            view: canvasTexture.createView(),
            clearValue: [0.3, 0.3, 0.3, 1],
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

    boundsRenderer.Draw(pass);
    particleRenderer.Draw(pass);

    pass.end();
    const commandBuffer = encoder.finish();
    gpu.device.queue.submit([commandBuffer]);
    requestAnimationFrame(draw);
}

draw();
