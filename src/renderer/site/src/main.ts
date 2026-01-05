import { WebGPUContext } from "./WebgpuContext";
import { Camera } from "./Camera";
import { InputController } from "./ImputController";
import { ParticleRenderer } from "./ParticleRenderer";
import { RenderTest } from "./RenderTest";

const canvas = document.getElementById("simulation") as HTMLCanvasElement;

canvas.width = canvas.clientWidth * window.devicePixelRatio;
canvas.height = canvas.clientHeight * window.devicePixelRatio;

const gpu = new WebGPUContext(canvas);
await gpu.initialize(); //create and initialize context

const camera = new Camera();
new InputController(canvas, camera); //create camera and bind it with our inputcontroller

const renderer = new ParticleRenderer(gpu, camera, 1000);
renderer.initialize(); //setup renderer for x amount fo particles

//const renderer = new RenderTest(gpu, camera);
//renderer.initialize();

function draw() {
    renderer.draw();
    requestAnimationFrame(draw);
}

draw();
