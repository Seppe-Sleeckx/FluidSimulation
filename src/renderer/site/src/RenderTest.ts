import triangleVertWGSL from "./shaders/triangle.vert.wgsl?raw"
import singleColorFragWGSL from "./shaders/single-color-particles.frag.wgsl?raw"

import { Camera } from "./Camera";
import { WebGPUContext } from "./WebgpuContext";

export class RenderTest {
    private pipeline!: GPURenderPipeline;
    private uniformBuffer!: GPUBuffer;
    private bindGroup!: GPUBindGroup;
    private particleBuffer!: GPUBuffer;
    private depthTexture!: GPUTexture;
    private gpuContext: WebGPUContext;
    private camera: Camera;

    constructor(gpuContext: WebGPUContext, camera: Camera) {
        this.gpuContext = gpuContext;
        this.camera = camera;
    }

    initialize() {
        const canvas = document.querySelector('canvas') as HTMLCanvasElement;

        const device = this.gpuContext.device
        const context = this.gpuContext.context

        const devicePixelRatio = window.devicePixelRatio;
        canvas.width = canvas.clientWidth * devicePixelRatio;
        canvas.height = canvas.clientHeight * devicePixelRatio;
        const presentationFormat = navigator.gpu.getPreferredCanvasFormat();

        context.configure({
            device,
            format: presentationFormat,
        });

        this.pipeline = device.createRenderPipeline({
            layout: 'auto',
            vertex: {
                module: device.createShaderModule({
                    code: triangleVertWGSL,
                }),
            },
            fragment: {
                module: device.createShaderModule({
                    code: singleColorFragWGSL,
                }),
                targets: [
                    {
                        format: presentationFormat,
                    },
                ],
            },
            primitive: {
                topology: 'triangle-list',
            },
        });
    }

    draw() {
        const commandEncoder = this.gpuContext.device.createCommandEncoder();
        const textureView = this.gpuContext.context.getCurrentTexture().createView();

        const renderPassDescriptor: GPURenderPassDescriptor = {
            colorAttachments: [
                {
                    view: textureView,
                    clearValue: [0, 0, 0, 0], // Clear to transparent
                    loadOp: 'clear',
                    storeOp: 'store',
                },
            ],
        };

        const passEncoder = commandEncoder.beginRenderPass(renderPassDescriptor);
        passEncoder.setPipeline(this.pipeline);
        passEncoder.draw(3);
        passEncoder.end();

        this.gpuContext.device.queue.submit([commandEncoder.finish()]);
    }
}