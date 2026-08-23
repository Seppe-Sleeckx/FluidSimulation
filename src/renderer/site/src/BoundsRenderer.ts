import gridBoundsWGSL from "./shaders/grid-bounds.vert.wgsl?raw"
import singleColorWGSL from "./shaders/single-color-particles.frag.wgsl?raw"

import { Camera } from "./Camera"
import { WebGPUContext } from "./WebgpuContext";

export class BoundsRenderer {
    private pipeline!: GPURenderPipeline;
    private uniformBuffer!: GPUBuffer;
    private bindGroup!: GPUBindGroup;
    private gridVertexBuffer!: GPUBuffer;
    private gpuContext: WebGPUContext;
    private camera: Camera;
    private gridX: number;
    private gridY: number;
    private gridZ: number;

    constructor(gpuContext: WebGPUContext, camera: Camera, gridX: number, gridY: number, gridZ: number) {
        this.gpuContext = gpuContext;
        this.camera = camera;
        this.gridX = gridX;
        this.gridY = gridY;
        this.gridZ = gridZ;
    }

    Initialize(): void {
        const x = this.gridX;
        const y = this.gridY;
        const z = this.gridZ;

        const vertices = new Float32Array([
            0, 0, 0, 1,             x, 0, 0, 1, //Left front bottom -> right front bottom
            x, 0, 0, 1,             x, 0, z, 1, //Right front bottom -> right back bottom
            x, 0, z, 1,             0, 0, z, 1,   //right back bottom -> left back bottom
            0, 0, z, 1,             0, 0, 0, 1, //left back bottom -> left front bottom 

            0, y, 0, 1,             x, y, 0, 1, //Left front top -> right front top
            x, y, 0, 1,             x, y, z, 1, //Right front top -> right back top
            x, y, z, 1,             0, y, z, 1, //right back top -> left back top
            0, y, z, 1,             0, y, 0, 1, //Left back top -> Left back front
            
            0, 0, 0, 1,             0, y, 0, 1, //left front bottom -> left front top
            x, 0, 0, 1,             x, y, 0, 1, //right front bottom -> right front top
            x, 0, z, 1,             x, y, z, 1,//right back bottom -> right back top
            0, 0, z, 1,             0, y, z, 1,//left back bottom -> left back top
        ]);


        const device = this.gpuContext.device;

        //======================
        //Buffers
        //======================
        this.gridVertexBuffer = device.createBuffer({
            size: vertices.byteLength,
            usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST
        });
        device.queue.writeBuffer(this.gridVertexBuffer, 0, vertices);


        const uniformValues = new Float32Array(20) //4x4 matrix + vec2f resolution + f32 thickness + f32 _padding
        this.uniformBuffer = device.createBuffer({
            size: uniformValues.byteLength,
            usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST
        });

        const worldViewProjMatOffset = 0;
        const resolutionOffset = 4 * 4; //4x4 matrix
        const thicknessOffset = resolutionOffset + 2; //res_offset + vec2f
        const matrixValue = uniformValues.subarray(worldViewProjMatOffset, worldViewProjMatOffset + 16);
        const resolutionValue = uniformValues.subarray(resolutionOffset, resolutionOffset + 2);
        const thicknessValue = uniformValues.subarray(thicknessOffset, thicknessOffset + 1)

        const canvas = this.gpuContext.context.canvas as HTMLCanvasElement;
        thicknessValue[0] = 10.0; //temp
        resolutionValue[0] = canvas.width;
        resolutionValue[1] = canvas.height;
        matrixValue.set(this.camera.getViewProjection(canvas.width / canvas.height));

        device.queue.writeBuffer(this.uniformBuffer, 0, uniformValues);


        //=================
        //Shaders
        //=================
        const shaderModules = Object.fromEntries( //compile all shaders, and put them in a map
            Object.entries({
                gridBoundsWGSL,
                singleColorWGSL
            }).map(([key, code]) => [key, device.createShaderModule({ code })])
        );


        //=================
        //Pipeline
        //=================

        const bindGroupLayout = device.createBindGroupLayout({
            entries: [{binding: 0, visibility: GPUShaderStage.VERTEX, buffer: { type:'uniform'}},
                {binding: 1, visibility: GPUShaderStage.VERTEX, buffer:{type:'read-only-storage'}}]
        });

        const pipelineLayout = device.createPipelineLayout({
            bindGroupLayouts: [bindGroupLayout],
        });


        this.pipeline = device.createRenderPipeline({
            layout: pipelineLayout,
            vertex: {
                module: shaderModules.gridBoundsWGSL,
                entryPoint: "vs_main",
                buffers: []
            },
            fragment: {
                module: shaderModules.singleColorWGSL,
                entryPoint: "fs_main",
                targets: [{format: this.gpuContext.format,}]
            },
            primitive:{
                topology: "triangle-list", //for each line we draw a quad
                cullMode: "none"
            },
            depthStencil: {
                format: "depth24plus",
                depthWriteEnabled: true, //updates depth buffer when rendering
                depthCompare: "less"
            }
        });

        //================
        //Bindgroup
        //================
        this.bindGroup = device.createBindGroup({
            layout: this.pipeline.getBindGroupLayout(0), //@group(0)
            entries: [
                { binding: 0, /*@binding(0)*/ resource: { buffer: this.uniformBuffer } /*camera view projection matrix buffer*/ },
                { binding: 1, /*@binding(1)*/ resource: { buffer: this.gridVertexBuffer}}
            ]
        });
    }

    Destroy(): void {
        this.gridVertexBuffer.destroy();
        this.uniformBuffer.destroy();
    }

    Draw(pass: GPURenderPassEncoder) : void{
        //Set view projection matrix from camera
        const canvas = this.gpuContext.context.canvas as HTMLCanvasElement;
        const viewProjection = this.camera.getViewProjection(canvas.width / canvas.height) as Float32Array;
        this.gpuContext.device.queue.writeBuffer(this.uniformBuffer, 0, viewProjection.buffer, viewProjection.byteOffset, viewProjection.byteLength);

        pass.setPipeline(this.pipeline);
        pass.setBindGroup(0, this.bindGroup);
        pass.draw(6, 12); //6 vertices per line, 12 lines total (for a box)
    }
}