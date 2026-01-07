import pointParticlesWGSL from "./shaders/point-particles.vert.wgsl?raw"
import singleColorParticlesWGSL from "./shaders/single-color-particles.frag.wgsl?raw"


import { Camera } from "./Camera";
import { WebGPUContext } from "./WebgpuContext";

export class ParticleRenderer {
    private pipeline!: GPURenderPipeline;
    private uniformBuffer!: GPUBuffer;
    private bindGroup!: GPUBindGroup;
    private particleBuffer!: GPUBuffer;
    private depthTexture!: GPUTexture;
    private gpuContext: WebGPUContext;
    private camera: Camera;
    private maxParticles: number;

    constructor(gpuContext: WebGPUContext, camera: Camera, maxParticles: number) {
        this.gpuContext = gpuContext;
        this.camera = camera;
        this.maxParticles = maxParticles;
    }

    initialize() {
        const device = this.gpuContext.device;

        const bindGroupLayout = device.createBindGroupLayout({
            entries: [
                {
                    binding: 0,
                    visibility: GPUShaderStage.VERTEX,
                    buffer: {}
                }
            ]
        })

        const pipelineLayout = device.createPipelineLayout({
            bindGroupLayouts: [bindGroupLayout],
        });


        //=================
        //Shaders
        //=================
        const shaderModules = Object.fromEntries( //compile all shaders, and put them in a map
            Object.entries({
                pointParticlesWGSL,
                singleColorParticlesWGSL
            }).map(([key, code]) => [key, device.createShaderModule({ code })])
        );


        //=================
        //Pipeline
        //=================
        this.pipeline = device.createRenderPipeline({
            layout: pipelineLayout,
            vertex: {
                module: shaderModules.pointParticlesWGSL,
                entryPoint: "vs_main",
                buffers: [{
                    arrayStride: 4 * 4, //vec4<f32>
                    stepMode: "instance",
                    attributes: [{ shaderLocation: 0, offset: 0, format: "float32x4" }] //shaderLocation: 0 == @location(0), float32x4 == vec4<f32>
                }]
            },
            fragment: {
                module: shaderModules.singleColorParticlesWGSL,
                entryPoint: "fs_main",
                targets: [
                    { 
                        format: this.gpuContext.format,
                        blend:{
                            color: {
                                srcFactor: "one",
                                dstFactor: "one-minus-src-alpha",
                            },
                            alpha: {
                                srcFactor: "one",
                                dstFactor: "one-minus-src-alpha",
                            }
                        }
                    }
                ]
            },
            depthStencil: {
                format: "depth24plus",
                depthWriteEnabled: true, //updates depth buffer when rendering
                depthCompare: "less"
            }
        });


        //======================
        //Buffers
        //======================


        const uniformValues = new Float32Array(20) //4x4 matrix + resolution(vec2f) + size (f32) + 4 bytes padding
        this.uniformBuffer = device.createBuffer({
            size: uniformValues.byteLength,
            usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST
        });
        //UNIFORM: readable by shaders
        //COPY_DST: can be updated by cpu
        const viewProjMatrixOffset = 0;
        const resolutionOffset = 4 * 4; //4x4 matrix
        const sizeOffset = resolutionOffset + 2; //res_offset + vec2f
        const matrixValue = uniformValues.subarray(viewProjMatrixOffset, viewProjMatrixOffset + 16);
        const resolutionValue = uniformValues.subarray(resolutionOffset, resolutionOffset + 2);
        const sizeValue = uniformValues.subarray(sizeOffset, sizeOffset + 1)

        const canvas = this.gpuContext.context.canvas as HTMLCanvasElement;
        sizeValue[0] = 10.0; //temp
        resolutionValue[0] = canvas.width;
        resolutionValue[1] = canvas.height;
        matrixValue.set(this.camera.getViewProjection(canvas.width / canvas.height));

        device.queue.writeBuffer(this.uniformBuffer, 0, uniformValues); //write all values to the buffer


        //===============
        //Particle buffer
        //===============
        //TEMP: test
        const rand = (min: number, max: number) => min + Math.random() * (max - min);
        const vertexData = new Float32Array(this.maxParticles * 4);
        for (let i = 0; i < this.maxParticles; ++i) {
            const offset = i * 4;
            vertexData[offset + 0] = rand(-1, 1);
            vertexData[offset + 1] = rand(-1, 1);
            vertexData[offset + 2] = rand(-1, 1);
            vertexData[offset + 3] = 1;
        }


        this.particleBuffer = device.createBuffer({ //particle buffer, max particles * vec4<f32>
            size: vertexData.byteLength,
            usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST
        });
        //VERTEX: used as vertex input
        //COPY_DST: can be updated by cpu


        //TEMP: test
        device.queue.writeBuffer(this.particleBuffer, 0, vertexData);


        this.depthTexture = device.createTexture({
            size: [canvas.width, canvas.height],
            format: "depth24plus", //24 bit depth buffer
            usage: GPUTextureUsage.RENDER_ATTACHMENT //allow this texture to be used in a render pass
        });


        //==============
        //Bindgroup
        //==============
        this.bindGroup = device.createBindGroup({
            layout: this.pipeline.getBindGroupLayout(0), //@group(0)
            entries: [
                { binding: 0, /*@binding(0)*/ resource: { buffer: this.uniformBuffer } /*camera view projection matrix buffer*/ }
            ]
        });



    }

    updateParticles(data: Float32Array) {
        this.gpuContext.device.queue.writeBuffer(this.particleBuffer, 0, data.buffer, data.byteOffset, data.byteLength);
    }

    draw() {

        const canvasTexture = this.gpuContext.context.getCurrentTexture();

        const renderPassDescriptor: GPURenderPassDescriptor = {
            colorAttachments: [{
                view: canvasTexture.createView(),
                clearValue: [0.1, 0.1, 0.1, 1],
                loadOp: "clear",
                storeOp: "store"
            }],
            depthStencilAttachment: {
                view: this.depthTexture.createView(),
                depthClearValue: 1.0,
                depthLoadOp: "clear",
                depthStoreOp: "store"
            },
        };

        //Set view projection matrix from camera
        const canvas = this.gpuContext.context.canvas as HTMLCanvasElement;
        const viewProjection = this.camera.getViewProjection(canvas.width / canvas.height);
        this.gpuContext.device.queue.writeBuffer(this.uniformBuffer, 0, viewProjection.buffer, viewProjection.byteOffset, viewProjection.byteLength);

        const encoder = this.gpuContext.device.createCommandEncoder();
        const pass = encoder.beginRenderPass(renderPassDescriptor);

        pass.setPipeline(this.pipeline);
        pass.setBindGroup(0, this.bindGroup);
        pass.setVertexBuffer(0, this.particleBuffer);
        pass.draw(6, this.maxParticles);
        pass.end();

        const commandBuffer = encoder.finish();
        this.gpuContext.device.queue.submit([commandBuffer]);
    }
}
