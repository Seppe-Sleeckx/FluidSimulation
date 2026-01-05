export class WebGPUContext {
    public device!: GPUDevice;
    public context!: GPUCanvasContext;
    public format!: GPUTextureFormat;

    private canvas: HTMLCanvasElement

    constructor(canvas: HTMLCanvasElement) {this.canvas = canvas}

    async initialize() {
        if (!navigator.gpu) { //request gpu object, entry point for webgpu
            throw new Error("WebGPU not supported");
        }

        const adapter = await navigator.gpu.requestAdapter();
        if (!adapter) throw new Error("No GPU adapter found!!");

        this.device = await adapter.requestDevice();
        this.context = this.canvas.getContext("webgpu")!;
        this.format = navigator.gpu.getPreferredCanvasFormat(); //get optimal format for current system, !!!doesnt work on every browser, test!!!

        this.context.configure({
            device: this.device,
            format: this.format,
            alphaMode: "opaque"
        });
    }
}
