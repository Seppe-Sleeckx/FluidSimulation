import { Camera } from "./Camera";

export class InputController {
    private dragging = false;
    private lastX = 0;
    private lastY = 0;
    private sensitivity = 0.005;

    constructor(canvas: HTMLCanvasElement, camera: Camera) {
        canvas.addEventListener("mousedown", e => {
            this.dragging = true;
            this.lastX = e.clientX;
            this.lastY = e.clientY;
        });

        canvas.addEventListener("mouseup", () => {
            this.dragging = false;
        });

        canvas.addEventListener("mousemove", e => {
            if (!this.dragging) return;

            camera.yaw += (e.clientX - this.lastX) * this.sensitivity;
            camera.pitch += (e.clientY - this.lastY) * this.sensitivity;

            camera.pitch = Math.max(-1.5, Math.min(1.5, camera.pitch));

            this.lastX = e.clientX;
            this.lastY = e.clientY;
        });
    }
}
