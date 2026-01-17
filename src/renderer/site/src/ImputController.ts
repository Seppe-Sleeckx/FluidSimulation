import { Camera } from "./Camera";
import {vec3} from "gl-matrix"

export class InputController {
    private dragging = false;
    private lastX = 0;
    private lastY = 0;
    private sensitivity = 0.005;
    private keys = new Set<string>();
    private controlledCamera : Camera; 

    constructor(canvas: HTMLCanvasElement, camera: Camera) {
        this.controlledCamera = camera;

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

            this.controlledCamera.yaw -= (e.clientX - this.lastX) * this.sensitivity;
            this.controlledCamera.pitch -= (e.clientY - this.lastY) * this.sensitivity;

            this.controlledCamera.pitch = Math.max(-1.5, Math.min(1.5, this.controlledCamera.pitch));

            this.lastX = e.clientX;
            this.lastY = e.clientY;
        });

        //Keyboard
        window.addEventListener("keydown", e => this.keys.add(e.code));
        window.addEventListener("keyup", e => this.keys.delete(e.code));
    }

    update(dt: number) {
        const speed = this.controlledCamera.moveSpeed * dt;

        //Update camera positoin
        if (this.keys.has("KeyW")) vec3.scaleAndAdd(this.controlledCamera.position, this.controlledCamera.position, this.controlledCamera.forward, speed);
        if (this.keys.has("KeyS")) vec3.scaleAndAdd(this.controlledCamera.position, this.controlledCamera.position, this.controlledCamera.forward, -speed);
        if (this.keys.has("KeyA")) vec3.scaleAndAdd(this.controlledCamera.position, this.controlledCamera.position, this.controlledCamera.right, -speed);
        if (this.keys.has("KeyD")) vec3.scaleAndAdd(this.controlledCamera.position, this.controlledCamera.position, this.controlledCamera.right, speed);
    }
}
