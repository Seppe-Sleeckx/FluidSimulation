import { vec3, mat4 } from "gl-matrix";

export class Camera {
    position = vec3.fromValues(0, 0, 0);
    yaw = 0;
    pitch = 0;

    moveSpeed = 5.0;

    forward = vec3.fromValues(0, 0, 1);
    right = vec3.fromValues(1, 0, 0);
    up = vec3.fromValues(0, 1, 0);

    private view = mat4.create();
    private proj = mat4.create();
    private viewProj = mat4.create();

    // Compute forward/right vectors
    updateVectors() {
        this.forward[0] = Math.cos(this.pitch) * Math.sin(this.yaw);
        this.forward[1] = Math.sin(this.pitch);
        this.forward[2] = Math.cos(this.pitch) * Math.cos(this.yaw);
        vec3.normalize(this.forward, this.forward);

        const worldUp = vec3.fromValues(0, 1, 0)
        vec3.cross(this.right, this.forward, worldUp);
        vec3.normalize(this.right, this.right);

        vec3.cross(this.up, this.right, this.forward);
        vec3.normalize(this.up, this.up);
    }

    lookAtBounds(x: number, y: number, z: number, fovY = Math.PI / 4) {
        const center = vec3.fromValues(x / 2, y / 2, z / 2);
        const halfExtent = Math.max(x, y) / 2;
        const distance = halfExtent / Math.tan(fovY / 2) * 1.2; // 20% margin

        vec3.set(this.position, center[0], center[1], z + distance);

        const dir = vec3.create();
        vec3.subtract(dir, center, this.position);
        vec3.normalize(dir, dir);
        this.yaw = Math.atan2(dir[0], dir[2]);
        this.pitch = Math.asin(dir[1]);
        this.updateVectors();
    }

    getViewProjection(aspect: number): mat4 {
        this.updateVectors();

        const target = vec3.create();
        vec3.add(target, this.position, this.forward);

        mat4.lookAt(this.view, this.position, target, this.up);
        mat4.perspective(this.proj, Math.PI / 4, aspect, 0.01, 1000);
        mat4.multiply(this.viewProj, this.proj, this.view);

        return this.viewProj;
    }
}
