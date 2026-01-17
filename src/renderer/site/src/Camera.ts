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
