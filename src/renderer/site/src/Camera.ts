export class Camera {
    yaw = 0;
    pitch = 0;
    distance = 3;

    getViewProjection(aspect: number): Float32Array {
        const eye = [
            this.distance * Math.cos(this.pitch) * Math.sin(this.yaw),
            this.distance * Math.sin(this.pitch),
            this.distance * Math.cos(this.pitch) * Math.cos(this.yaw),
        ];

        const view = lookAt(eye, [0,0,0], [0,1,0]);
        const proj = perspective(Math.PI/4, aspect, 0.01, 100);

        return multiplyMat4(proj, view);
    }
}

function perspective(fovy: number, aspect: number, near: number, far: number) : Float32Array {
    const f = 1.0 / Math.tan(fovy / 2);
    const nf = 1 / (near - far);

    return new Float32Array([
        f / aspect, 0, 0, 0,
        0, f, 0, 0,
        0, 0, (far + near) * nf, -1,
        0, 0, (2 * far * near) * nf, 0,
    ]);
}

function lookAt(eye: number[], center: number[], up: number[]): Float32Array {
    const zx = eye[0] - center[0];
    const zy = eye[1] - center[1];
    const zz = eye[2] - center[2];
    const zl = Math.hypot(zx, zy, zz);
    const z = [zx / zl, zy / zl, zz / zl];

    let x = [
        up[1] * z[2] - up[2] * z[1],
        up[2] * z[0] - up[0] * z[2],
        up[0] * z[1] - up[1] * z[0]
    ];
    const xl = Math.hypot(x[0], x[1], x[2]);
    x = x.map(v => v / xl);

    const y = [
        z[1]*x[2] - z[2]*x[1],
        z[2]*x[0] - z[0]*x[2],
        z[0]*x[1] - z[1]*x[0]
    ];

    // Column-major
    return new Float32Array([
        x[0], y[0], z[0], 0,
        x[1], y[1], z[1], 0,
        x[2], y[2], z[2], 0,
        -(x[0]*eye[0]+x[1]*eye[1]+x[2]*eye[2]),
        -(y[0]*eye[0]+y[1]*eye[1]+y[2]*eye[2]),
        -(z[0]*eye[0]+z[1]*eye[1]+z[2]*eye[2]),
        1
    ]);
}


function multiplyMat4(a: Float32Array, b: Float32Array): Float32Array {
    const out = new Float32Array(16);
    for (let r = 0; r < 4; ++r) {
        for (let c = 0; c < 4; ++c) {
            out[c*4 + r] =
                a[0*4 + r]*b[c*4 + 0] +
                a[1*4 + r]*b[c*4 + 1] +
                a[2*4 + r]*b[c*4 + 2] +
                a[3*4 + r]*b[c*4 + 3];
        }
    }
    return out;
}
