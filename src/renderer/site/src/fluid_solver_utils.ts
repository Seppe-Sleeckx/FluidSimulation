export interface SolverConfig {
    gridX: number;
    gridY: number;
    gridZ: number;
    numParticles: number;
    particleRadius: number;
    alphaPic: number;
    useAdaptiveMixing: boolean;
}

export function createSolverConfig(Module: any, config: SolverConfig): number { //we return a number, this is actually the ptr to the config in wasm memory
    const BYTESIZE = 24;
    const ptr = Module._malloc(BYTESIZE);

    const i32 = Module.HEAP32;
    const f32 = Module.HEAPF32;

    const baseI32 = ptr >> 2;

    i32[baseI32 + 0] = config.gridX;
    i32[baseI32 + 1] = config.gridY;
    i32[baseI32 + 2] = config.gridZ;
    i32[baseI32 + 3] = config.numParticles;

    f32[(ptr + 16) >> 2] = config.particleRadius;
    f32[(ptr + 20) >> 2] = config.alphaPic;

    return ptr;
}

export function freeSolverConfig(Module: any, ptr: number): void
{
    Module._free(ptr);
}
