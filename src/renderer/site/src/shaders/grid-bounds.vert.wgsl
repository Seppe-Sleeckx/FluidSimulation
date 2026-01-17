struct Uniforms{
    worldViewProjMat: mat4x4<f32>,
    resolution: vec2f,
    thickness: f32,
    _padding: f32 //best to explicitly add padding for clarity
}

@group(0) @binding(0) var<uniform> uni : Uniforms;
@group(0) @binding(1) var<storage, read> linePositions : array<vec4 <f32>>; //p0 edge0, p1 edge0, p0 edge 1, p1 edge 1, ...

struct VertexInput {
    @builtin(vertex_index) vertexIdx : u32, //0,1,2,3,4,5
    @builtin(instance_index) instanceIdx : u32, //line 0,1,2,...
};

struct VertexOutput {
    @builtin(position) pos : vec4<f32>,
};

@vertex
fn vs_main(input: VertexInput) -> VertexOutput {
    let edge = input.instanceIdx * 2u;
    let p0 = linePositions[edge + 0u];
    let p1 = linePositions[edge + 1u];

    //transform to clip space
    let clipp0 = uni.worldViewProjMat * p0;
    let clipp1 = uni.worldViewProjMat * p1;

    //transform to ndc coordinate space (WEBGPU z == [0, 1])                                                                                               
    let ndcp0 = clipp0.xyz / clipp0.w;
    let ndcp1 = clipp1.xyz / clipp1.w;

    //(needed for line thickness)
    let dir = ndcp1.xy - ndcp0.xy;
    let len = length(dir);

    let linedir = select(
    vec2<f32>(1.0, 0.0), // fallback direction
    dir / len,
    len > 1e-5
    );

    let perp = vec2<f32>(-linedir.y, linedir.x);


    let halfWidth = uni.thickness * 0.5 / uni.resolution.x;


    //get correct corner of quad
    var offset : vec2<f32>;
    var t : f32;
    switch (input.vertexIdx) {
        case 0u: { offset = perp * halfWidth; t = 0.0;} //(p0 top)
        case 1u: { offset = -perp * halfWidth; t = 0.0;} //(p0 bottom)
        case 2u: { offset = perp * halfWidth; t = 1.0;} //(p1 top)
        case 3u: { offset = perp * halfWidth; t = 1.0;} //(p1 top_)
        case 4u: { offset = -perp * halfWidth; t = 0.0; } //(p0 bottom)
        case 5u: { offset = -perp * halfWidth; t = 1.0; } //(p1 bottom)
        default:{ offset = vec2<f32>(0,0); t = 0.0;} 
    }

    let w = mix(clipp0.w, clipp1.w, t);
    let posNDC = mix(ndcp0, ndcp1, t) + vec3<f32>(offset, 0.0);

    var out: VertexOutput;
    out.pos = vec4<f32>(posNDC * w, w);
    return out;
}
