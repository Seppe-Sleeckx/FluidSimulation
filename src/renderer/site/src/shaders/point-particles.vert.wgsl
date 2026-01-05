struct Uniforms{
    viewProjMat: mat4x4<f32>,
    resolution: vec2f,
    size: f32,
}

@group(0) @binding(0) var<uniform> uni : Uniforms;

struct VertexInput {
    @location(0) position: vec4f,
};

struct VertexOutput{
    @builtin(position) position: vec4f,
}

@vertex
fn vs_main(input: VertexInput,  @builtin(vertex_index) vIdx: u32) -> VertexOutput{
    let points = array(
    vec2f(-1, -1),
    vec2f( 1, -1),
    vec2f(-1,  1),
    vec2f(-1,  1),
    vec2f( 1, -1),
    vec2f( 1,  1),
    );

    let pos = points[vIdx];
    let clipPos = uni.viewProjMat * input.position;
    let pointPos = vec4f(pos * uni.size / uni.resolution, 0, 0);
    var vertexOut: VertexOutput;
    vertexOut.position = clipPos + pointPos;
    return vertexOut;
}