
#version 460 core

#define kAntialiasing 2.0

struct InstanceData
{
    mat4 Model;
    vec4 Color;
};

layout(std430, binding = 0) readonly buffer InstanceBuffer 
{
    InstanceData instances[];
};


struct EdgeData 
{
    vec3 Position1;
    uint Color1;
    vec3 Position2;
    uint Color2;
};

layout(std430, binding = 1) readonly buffer EdgeBuffer 
{
    EdgeData edges[];
};

uniform mat4 uViewProj;
uniform vec2 uResolution;
uniform float uLineWidth;
uniform int uEdgeCount;

noperspective out vec4 vColor;
noperspective out float vEdgeDist;
noperspective out float vSize;

vec4 unpackColor(uint c) 
{
    return vec4
    (
        float((c >> 0) & 0xFF) / 255.0,
        float((c >> 8) & 0xFF) / 255.0,
        float((c >> 16) & 0xFF) / 255.0,
        float((c >> 24) & 0xFF) / 255.0
    );
}

void main() 
{
    int vertIdx = gl_VertexID;
    int edgeIdx = gl_InstanceID % uEdgeCount;
    int meshIdx = gl_InstanceID / uEdgeCount;
    
    EdgeData edge = edges[edgeIdx];
    mat4 model = instances[meshIdx].Model;
    
    vec3 worldP0 = (model * vec4(edge.Position1, 1.0)).xyz;
    vec3 worldP1 = (model * vec4(edge.Position2, 1.0)).xyz;
    
    vec4 clip0 = uViewProj * vec4(worldP0, 1.0);
    vec4 clip1 = uViewProj * vec4(worldP1, 1.0);
    
    vec2 screen0 = uResolution * (0.5 * clip0.xy / clip0.w + 0.5);
    vec2 screen1 = uResolution * (0.5 * clip1.xy / clip1.w + 0.5);
    
    vec2 dir = screen1 - screen0;
    float lineLen = length(dir);
    dir = lineLen > 0.0 ? dir / lineLen : vec2(1.0, 0.0);
    vec2 normal = vec2(-dir.y, dir.x);
    
    float expand = uLineWidth * 0.5 + 0.5;
    
    // Fixed vertex mapping for GL_TRIANGLES (0,1,2) and (3,4,5)
    // Quad:  0---2
    //        |  /|
    //        | / |
    //        |/  |
    //        1---3
    // Triangle 1: 0,1,2  Triangle 2: 2,1,3 remapped to 3,4,5
    float endpoint = (vertIdx >= 2 && vertIdx != 4) ? 1.0 : 0.0;
    float side = (vertIdx == 1 || vertIdx >= 4) ? 1.0 : -1.0;
    
    vec2 screen = mix(screen0, screen1, endpoint);
    screen += normal * side * expand;
    
    vec4 clip = mix(clip0, clip1, endpoint);
    vec2 ndc = (screen / uResolution) * 2.0 - 1.0;
    
    gl_Position = vec4(ndc * clip.w, clip.z, clip.w);
    vColor = unpackColor(endpoint < 0.5 ? edge.Color1 : edge.Color2);
    vEdgeDist = side * expand;
    vSize = max(uLineWidth, kAntialiasing);
}