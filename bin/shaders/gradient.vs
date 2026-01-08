#version 460 core

// Varying
out vec2 vUV;

void main() 
{
    uint idx = uint(gl_VertexID);
    float x = -1.0 + float((idx & 1U) << 2U);
    float y = -1.0 + float((idx & 2U) << 1U);

    vUV = vec2(x, y) * 0.5 + 0.5;
    gl_Position = vec4(x, y, 0.0, 1.0);
}