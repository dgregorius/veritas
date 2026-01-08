#version 460 core

layout( std140, binding = 0 ) uniform CameraBlock 
{
    mat4 ViewMatrix;
    mat4 ProjectionMatrix;
    vec4 Position;
} Camera;

out vec3 vNearPoint;
out vec3 vFarPoint;

// Helper to un-project screen coordinates back into world space
vec3 UnprojectPoint(float X, float Y, float Z, mat4 ViewInv, mat4 ProjInv) 
{
    vec4 UnprojectedPoint =  ViewInv * ProjInv * vec4(X, Y, Z, 1.0);
    return UnprojectedPoint.xyz / UnprojectedPoint.w;
}

void main() {
    uint Index = uint(gl_VertexID);
    float X = float((Index & 1U) << 2U) - 1.0;
    float Y = float((Index & 2U) << 1U) - 1.0;

    mat4 ViewInv = inverse(Camera.ViewMatrix);
    mat4 ProjInv = inverse(Camera.ProjectionMatrix);

    // We calculate a ray from the 'near plane' to the 'far plane' for every pixel
    vNearPoint = UnprojectPoint(X, Y, 0.0, ViewInv, ProjInv);
    vFarPoint = UnprojectPoint(X, Y, 1.0, ViewInv, ProjInv);

    gl_Position = vec4(X, Y, 0.0, 1.0); // Full screen coverage
}