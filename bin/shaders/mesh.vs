#version 460 core

// Attributes
layout (location = 0) in vec3 aVertexPosition;
layout (location = 1) in vec3 aVertexNormal;

layout( std140, binding = 0 ) uniform CameraBlock 
{
mat4 ViewMatrix;
mat4 ProjectionMatrix;
vec4 Position;
} Camera;

// SSBO for mesh instances
struct InstanceData
{
	mat4 Transform;
	vec4 Color;
};

layout(std430, binding = 0) buffer MeshInstances
{
    InstanceData Instance[];
};

// Varyings
out vec3 vWorldPosition;
out vec3 vWorldNormal;
out vec4 vColor;

void main()
{
	// Calculate World Position and Normal
	mat4 ModelMatrix = Instance[ gl_InstanceID ].Transform;
    vWorldPosition = vec3( ModelMatrix * vec4( aVertexPosition, 1.0 ) );
	vWorldNormal = mat3( transpose( inverse( ModelMatrix ) ) ) * aVertexNormal;

    // Pass color
    vColor = Instance[ gl_InstanceID ].Color;
	
    // Final Clip Space Position
	gl_Position = Camera.ProjectionMatrix * Camera.ViewMatrix * vec4( vWorldPosition, 1.0 );
}