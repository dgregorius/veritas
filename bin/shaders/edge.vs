
#version 460 core

// Attributes
layout (location = 0) in vec3 aVertexPosition;
layout (location = 1) in vec4 aVertexColor;
	
layout( std140, binding = 0 ) uniform CameraBlock 
{
mat4 ViewMatrix;
mat4 ProjectionMatrix;
vec4 Position;
} Camera;

// SSBO for mesh instances
struct MeshData
{
	mat4 Transform;
	vec4 Color;
};

layout(std430, binding = 0) buffer MeshInstances
{
    MeshData Instance[];
};

// Varyings
out vec4 vColor;
		
void main()
{
	mat4 ModelMatrix = Instance[ gl_InstanceID ].Transform;
	gl_Position = Camera.ProjectionMatrix * Camera.ViewMatrix * ModelMatrix * vec4( aVertexPosition, 1.0 );
	
	vColor = aVertexColor; 
}