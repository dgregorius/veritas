
#version 460 core

// Attributes
layout (location = 0) in vec3 aVertexPosition;
layout (location = 1) in vec4 aVertexColor;
	
// Uniforms
layout (location = 0) uniform mat4 uProjectionMatrix;
layout (location = 1) uniform mat4 uViewMatrix;

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
	gl_Position = uProjectionMatrix * uViewMatrix * ModelMatrix * vec4( aVertexPosition, 1.0 );
	
	vColor = Instance[ gl_InstanceID ].Color * aVertexColor; 
}