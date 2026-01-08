//--------------------------------------------------------------------------------------------------
// shaderlibrary.cpp
//
// Copyright (c) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------
#include "shaderlibrary.h"
#include "shader.h"

// OpenGL
#include <glad.h>
#include <glfw3.h>


//--------------------------------------------------------------------------------------------------
// Shader Sources
//--------------------------------------------------------------------------------------------------
const char* SuperTriangleVS = R"(
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
)";


//--------------------------------------------------------------------------------------------------
const char* BackgroundFS = R"(
#version 460 core

// Varying
in vec2 vUV;

// Output
out vec4 FragColor;

void main() 
{
   // Exact colors sampled from your uploaded image
    vec3 topColor    = vec3(0.38, 0.45, 0.50); // Muted slate blue
    vec3 bottomColor = vec3(0.69, 0.73, 0.75); // Light grey-blue

    // Smoothstep creates a more natural "lens" feel than a linear mix
    // It holds the colors slightly longer at the top and bottom
    float t = smoothstep(-0.1, 1.1, vUV.y);
    
    vec3 sky = mix(bottomColor, topColor, t);

    // Simple noise dither to prevent 8-bit banding
    float ditherStrength = 0.5; 
    float noise = fract(sin(dot(vUV, vec2(12.9898, 78.233))) * 43758.5453);
    sky += (noise - 0.5) * (ditherStrength / 255.0);
   
    FragColor = vec4(sky, 1.0);
}
)";


//--------------------------------------------------------------------------------------------------
const char* GridVS = R"(
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
)";

const char* GridFS = R"(
#version 460 core

layout( std140, binding = 0 ) uniform CameraBlock 
{
mat4 ViewMatrix;
mat4 ProjectionMatrix;
vec4 Position;
} Camera;

in vec3 vNearPoint;
in vec3 vFarPoint;

out vec4 FragColor;


float getGrid(vec2 pos, float scale, float thickness) 
{
vec2 coord = pos / scale; 
vec2 derivative = fwidth(coord);
vec2 grid = abs(fract(coord - 0.5) - 0.5) / (derivative * thickness);
float line = min(grid.x, grid.y);
return 1.0 - clamp(line, 0.0, 1.0);
}

void main() {
float t = -vNearPoint.y / (vFarPoint.y - vNearPoint.y);
if (t <= 0.0) discard;
vec3 worldPos = vNearPoint + t * (vFarPoint - vNearPoint);

// BACKGROUND: The horizon color you picked (0.69, 0.73, 0.75)
vec3 bg = vec3(0.69, 0.73, 0.75);

// GREYISH GRID COLORS:
// Minor: Only slightly darker than background
// Major: A medium neutral grey
vec3 minorColor = vec3(0.42, 0.45, 0.48); 
vec3 majorColor = vec3(0.30, 0.33, 0.36); 

float minorGrid = getGrid(worldPos.xz, 1.0, 0.75);
float majorGrid = getGrid(worldPos.xz, 10.0, 1.25);

float fogDensity = 0.012;
float dist = length(worldPos - Camera.Position.xyz);
float fade = exp(-fogDensity * dist);

// Blend major over minor
vec3 gridRGB = mix(minorColor, majorColor, majorGrid);
float gridMask = max(minorGrid, majorGrid);

// Final mix with background
vec3 color = mix(bg, gridRGB, gridMask * fade);

FragColor = vec4(color, fade);
if (fade < 0.01) discard;
}
)";

const char* MeshVS = R"(
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
)";

const char* MeshFS = R"(
#version 460 core

layout( std140, binding = 0 ) uniform CameraBlock 
{
mat4 ViewMatrix;
mat4 ProjectionMatrix;
vec4 Position;
} Camera;

// Varyings
in vec3 vWorldPosition;
in vec3 vWorldNormal;
in vec4 vColor;

// Output
out vec4 fColor;

// --- AMBIENT SETTINGS ---
const float AMBIENT_STRENGTH = 0.1;
const vec3 AMBIENT_COLOR = vec3(1.0, 1.0, 1.0 );

// --- LIGHTING CONSTANTS (World Space) ---
const vec3 KEY_DIR = normalize(vec3(0.5, 1.0, 0.5)); 
const vec3 KEY_COLOR = vec3(1.0, 0.95, 0.8);
const float KEY_INTENSITY = 1.0;

const vec3 FILL_DIR = normalize(vec3(-0.5, 0.2, 0.5)); 
const vec3 FILL_COLOR = vec3(0.6, 0.7, 0.9);
const float FILL_INTENSITY = 0.4;

const vec3 RIM_DIR = normalize(vec3(0.0, 0.5, -1.0)); 
const vec3 RIM_COLOR  = vec3(1.0, 1.0, 1.0);
const float RIM_INTENSITY = 0.6;

vec3 CalculateLight(vec3 Normal, vec3 ViewDir, vec3 LightDir, vec3 LightColor, float Intensity) 
{
    // Diffuse
    float Diffuse = max( dot( Normal, LightDir ), 0.0 );
    
    // Specular (Blinn-Phong)
    vec3 HalfwayDir = normalize( LightDir + ViewDir );
    float Specular = pow( max( dot( Normal, HalfwayDir), 0.0 ), 16.0 );
    
    vec3 DiffuseColor = Diffuse * LightColor * Intensity;
    vec3 SpecularColor = Specular * LightColor * Intensity * 0.0;
    
    return DiffuseColor + SpecularColor;
}

void main()
{
	// Ambient
    vec3 Result = AMBIENT_STRENGTH * AMBIENT_COLOR;

    // Diffuse + Specular 
    vec3 WorldNormal = normalize( vWorldNormal );
    vec3 ViewDirection = normalize( Camera.Position.xyz - vWorldPosition );
    Result += CalculateLight(WorldNormal, ViewDirection, KEY_DIR, KEY_COLOR, KEY_INTENSITY);
    Result += CalculateLight(WorldNormal, ViewDirection, FILL_DIR, FILL_COLOR, FILL_INTENSITY);
    Result += CalculateLight(WorldNormal, ViewDirection, RIM_DIR, RIM_COLOR, RIM_INTENSITY);

    vec3 FinalColor = Result * vColor.rgb;
    //FinalColor = pow( FinalColor, vec3(1.0 / 2.2) ); 
   
    // Result
    fColor = vec4(FinalColor, vColor.a);
}
)";

//--------------------------------------------------------------------------------------------------
// VsShaderLibrary
//--------------------------------------------------------------------------------------------------
void vsLoadShaderLibrary()
	{
    VS_ASSERT( !VsShaderLibrary::BackgroundShader );
    VsShaderLibrary::BackgroundShader = new VsShader( SuperTriangleVS, BackgroundFS );
	VS_ASSERT( !VsShaderLibrary::GridShader );
	VsShaderLibrary::GridShader = new VsShader( GridVS, GridFS );
	VS_ASSERT( !VsShaderLibrary::MeshShader );
	VsShaderLibrary::MeshShader = new VsShader( MeshVS, MeshFS );
	}


//--------------------------------------------------------------------------------------------------
void vsUnloadShaderLibrary()
	{
	VS_ASSERT( VsShaderLibrary::MeshShader );
	delete VsShaderLibrary::MeshShader;
	VsShaderLibrary::MeshShader = nullptr;

    VS_ASSERT( VsShaderLibrary::GridShader );
	delete VsShaderLibrary::GridShader;
	VsShaderLibrary::GridShader = nullptr;

    VS_ASSERT( VsShaderLibrary::BackgroundShader );
    delete VsShaderLibrary::BackgroundShader;
    VsShaderLibrary::BackgroundShader = nullptr;
	}
