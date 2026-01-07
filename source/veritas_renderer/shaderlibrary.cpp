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
    // 1. Define your colors (Muted, professional palette)
    // Zenith is the top of the sky, Horizon is the bottom
    vec3 ZenithColor  = vec3( 0.05, 0.15, 0.30 ); // Deep, dark blue
    vec3 HorizonColor = vec3( 0.40, 0.50, 0.60 ); // Desaturated grey-blue

    // 2. Adjust the 'bias' of the gradient
    // Using pow( vUV.y, 0.8 ) makes the horizon feel 'wider' and more natural
    float Weight = pow( clamp( vUV.y, 0.0, 1.0 ), 0.8 );

    // 3. Mix the colors based on vertical position
    vec3 Sky = mix( HorizonColor, ZenithColor, Weight );

    // 4. Subtle Dithering
    // This adds a tiny amount of noise to break up color banding
    float Noise = fract( sin( dot( vUV, vec2( 12.9898, 78.233 ) ) ) * 43758.5453 );
    Sky += ( Noise - 0.5 ) * ( 1.0 / 255.0 );

    FragColor = vec4( Sky, 1.0 );
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

layout( location = 0 ) out vec3 vWorldPos;

void main()
{
    vec2 UV = vec2( ( gl_VertexID << 1 ) & 2, gl_VertexID & 2 );
    vec2 Position = 2.0f * UV - 1.0;

    float GridScale = 1000.0; 
    vWorldPos = vec3( Position.x * GridScale, 0.0, Position.y * GridScale );
    
    gl_Position = Camera.ProjectionMatrix * Camera.ViewMatrix * vec4( vWorldPos, 1.0 );
} 
)";

const char* GridFS = R"(
#version 460 core

// Uniforms
layout( std140, binding = 0 ) uniform CameraBlock 
{
    mat4 ViewMatrix;
    mat4 ProjectionMatrix;
    vec4 Position;
} Camera;

// Varying
layout( location = 0 ) in vec3 vWorldPos;

// Output
layout( location = 0 ) out vec4 FragColor;


float calculateGrid( vec2 planePos, float scale )
{
    vec2 coord = planePos * scale;
    vec2 derivative = fwidth( coord );
    vec2 grid = abs( fract( coord - 0.5 ) - 0.5 ) / derivative;
    float line = min( grid.x, grid.y );
    return 1.0 - min( line, 1.0 );
}

void main()
{
    float dist = length( vWorldPos - Camera.Position.xyz );
    
    float thickLine = calculateGrid( vWorldPos.xz, 0.1 );
    float thinLine  = calculateGrid( vWorldPos.xz, 1.0 );
    
    float xAxis = 1.0 - smoothstep( 0.0, fwidth( vWorldPos.z ) * 2.5, abs( vWorldPos.z ) );
    float zAxis = 1.0 - smoothstep( 0.0, fwidth( vWorldPos.x ) * 2.5, abs( vWorldPos.x ) );

    vec4 color = vec4( 0.2, 0.2, 0.2, thinLine * 0.3 );
    color = mix( color, vec4( 0.3, 0.3, 0.3, 1.0 ), thickLine * 0.6 );
    
    color = mix( color, vec4( 0.8, 0.1, 0.1, 1.0 ), xAxis );
    color = mix( color, vec4( 0.1, 0.1, 0.8, 1.0 ), zAxis );

    float fade = exp( -0.005 * dist );
    
    FragColor = color;
    FragColor.a *= fade;

    if ( FragColor.a < 0.01 ) 
    {
        discard;
    }
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
	}


//--------------------------------------------------------------------------------------------------
void vsUnloadShaderLibrary()
	{
    VS_ASSERT( VsShaderLibrary::GridShader );
	delete VsShaderLibrary::GridShader;
	VsShaderLibrary::GridShader = nullptr;

    VS_ASSERT( VsShaderLibrary::BackgroundShader );
    delete VsShaderLibrary::BackgroundShader;
    VsShaderLibrary::BackgroundShader = nullptr;
	}
