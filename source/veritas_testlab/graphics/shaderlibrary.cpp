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
const char* SuperTriangleVertexShaderSource = 
R"(
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
const char* BackgroundFragmentShaderSource = 
R"(
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
// VsShaderLibrary
//--------------------------------------------------------------------------------------------------
void vsLoadShaderLibrary()
	{
    VS_ASSERT( !VsShaderLibrary::BackgroundShader );
    VsShaderLibrary::BackgroundShader = new VsShader( SuperTriangleVertexShaderSource, BackgroundFragmentShaderSource );
	}


//--------------------------------------------------------------------------------------------------
void vsUnloadShaderLibrary()
	{
    delete VsShaderLibrary::BackgroundShader;
    VsShaderLibrary::BackgroundShader = nullptr;
	}
