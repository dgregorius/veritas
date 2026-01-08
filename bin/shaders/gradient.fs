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