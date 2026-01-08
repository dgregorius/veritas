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