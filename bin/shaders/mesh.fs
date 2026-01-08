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