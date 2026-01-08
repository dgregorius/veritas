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
const float AMBIENT_STRENGTH = 1.0;
const vec3 AMBIENT_COLOR = vec3( 0.03, 0.04, 0.05 );

// --- LIGHTING CONSTANTS (World Space) ---
const vec3 KEY_DIR = normalize( vec3( -0.3, -0.2, -1.0 ) ); 
const vec3 KEY_COLOR = vec3(1.0, 0.98, 0.94 );
const float KEY_INTENSITY = 1.0;

const vec3 FILL_DIR = normalize( vec3( 1.0, -0.5, -1.0 ) ); 
const vec3 FILL_COLOR = vec3( 0.4, 0.42, 0.45 );
const float FILL_INTENSITY = 0.4;

const vec3 RIM_DIR = normalize( vec3( -0.0, -2.0, 0.5 ) ); 
const vec3 RIM_COLOR  = vec3( 1.0, 1.0, 1.0 );
const float RIM_INTENSITY = 0.7;

vec3 CalculateLight(vec3 N, vec3 V, vec3 L, vec3 LightColor, float Intensity ) 
{
    // Diffuse
    float Diffuse = max( dot( N, L ), 0.0 );
    
    // Specular (Blinn-Phong)
    vec3 H = normalize( L + V );
    float Specular = pow( max( dot( N, H ), 0.0 ), 16.0 );
    
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
    Result += CalculateLight(WorldNormal, ViewDirection, inverse( mat3( Camera.ViewMatrix ) ) * -KEY_DIR, KEY_COLOR, KEY_INTENSITY);
    Result += CalculateLight(WorldNormal, ViewDirection, inverse( mat3( Camera.ViewMatrix ) ) * -FILL_DIR, FILL_COLOR, FILL_INTENSITY);
    Result += CalculateLight(WorldNormal, ViewDirection, inverse( mat3( Camera.ViewMatrix ) ) * -RIM_DIR, RIM_COLOR, RIM_INTENSITY);

    vec3 FinalColor = Result * vColor.rgb;
    //FinalColor = pow( FinalColor, vec3(1.0 / 2.2) ); 
   
    // Result
    fColor = vec4(FinalColor, vColor.a);
}