
#version 460 core

#define kAntialiasing 2.0

noperspective in vec4 vColor;
noperspective in float vEdgeDist;
noperspective in float vSize;

out vec4 fColor;

void main() 
{
    fColor = vColor;

    float d = abs(vEdgeDist) / vSize;
	d = smoothstep(1.0, 1.0 - (kAntialiasing / vSize), d);
	fColor.a *= d;
    if ( fColor.a  < 0.01 ) 
    {
        discard;
    }
}