#version 330 core
layout (location = 0) in vec3 pos;

uniform mat4 mvp;
uniform vec4 globalColor;
out vec4 color;

void main()
{
    gl_Position = mvp * vec4(pos, 1.0);
    color = globalColor;
}
