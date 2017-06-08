#version 140

in vec3 v_tex_color;

out vec4 color;

void main() {
     color = vec4(v_tex_color, 1.0);
}
