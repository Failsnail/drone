#version 140

in vec3 position;
in vec3 color;

uniform mat4 object_to_camera;
uniform mat4 projection;

out vec3 v_tex_color;

void main() {
     gl_Position = projection * object_to_camera * vec4(position, 1.0);
     v_tex_color = color;
}