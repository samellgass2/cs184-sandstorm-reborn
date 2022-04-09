#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
  mediump vec3 b = cross(v_normal.xyz, v_tangent.xyz);
  mediump mat3 TBN = mat3(v_tangent.xyz, b, v_normal.xyz);

  // Height constant controls prevalence of bumps
  mediump float height_constant = 30;
  mediump float dU = height_constant * (h(vec2(v_uv.x + (1/u_texture_2_size.x), v_uv.y)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  mediump float dV = height_constant * (h(vec2(v_uv.x, v_uv.y + (1/u_texture_2_size.y))) - h(v_uv)) * u_height_scaling * u_normal_scaling;

  mediump vec3 local_norm = vec3(-dU, -dV, 1);
  mediump vec3 n_d = TBN * local_norm;
  
  // PHONG TIME
  mediump vec3 l_vector = u_light_pos.xyz - v_position.xyz;
  mediump vec3 v_vector = u_cam_pos.xyz - v_position.xyz;
  mediump vec3 h = (v_vector.xyz + l_vector.xyz) / length(v_vector.xyz + l_vector.xyz);

  mediump vec3 light = u_light_intensity / length(l_vector) / length(l_vector);
  mediump float costerm = max(0.0, dot(n_d.xyz, normalize(h.xyz)));
  mediump float inputterm = max(0.0, dot(n_d.xyz, normalize(l_vector.xyz)));

  mediump float k_a = 0.2; // Ambient coef
  mediump float k_d = 0.4; // Diffuse coef
  mediump float k_s = 1; // Specular coef
  mediump float p = 64; // Higher p = sharper drop off on reflection

  out_color.xyz = k_a * vec3(1,1,1) + k_d * light * inputterm + k_s * light * pow(costerm, p);
  out_color.a = 1;
}

