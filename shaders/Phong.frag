#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  mediump vec3 l_vector = u_light_pos.xyz - v_position.xyz;
  mediump vec3 v_vector = u_cam_pos.xyz - v_position.xyz;
  mediump vec3 h = (v_vector.xyz + l_vector.xyz) / length(v_vector.xyz + l_vector.xyz);

  mediump vec3 light = u_light_intensity / length(l_vector) / length(l_vector);
  mediump float costerm = max(0.0, dot(v_normal.xyz, normalize(h.xyz)));
  mediump float inputterm = max(0.0, dot(v_normal.xyz, normalize(l_vector.xyz)));

  mediump float k_a = 0.2; // Ambient coef
  mediump float k_d = 0.4; // Diffuse coef
  mediump float k_s = 1; // Specular coef
  mediump float p = 64; // Higher p = sharper drop off on reflection
  
  out_color.xyz = k_a * normalize(u_light_intensity) + k_d * light * inputterm + k_s * light * pow(costerm, p);

  // YELLOW FILTER
  out_color.x = min(out_color.x * 3, 1);
  out_color.y = min(out_color.y * 3, 1);
  out_color.z = 0;
  out_color.a = 1;
}

