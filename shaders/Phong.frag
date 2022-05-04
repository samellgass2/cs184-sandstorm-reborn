#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;
uniform bool in_is_sand;
uniform bool is_wind;
uniform bool is_skybox;
uniform float brown_tint;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 raw_position;
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
  out_color.a = 1;

  // BROWN TINT
  mediump float r = 78 / 255.0;
  mediump float g = 67 / 255.0;
  mediump float b = 43 / 255.0;

  // YELLOW FILTER
  if (in_is_sand) {
    out_color.r = min(r * brown_tint * 3, 1);
    out_color.g = min(g * brown_tint * 3, 1);
    out_color.b = min(b * brown_tint * 3, 1);
    out_color.a = 1;
  }

  // CUBEMAP TEX
  if (is_skybox) {
    mediump vec3 w_o = normalize(v_position.xyz / v_position.w).xyz - u_cam_pos.xyz;
    out_color.xyz = texture(u_texture_cubemap, w_o).xyz;
  }
  if (is_wind) {
    out_color.x = 0;
    out_color.y = 0;
    out_color.z = 1;
  }
  out_color.a = 1;
}

