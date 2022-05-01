#include <nanogui/nanogui.h>

#include "../sand_particle.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(SandParticle &sp) {
  // TODO (Part 3): Handle collisions with spheres.
  if ((sp.position - origin).norm() < radius) {
    Vector3D dir_out = (sp.position - origin).unit(); //direction vector from center of sphere to pm.
    Vector3D tangent_point = origin + dir_out * radius;
    Vector3D correction_vec = tangent_point - sp.last_position;

    sp.position = sp.last_position + (1 - friction)*correction_vec;
  }

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92, false, is_textured);
}
