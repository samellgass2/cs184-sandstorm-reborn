#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../sandSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.00001

void Plane::collide(SandParticle &particle) {
  // TODO (Part 3): Handle collisions with planes.
  Vector3D prev_edge, curr_edge;
  double prev_dist = dot(normal, particle.last_position - point);
  // Particles are actually closer to the plane
  if (prev_dist < 0) {
    prev_dist += particle.radius;
  } else {
    prev_dist -= particle.radius;
  }
  double curr_dist = dot(normal, particle.position - point);
  if (curr_dist < 0) {
    curr_dist += particle.radius;
  } else {
    curr_dist -= particle.radius;
  }

  if (curr_dist * prev_dist <= 0) {
    Vector3D tang_point = particle.position - normal * curr_dist + normal * (particle.radius / 500);
    Vector3D corr_vec = tang_point - particle.last_position;

    particle.position = particle.last_position + (1 - friction)*corr_vec;
//    particle.last_position += (1 - friction)*(normal * (sp.radius));
  }



}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  Vector3f botLeft = Vector3f(point.x - (float) length/2, point.y, point.z - (float)width/2);
  Vector3f topLeft = Vector3f(point.x + (float) length/2, point.y, point.z - (float)width/2);
  Vector3f botRight = Vector3f(point.x - (float) length/2, point.y, point.z + (float)width/2);
  Vector3f topRight = Vector3f(point.x + (float) length/2, point.y, point.z + (float)width/2);
  positions.col(0) << topRight;
  positions.col(1) << topLeft;
  positions.col(2) << botRight;
  positions.col(3) << botLeft;

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  MatrixXf sand_mat(4, 4);
  sand_mat << -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;

//  if (shader.attrib("in_is_sand", false) != -1) {
//    shader.uploadAttrib("in_is_sand", sand_mat);
//  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
