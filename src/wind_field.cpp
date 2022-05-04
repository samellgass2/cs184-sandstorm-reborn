//
// Created by Samuel Ellgass on 4/24/22.
//

#include "wind_field.h"
#include "sandSimulator.h"
#include <cmath>

wind_field::wind_field(Vector3D top_left, Vector3D bottom_right, vector<double> xcoefficients, vector<double> ycoefficients, vector<double> zcoefficients) {
  this->top_left = top_left;
  this->bottom_right = bottom_right;
  this->xcoefficients = xcoefficients;
  this->ycoefficients = ycoefficients;
  this->zcoefficients = zcoefficients;
  this->is_cyclone = false;

}

Vector3D wind_field::wind_force(SandParticle &particle) {
  // Computes wind_force as coef[0] * 1 + coef[1] * x + coef[2] * x^2 etc. for x, y, z.
  Vector3D position = particle.position;
  if (in_bounds(position)) {
    if (is_cyclone) {
      return cyclone_force(particle);
    }
    double x = 0;
    double y = 0;
    double z = 0;
    double curr_x = 1;
    double curr_y = 1;
    double curr_z = 1;
    for (double coef : xcoefficients) {
      x += coef * curr_x;
      curr_x *= position.x;
    }
    for (double coef : ycoefficients) {
      y += coef * curr_y;
      curr_y *= position.y;
    }
    for (double coef : zcoefficients) {
      z += coef * curr_z;
      curr_z *= position.z;
    }
    return Vector3D(x, y, z);

  }
  return Vector3D(0.0);

};

bool wind_field::in_bounds(Vector3D &position) {
  return (position.x < top_left.x && position.y < top_left.y && position.z < top_left.z &&
  position.x > bottom_right.x && position.y > bottom_right.y && position.z > bottom_right.z);
}

Vector3D wind_field::cyclone_force(SandParticle &particle) {
  Vector3D position = particle.position;
  double new_x;
  double new_z;
  double local_radius = radius * pow(std::min(position.z + 0.5, (double) 2.0), 2) / 4.0f;

  double start_x = clamp(position.x, -local_radius + a, local_radius - a);
  double start_z = clamp(position.z, -local_radius + b, local_radius - b);
  bool inside_cylinder = (start_x == position.x && start_z == position.z);


  new_x = sqrt(std::max(local_radius * local_radius - pow((start_z + b), 2), (double) 0)) - a;

  new_z = sqrt(std::max(local_radius * local_radius - pow((start_x + a), 2), (double) 0)) - b;

  double spin_force = magnitude / 4;

  Vector3D correction_vec;
  if (!inside_cylinder) {
    correction_vec = (Vector3D(new_x, position.y, new_z) - position).unit();
    correction_vec *= magnitude;
    correction_vec.x += spin_force;
    correction_vec.z += spin_force;
  } else {
    double dist_to_center = sqrt(pow(position.x - a,2) + pow(position.z - b,2));
    double cylinder_force = 10 * (1 - pow((dist_to_center) / local_radius, 10));
    //std::cout << "x: " << position.x << "y: " << position.y << "z: " << position.z << "bool:" << inside_cylinder << ::endl;
    correction_vec = Vector3D(cylinder_force, 0, cylinder_force);

  }
  // If the particle is inside the cylinder or within one local_radius, then turn off k_d, k_t forces
  inside_cylinder = inside_cylinder || ((Vector3D(new_x, position.y, new_z) - position).norm() < local_radius);
  if (particle.inside_cyclone != inside_cylinder) {
    particle.inside_cyclone = inside_cylinder;
  }

  correction_vec.y = 9.8;

  return correction_vec;
}
