//
// Created by Nima Rezaeian on 4/12/22.
//

#include "sandbox.h"

#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Sandbox::Sandbox(Vector3D top_left, Vector3D bottom_right, int num_sand_particles, double sand_radius) {
  this->top_left = top_left;
  this->bottom_right = bottom_right;
  this->num_sand_particles = num_sand_particles;
  this->sand_radius = sand_radius;

  generate_particles();
}


Sandbox::~Sandbox() {
  sand_particles.clear();
}

void Sandbox::generate_particles() {
  //Currently just initializing
  Vector3D size = bottom_right - top_left;
  for (int i = 0; i < num_sand_particles; i++) {
    double x = ((float) rand() / RAND_MAX) * abs(size.x);
    double y = ((float) rand() / RAND_MAX) * abs(size.y);
    double z = ((float) rand() / RAND_MAX) * abs(size.z);
    sand_particles.emplace_back(top_left + Vector3D(x, y, z), sand_radius, 0.4);
  }


//  for (int x = 0; x < num_sand_particles; x++) {
//    for (int y = 0; y < num_sand_particles; y++) {
//      for (int z = 0; z < num_sand_particles; z++) {
//        sand_particles.emplace_back(top_left + Vector3D(x * abs(size.x), y * abs(size.y), z * abs(size.z)), sand_radius, 0.4);
//      }
//    }
//  }
}

void Sandbox::simulate(double frames_per_sec, double simulation_steps, SandParameters *sp,
              vector<Vector3D> external_accelerations,
              vector<CollisionObject *> *collision_objects) {
  double mass = 1;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  // Add in external accelerations
  Vector3D total_external_force;
  for (Vector3D acc: external_accelerations) {
    total_external_force += mass*acc;
  }

  for (SandParticle &sp: sand_particles) {
    sp.forces += total_external_force;
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (SandParticle &sp: sand_particles) {
    Vector3D x_t = sp.position + (sp.position - sp.last_position) + (sp.forces / mass) * delta_t * delta_t;
    sp.last_position = sp.position;
    sp.position = x_t;
  }
}


float Sandbox::hash_position(Vector3D pos) {
  Vector3D size = bottom_right - top_left;
  float w = size.x / (2 * sand_radius);
  float h = size.y / (2 * sand_radius);
  float d = size.z / (2 * sand_radius);

  int xpos = floor(pos.x/ w);
  int ypos = floor(pos.y/ h);
  int zpos = floor(pos.z/ d);

  return xpos * w * w + ypos * h + zpos;
}

void Sandbox::reset() {
  SandParticle *sp = &sand_particles[0];
  for (int i = 0; i < sand_particles.size(); i++) {
    sp->position = sp->origin;
    sp->last_position = sp->origin;
    sp++;
  }
}


void Sandbox::buildBoxMesh() {
  // TODO
}
