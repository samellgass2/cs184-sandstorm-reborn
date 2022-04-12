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

Sandbox::Sandbox(Vector3D top_left, Vector3D bottom_right, int num_sand_particles) {
  this->top_left = top_left;
  this->bottom_right = bottom_right;
  this->num_sand_particles = num_sand_particles;

  generate_particles();
}


Sandbox::~Sandbox() {
  sand_particles.clear();
}

void Sandbox::generate_particles() {
  Vector3D size = bottom_right - top_left;
  for (int i = 0; i < num_sand_particles; i++) {
    double x = (rand() / RAND_MAX) * size.x;
    double y = (rand() / RAND_MAX) * size.y;
    double z = (rand() / RAND_MAX) * size.z;
    sand_particles.emplace_back(top_left + Vector3D(x, y, z), sand_radius, 0.4);
  }
}

void Sandbox::simulate(double frames_per_sec, double simulation_steps, SandParameters *cp,
              vector<Vector3D> external_accelerations,
              vector<CollisionObject *> *collision_objects) {
  // TODO
}


float Sandbox::hash_position(Vector3D pos) {
  return 0.f;
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
