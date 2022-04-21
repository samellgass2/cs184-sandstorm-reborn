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

Sandbox::Sandbox(Vector3D top_left, Vector3D bottom_right, int num_sand_particles, double sand_radius, double mu) {
  this->top_left = top_left;
  this->bottom_right = bottom_right;
  this->num_sand_particles = num_sand_particles;
  this->sand_radius = sand_radius;
  this->mu = mu;

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
    sand_particles.emplace_back(top_left + Vector3D(x, y, z), sand_radius, mu);
  }
}

void Sandbox::simulate(double frames_per_sec, double simulation_steps, SandParameters *sp,
              vector<Vector3D> external_accelerations,
              vector<CollisionObject *> *collision_objects) {
  Vector3D gravity = external_accelerations[0];
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

  // Inter collision
  build_spatial_map();
  for (SandParticle& particle : sand_particles) {
      inter_collide(particle);
  }
  for (SandParticle &particle: sand_particles) {
    inter_collide_forces(particle, sp, delta_t, simulation_steps);
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (SandParticle &sp: sand_particles) {
    Vector3D x_t = sp.position + (sp.position - sp.last_position) + (sp.forces / mass) * delta_t * delta_t;
    sp.last_position = sp.position;
    sp.position = x_t;
  }

  for (SandParticle &sp : sand_particles) {
    for (CollisionObject * cobj : *collision_objects) {
      cobj->collide(sp);
    }
  }


  for (SandParticle &sp: sand_particles) {
    sp.forces = 0;
  }
}

void Sandbox::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  for (int i = 0; i < sand_particles.size(); i++) {
    float hash = hash_position(sand_particles[i].position);
    if (map.find(hash) != map.end()) {
      map[hash]->push_back(&sand_particles[i]);
    } else {
      map[hash] = new vector<SandParticle *>();
      map[hash]->push_back(&sand_particles[i]);
    }
  }

}

float Sandbox::hash_position(Vector3D pos) {
  Vector3D size = bottom_right - top_left;
  double cube_root = cbrt(num_sand_particles);
  float w = size.x / cube_root;
  float h = size.y / cube_root;
  float d = size.z / cube_root;

  float t = max(h, w);

  int xpos = floor(pos.x/ w);
  int ypos = floor(pos.y/ h);
  int zpos = floor(pos.z/ d);

  return xpos * w * w + ypos * t + zpos;
}

bool collisionsContains(vector<pair<SandParticle*, Vector3D>>& collisions, SandParticle* particle) {
    for (auto pair : collisions) {
        if (pair.first == particle) {
            return true;
        }
    }
    return false;
}

Vector3D lookupCollisions(vector<pair<SandParticle*, Vector3D>>& collisions, SandParticle* particle) {
    for (auto pair : collisions) {
        if (pair.first == particle) {
            return pair.second;
        }
    }
    return 0;
}

// Precondition: sphere r(eference) has collided with sphere m(oving). 
//Returns the *relative* position on r where the collision occured.
// r_p : position of r
// r_v : direction of velocity
// https://math.stackexchange.com/questions/3998455/how-to-find-the-intersection-point-of-two-moving-spheres
Vector3D movingSpheresIntersection(Vector3D r_p, Vector3D r_d, Vector3D m_p, Vector3D m_d, double sand_radius) {
    Vector3D pos = m_p - r_p;
    Vector3D dir = m_d - r_d;
    double t = (-dot(pos, dir) - pow(pow(2 * sand_radius, 2) * dir.norm2() - pow(pos.x * dir.y - pos.y * dir.x, 2) - pow(pos.x * dir.z - pos.z * dir.x, 2) - pow(pos.y * dir.z - pos.z * dir.y, 2), 0.5)) / dir.norm2();
    
    pos = pos + t * dir;
    pos /= 2;
    return pos;
}

// Update collisons attribute of SandParticles. 
void Sandbox::inter_collide(SandParticle& particle) {
    // Remove old collisions.
    //double sand_radius = sand_radius;
    //Vector3D pp = particle.position;
    //particle.collisions.erase(std::remove_if(particle.collisions.begin(), particle.collisions.end(), [sand_radius, particle](auto pair) {
    //    return 2 * sand_radius - (pair.first->position - particle.position).norm() <= 0;
    //    }), particle.collisions.end());

    //if (pp.x != particle.position.x || pp.y != particle.position.y || pp.z != particle.position.z) {
    //    cout << "hi";
    //}
    std::vector<pair<SandParticle*, Vector3D>> out;
    for (auto&& pair : particle.collisions) {
        if (2 * sand_radius - (pair.first->position - particle.position).norm() > 0) {
            out.push_back(pair);
        }
    }
    particle.collisions = out;

    // Check for
    vector<SandParticle*> candidates = *(map[hash_position(particle.position)]);
    for (SandParticle* cand : candidates) {
        if (cand == &particle) {
            continue;
        }
        if (!collisionsContains(particle.collisions, cand) && 2 * sand_radius - (cand->position - particle.position).norm() > 0) {
            particle.collisions.push_back(pair<SandParticle*, Vector3D>(cand, movingSpheresIntersection(particle.position, particle.velocity(1), cand->position, cand->velocity(1), sand_radius)));
        }
    }
}


void Sandbox::inter_collide_forces(SandParticle &particle, SandParameters *sp, double delta_t, double simulation_steps) {
  vector<SandParticle *> candidates = *(map[hash_position(particle.position)]);
  double xi;
  double xi_dot;
  Vector3D N;
  Vector3D V;
  Vector3D V_t;
  double f_n;
  for (SandParticle *cand: candidates) {
    if (cand != &particle) {
      N = cand->position - particle.position;
      xi = max(0.0, 2 * sand_radius - N.norm());
      N.normalize();
      V = particle.velocity(delta_t) - cand->velocity(delta_t);
      xi_dot = dot(N, V);
      V_t = V - xi_dot * N;
      f_n = sp->k_d * pow(xi, sp->alpha) * xi_dot + sp->k_r * pow(xi, sp->beta);
      particle.forces += -f_n * N;
    }
  }
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
