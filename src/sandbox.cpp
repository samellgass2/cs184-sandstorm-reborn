//
// Created by Nima Rezaeian on 4/12/22.
//

#include "sandbox.h"
#include <math.h>
#include <vector>
#include "wind_field.h"


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
  Vector3D size = top_left - bottom_right;
  int num_success = 0;
  for (int i = 0; i < num_sand_particles; i++) {
    double x = ((float) rand() / RAND_MAX) * abs(size.x);
    double y = ((float) rand() / RAND_MAX) * abs(size.y);
    double z = ((float) rand() / RAND_MAX) * abs(size.z);
    Vector3D position = Vector3D (bottom_right.x + x, bottom_right.y + y,bottom_right.z + z);
    bool found = false;
    for (SandParticle particle: sand_particles) {
      if ((position - particle.position).norm() <= 2 * sand_radius) {
        found = true;
        break;
      }
    }
    if (!found) {
      sand_particles.emplace_back(position, sand_radius, mu);
      num_success++;
    }
  }
  std::cout << "Generated " << num_success << " particles." << std::endl;
}

void Sandbox::calculate_wind(vector<wind_field *> *wind_fields, SandParticle& particle, SandParameters *sp) {
    for (wind_field * windField : *wind_fields) {
      particle.forces += sp->mass * windField->wind_force(particle.position);
    }
}

void Sandbox::simulate(double frames_per_sec, double simulation_steps, SandParameters *sp,
              vector<Vector3D> external_accelerations,
              vector<CollisionObject *> *collision_objects,
              vector<wind_field *> *wind_fields) {
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  // TODO (Part 2): Compute total force acting on each point mass.
  // Add in external accelerations
  Vector3D total_external_force;
  for (Vector3D acc: external_accelerations) {
    total_external_force += sp->mass*acc;
  }

  for (SandParticle &particle: sand_particles) {
    particle.forces += total_external_force;
  }

  build_spatial_map();
  // Update Collisions
  for (SandParticle& particle : sand_particles) {
      update_collisions(particle);
  }
  // Update Forces
  for (SandParticle &particle: sand_particles) {
      update_forces(particle, sp, delta_t, simulation_steps);
  }
  // Include Wind
  if (sp->wind_on) {
    for (SandParticle &particle: sand_particles) {
      calculate_wind(wind_fields, particle, sp);
    }
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (SandParticle &particle: sand_particles) {
    Vector3D x_t = particle.position + (particle.position - particle.last_position) + (particle.forces / sp->mass) * delta_t * delta_t;
    particle.last_position = particle.position;
    particle.position = x_t;
  }

  for (SandParticle &particle : sand_particles) {
    for (CollisionObject * cobj : *collision_objects) {
      cobj->collide(particle);
    }
  }

  for (SandParticle &particle: sand_particles) {
    particle.forces = 0;
  }
}

void Sandbox::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  double cell_size = 2 * sand_radius;

  for (int i = 0; i < sand_particles.size(); i++) {
    Vector3D particle_position = sand_particles[i].position;
    float hash;
    // Insert particle into all its neighboring hash positions
    for (int dx = -1; dx < 2; dx++) {
      for (int dy = -1; dy < 2; dy++) {
        for (int dz = -1; dz < 2; dz++) {
          hash = hash_position(particle_position + Vector3D(dx * cell_size, dy * cell_size, dz * cell_size));
          auto it = map.find(hash);
          if (it != map.end()) {
            it->second->push_back(&sand_particles[i]);
          } else {
            auto* chain = new vector<SandParticle *>();
            chain->push_back(&sand_particles[i]);
            map.insert(std::make_pair(hash, chain));
          }
        }
      }
    }
  }
}


float Sandbox::hash_position(Vector3D pos) {
  Vector3D size = bottom_right - top_left;
  double cell_size = 2 * sand_radius;
  int w = ceil(size.x / cell_size);
  int h = ceil(size.y / cell_size);

  int t = max(h, w);

  int xpos = floor(pos.x/ cell_size);
  int ypos = floor(pos.y/ cell_size);
  int zpos = floor(pos.z/ cell_size);

  return xpos * t * t + ypos * t + zpos;
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
int movingSpheresIntersection(Vector3D r_p, Vector3D r_d, Vector3D m_p, Vector3D m_d, double sand_radius, Vector3D &result) {
    Vector3D pos = m_p - r_p;
    Vector3D dir = m_d - r_d;
    if (dir.norm2() == 0) {
      // Parallel lines
      return -1;
    }
    double det = 4 * sand_radius * sand_radius * dir.norm2() -
            pow(pos.x * dir.y - pos.y * dir.x, 2) -
            pow(pos.x * dir.z - pos.z * dir.x, 2) -
            pow(pos.y * dir.z - pos.z * dir.y, 2);
    if (det < 0) {
      // Will never intersect
      return -1;
    }
    double first = -dot(pos, dir);
    double t = (first - sqrt(det)) / dir.norm2();
    
    pos = pos + t * dir;
    pos /= 2;
    result = pos;
    return 0;
}

// Update collisions attribute of SandParticles.
void Sandbox::update_collisions(SandParticle& particle) {
    // Find out which collisions are still happening
    std::vector<pair<SandParticle*, Vector3D>> out;
    for (auto&& pair : particle.collisions) {
        if (2 * sand_radius - (pair.first->position - particle.position).norm() > 0) {
            out.push_back(pair);
        }
    }
    particle.collisions = out;

    // Find new collision
    vector<SandParticle*> candidates = *(map[hash_position(particle.position)]);
    for (SandParticle* cand : candidates) {
        if (cand == &particle) {
            continue;
        }
        if (!collisionsContains(particle.collisions, cand) && 2 * sand_radius - (cand->position - particle.position).norm() > 0) {
          Vector3D colPos;
          int ret = movingSpheresIntersection(particle.position, particle.velocity(1), cand->position, cand->velocity(1), sand_radius, colPos);
          if (ret != -1) {
            particle.collisions.push_back(pair<SandParticle*, Vector3D>(cand, colPos));
          }
        }
    }
}


void Sandbox::update_forces(SandParticle &particle, SandParameters *sp, double delta_t, double simulation_steps) {
  // Collision params
  double xi, xi_dot, f_n;
  Vector3D N, V;
  for (auto&& pair : particle.collisions) {
    SandParticle *cand = pair.first;
    N = cand->position - particle.position;
    xi = max(0.0, 2 * sand_radius - N.norm());
    N.normalize();
    V = particle.velocity(delta_t) - cand->velocity(delta_t);
    xi_dot = dot(N, V);
    f_n = sp->k_d * pow(xi, sp->alpha) * xi_dot + sp->k_r * pow(xi, sp->beta);
    particle.forces += -f_n * N;

    Vector3D D = ((cand->position + lookupCollisions(cand->collisions, &particle)) - (particle.position + lookupCollisions(particle.collisions, cand)));
    // Make ito
    if (D.norm() < 0.00000001) {
      continue;
    }
    particle.forces += min(mu * f_n, sp->k_t * D.norm()) * D.unit();
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
