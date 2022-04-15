#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  double dx = width / num_width_points;
  double dy = height / num_height_points;

  if (orientation == HORIZONTAL) {
    for (int yoffset = 0; yoffset < num_width_points; yoffset++) {
      for (int xoffset = 0; xoffset < num_height_points; xoffset++) {
        Vector3D position = Vector3D(dx * xoffset, 1, dy * yoffset);
        PointMass newmass = PointMass(position, false);
        point_masses.emplace_back(newmass);
      }
    }

  } else {
    for (int yoffset = 0; yoffset < num_height_points; yoffset++) {
      for (int xoffset = 0; xoffset < num_width_points; xoffset++) {
        double zrand = ((double) rand() / RAND_MAX) * 0.002 - 0.001;
        Vector3D position = Vector3D(dx * xoffset,  dy * yoffset, zrand);
        PointMass newmass = PointMass(position, false);
        point_masses.emplace_back(newmass);
      }
    }
  }

  // Pinning relevant points
  for (int ind = 0; ind < pinned.size(); ind++) {
    point_masses[num_width_points*pinned[ind][1]+pinned[ind][0]].pinned = true;
  }

  // Assigning STRUCTURAL springs
  for (int yind = 0; yind < num_height_points; yind++) {
    for (int xind = 0; xind < num_width_points; xind++) {
      int currind = yind * num_width_points + xind;
      // If there exists a point above currind
      if (yind - 1 >= 0) {
        Spring newstrucspr = Spring(&(point_masses[currind]), &(point_masses[currind - num_width_points]), STRUCTURAL);
        springs.emplace_back(newstrucspr);
      }
      // If there exists a point to the left of currind
      if (xind - 1 >= 0) {
        Spring newstrucspr = Spring(&(point_masses[currind]), &(point_masses[currind - 1]), STRUCTURAL);
        springs.emplace_back(newstrucspr);
      }
    }
  }

  // Assigning SHEARING springs
  for (int yind = 0; yind < num_height_points; yind++) {
    for (int xind = 0; xind < num_width_points; xind++) {
      int currind = yind * num_width_points + xind;
      // If there exists a point up one and one to the left
      if (yind - 1 >= 0 && xind - 1 >= 0) {
        Spring newshearspr = Spring(&(point_masses[currind]),
                                    &(point_masses[currind - num_width_points - 1]), SHEARING);
        springs.emplace_back(newshearspr);
      }
      // If there exists a point up one and one to the right
      if (yind - 1 >= 0 && xind + 1 < num_width_points) {
        Spring newshearspr = Spring(&(point_masses[currind]),
                                    &(point_masses[currind - num_width_points + 1]), SHEARING);
        springs.emplace_back(newshearspr);
      }
    }
  }

  // Assigning BENDING springs
  for (int yind = 0; yind < num_height_points; yind++) {
    for (int xind = 0; xind < num_width_points; xind++) {
      int currind = yind * num_width_points + xind;
      // If there exists a point two above curr
      if (yind - 2 >= 0) {
        Spring newbendspr = Spring(&(point_masses[currind]),
                                   &(point_masses[currind - 2*num_width_points]), BENDING);
        springs.emplace_back(newbendspr);
      }
      // If there exists a point two to the left
      if (xind - 2 >= 0) {
        Spring newbendspr = Spring(&(point_masses[currind]),
                                   &(point_masses[currind - 2]), BENDING);
        springs.emplace_back(newbendspr);
      }

    }
  }


}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  // Add in external accelerations
  Vector3D total_external_force;
  for (Vector3D acc: external_accelerations) {
    total_external_force += mass*acc;
  }

  for (PointMass &pm: point_masses) {
    pm.forces += total_external_force;
  }

  for (Spring &sp: springs) {
    double local_ks = 0;
    switch (sp.spring_type) {
      case CGL::STRUCTURAL:
        if (cp->enable_structural_constraints) {
          local_ks = cp->ks;
        }
      case CGL::SHEARING:
        if (cp->enable_shearing_constraints) {
          local_ks = cp->ks;
        }
      case CGL::BENDING:
        if (cp->enable_bending_constraints) {
          local_ks = 0.2 * cp->ks;
        }
    }
    if (local_ks != 0) {
      Vector3D diff = (sp.pm_b->position - sp.pm_a->position);
      double F_s = local_ks * (diff.norm() - sp.rest_length);
      diff.normalize();
      sp.pm_a->forces += F_s * diff;
      sp.pm_b->forces += -F_s * diff;
    }
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (PointMass &pm: point_masses) {
    if (!pm.pinned) {
      Vector3D x_t = pm.position + (1-cp->damping/100) * (pm.position - pm.last_position) + (pm.forces / mass) * delta_t * delta_t;
      pm.last_position = pm.position;
      pm.position = x_t;
    }
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (int i = 0; i < point_masses.size(); i++) {
    self_collide(point_masses[i], simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring &sp: springs) {
    Vector3D diff = sp.pm_b->position - sp.pm_a->position;
    double correction = diff.norm() - 1.1 * sp.rest_length;
    if (correction > 0) {
      if (!sp.pm_a->pinned && !sp.pm_b->pinned) {
        sp.pm_a->position += correction/2 * diff;
        sp.pm_b->position += -correction/2 * diff;
      } else if (sp.pm_a->pinned) {
        sp.pm_b->position += -correction * diff;
      } else if (sp.pm_b->pinned) {
        sp.pm_a->position += correction * diff;
      }
    }
  }

  for (PointMass &pm: point_masses) {
    pm.forces = 0;
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  for (int i = 0; i < point_masses.size(); i++) {
    float hash = hash_position(point_masses[i].position);
    if (map.find(hash) != map.end()) {
      map[hash]->push_back(&point_masses[i]);
    } else {
      map[hash] = new vector<PointMass *>();
      map[hash]->push_back(&point_masses[i]);
    }
  }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

  vector<PointMass *> candidates = *(map[hash_position(pm.position)]);
  Vector3D correction_vec = Vector3D(0);
  int num_corrs = 0;
  for (int i = 0; i < candidates.size(); i++) {
    double dist = (pm.position - candidates[i]->position).norm();
    //if not self and too close
    if (&pm != candidates[i] && dist < 2*thickness) {
      Vector3D corr_dir = (pm.position - candidates[i]->position).unit();
      num_corrs += 1;
      correction_vec += (corr_dir) * (2*thickness - dist);
    }
  }

  //final step : if we made corrections, average correction
  if (num_corrs > 0) {
    pm.position += correction_vec / num_corrs / simulation_steps;
  }

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  float w = 3 * width / num_width_points;
  float h = 3 * height / num_height_points;
  float t = max(w, h);
  float space = width * height;

  // Runs VERY SLOWLY on this machine for whatever reason... I think too few divisions in space?
//  int xpos = fmod(pos.x, w);
//  int ypos = fmod(pos.y, h);
//  int zpos = fmod(pos.z, t);

  int xpos = floor(pos.x/ w);
  int ypos = floor(pos.y/ h);
  int zpos = floor(pos.z/ t);

  // x * space^2 + y * space + z - should be unique
  return xpos * space * space + ypos * space + zpos;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
