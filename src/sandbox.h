//
// Created by Nima Rezaeian on 4/12/22.
//

#ifndef CLOTHSIM_SANDBOX_H
#define CLOTHSIM_SANDBOX_H


#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "collision/collisionObject.h"
#include "sand_particle.h"

struct SandParameters {
    SandParameters() {}
    SandParameters(double alpha,
                    double beta,
                    double k_d, double k_r,
                    double mass)
            : alpha(alpha),
              beta(beta),
              k_d(k_d), k_r(k_r) {}
    ~SandParameters() {}

    // Global simulation parameters
    double alpha;
    double beta;
    double k_d;
    double k_r;
};


struct Sandbox {
    Sandbox() {}
    Sandbox(Vector3D top_left, Vector3D bottom_right, int num_sand_particles, double sand_radius);
    ~Sandbox();

    void generate_particles();

    void simulate(double frames_per_sec, double simulation_steps, SandParameters *cp,
                  vector<Vector3D> external_accelerations,
                  vector<CollisionObject *> *collision_objects);

    void reset();
    void buildBoxMesh();

    void build_spatial_map();
    void self_collide(SandParticle &pm, double simulation_steps);
    float hash_position(Vector3D pos);

    // Cloth properties
    Vector3D top_left;
    Vector3D bottom_right;
    int num_sand_particles;
    double sand_radius;

    // Sandbox components
    vector<SandParticle> sand_particles;

    // Spatial hashing
    unordered_map<float, vector<PointMass *> *> map;
};


#endif //CLOTHSIM_SANDBOX_H
