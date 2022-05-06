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
#include "wind_field.h"

struct SandParameters {
    SandParameters() {}
    SandParameters(double alpha,
                    double beta,
                    double k_d, double k_r,
                    double mass, double mu, double k_t, double spring_damping)
            : alpha(alpha),
              wind_on(true),
              beta(beta),
              k_d(k_d), k_r(k_r), k_t(k_t), spring_damping(spring_damping), mass(mass) {}
    ~SandParameters() {}

    // Global simulation parameters
    double alpha;
    double beta;
    double k_d;
    double k_r;
    double k_t;
    double spring_damping;
    double mass;
    bool wind_on;
};


struct Sandbox {
    Sandbox() {}
    Sandbox(Vector3D top_left, Vector3D bottom_right, int num_sand_particles, double sand_radius, double mu);
    ~Sandbox();

    void generate_particles();

    void simulate(double frames_per_sec, double simulation_steps, SandParameters *cp,
                  vector<Vector3D> external_accelerations,
                  vector<CollisionObject *> *collision_objects,
                  vector<wind_field *> *wind_fields, int nth);

    void reset();
    void buildBoxMesh();

    void build_spatial_map();
    float hash_position(Vector3D pos);
    void update_collisions(SandParticle& particle);
    void update_forces(SandParticle &particle, SandParameters *sp, double delta_t, double simulation_steps);
    void calculate_wind(vector<wind_field *> *wind_fields, SandParticle& particle, SandParameters *sp);


    // Sandbox properties
    Vector3D top_left;
    Vector3D bottom_right;
    int num_sand_particles;
    double sand_radius;
    double mu;

    // Sandbox components
    vector<SandParticle> sand_particles;

    // Spatial hashing
    unordered_map<float, vector<SandParticle *> *> map;
};


#endif //CLOTHSIM_SANDBOX_H
