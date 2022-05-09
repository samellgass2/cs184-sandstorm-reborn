//
// Created by Nima Rezaeian on 4/12/22.
//

#ifndef CLOTHSIM_SAND_PARTICLE_H
#define CLOTHSIM_SAND_PARTICLE_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

#include "misc/sphere_drawing.h"

using namespace CGL;

class SandParticle {
public:
    SandParticle(const Vector3D &position, double radius, double friction, int numLat = 5, int numLon = 5)
    : radius(radius), friction(friction), numLat(numLat), numLon(numLon),
    position(position), origin(position), m_sphere_mesh(Misc::SphereMesh(numLat, numLon)),
    last_position(position), velocity(0) {
        brown_tint = ((float)rand() / (RAND_MAX)) * 0.5 + 0.5; // random 0 to 1 inclusive
        inside_cyclone = false;
    }

    Vector3D normal();


    // static values
    double mass;
    bool inside_cyclone;

    // dynamic values
    Vector3D position;
    Vector3D velocity;
    Vector3D last_position;
    Vector3D forces;
    Vector3D origin;
    std::vector<std::pair<SandParticle*, Vector3D>> collisions;

    double hash;
    double radius;
    double numLat;
    double numLon;
    double friction;
    float brown_tint;


    Misc::SphereMesh m_sphere_mesh;
};


#endif //CLOTHSIM_SAND_PARTICLE_H
