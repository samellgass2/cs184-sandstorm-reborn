//
// Created by Nima Rezaeian on 4/12/22.
//

#ifndef CLOTHSIM_SAND_PARTICLE_H
#define CLOTHSIM_SAND_PARTICLE_H


#include "collision/sphere.h"
#include "misc/sphere_drawing.h"

class SandParticle: public Sphere {
public:
    SandParticle(const Vector3D &origin, double radius, double friction, int numLat = 40, int numLon = 40)
    : Sphere(origin, radius, friction, numLat, numLon), position(origin),
      last_position(origin) {}

    Vector3D normal();
    Vector3D velocity(double delta_t) {
      return (position - last_position) / delta_t;
    }

    // static values

    // dynamic values
    Vector3D position = origin;
    Vector3D last_position;
    Vector3D forces;

    double hash;

    Misc::SphereMesh m_sphere_mesh;
};


#endif //CLOTHSIM_SAND_PARTICLE_H
