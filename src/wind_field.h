//
// Created by Samuel Ellgass on 4/24/22.
//

#ifndef CLOTHSIM_WIND_FIELD_H
#define CLOTHSIM_WIND_FIELD_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"
#include <vector>
#include "sand_particle.h"

using namespace CGL;
using namespace std;

//TODO: expand functionality of wind_field to account for using x, y, and z for one another
class wind_field {
public:
    wind_field(Vector3D top_left, Vector3D bottom_right, vector<double> xcoefficients, vector<double> ycoefficients, vector<double> zcoefficients);
    wind_field() {};
    Vector3D top_left;
    Vector3D bottom_right;
    vector<double> xcoefficients;
    vector<double> ycoefficients;
    vector<double> zcoefficients;

    bool is_cyclone;
    double radius;
    double a, b, magnitude;

    Vector3D wind_force(SandParticle &particle);
    Vector3D cyclone_force(SandParticle &particle);
    bool in_bounds(Vector3D &position);

};


#endif //CLOTHSIM_WIND_FIELD_H
