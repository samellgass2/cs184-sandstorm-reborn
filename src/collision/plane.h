#ifndef COLLISIONOBJECT_PLANE_H
#define COLLISIONOBJECT_PLANE_H

#include <nanogui/nanogui.h>

#include "collisionObject.h"

using namespace nanogui;
using namespace CGL;
using namespace std;

struct Plane : public CollisionObject {
public:
    Plane(const Vector3D& point, const Vector3D& normal, double friction, double length, double width)
        : point(point), normal(normal.unit()), length(length), width(width), friction(friction)
    {
        _computePoints();
    };

    void render(GLShader &shader);
    void collide(SandParticle &pm);
    void _computePoints();

    Vector3D point;
    Vector3D normal;
    Vector3D botLeft;
    Vector3D topLeft;
    Vector3D botRight;
    Vector3D topRight;

    double friction;
    double length;
    double width;
    bool is_textured;

};

#endif /* COLLISIONOBJECT_PLANE_H */
