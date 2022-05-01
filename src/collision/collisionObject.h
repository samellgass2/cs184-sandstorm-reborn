#ifndef COLLISIONOBJECT
#define COLLISIONOBJECT

#include <nanogui/nanogui.h>

#include "../sand_particle.h"

using namespace CGL;
using namespace std;
using namespace nanogui;

class CollisionObject {
public:
  virtual void render(GLShader &shader) = 0;
  virtual void collide(SandParticle &pm) = 0;
  bool is_textured;

private:
  double friction;
};

#endif /* COLLISIONOBJECT */
