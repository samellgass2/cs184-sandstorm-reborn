#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../sandSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.00001

// stolen from project 3-1
// ray: origin, direction
// triangle: p1, p2, p3
bool triangle_has_intersection(Vector3D& origin, Vector3D& direction, Vector3D& p1, Vector3D& p2, Vector3D& p3) {
	// Part 1, Task 3: implement ray-triangle intersection
	// The difference between this function and the next function is that the next
	// function records the "intersection" while this function only tests whether
	// there is a intersection.

	Vector3D e1 = p2 - p1;
	Vector3D e2 = p3 - p1;
	Vector3D s = origin - p1;
	Vector3D s1 = cross(direction, e2);
	Vector3D s2 = cross(s, e1);

	Vector3D solution = (1 / (dot(s1, e1))) * Vector3D(dot(s2, e2), dot(s1, s), dot(s2, direction));

	double t = solution.x;
	double b1 = solution.y;
	double b2 = solution.z;

	if (t >= 0
		&& 0 <= b1 && b1 <= 1
		&& 0 <= b2 && b2 <= 1
		&& 0 <= 1 - b1 - b2 && 1 - b1 - b2 <= 1
		) {
		return true;
	}
	else {
		return false;
	}


}

void Plane::collide(SandParticle &particle) {
  // TODO (Part 3): Handle collisions with planes.
  Vector3D prev_edge, curr_edge;
  double prev_dist = dot(normal, particle.last_position - point);
  // Particles are actually closer to the plane
  if (prev_dist < 0) {
    prev_dist += particle.radius;
  } else {
    prev_dist -= particle.radius;
  }
  double curr_dist = dot(normal, particle.position - point);
  if (curr_dist < 0) {
    curr_dist += particle.radius;
  } else {
    curr_dist -= particle.radius;
  }
  Vector3D origin(particle.last_position);
  Vector3D direction(particle.position - particle.last_position);
  if (curr_dist * prev_dist <= 0 
	  && (triangle_has_intersection(origin, direction, botLeft, topLeft, botRight) 
	  || triangle_has_intersection(origin, direction, botRight, topLeft, topRight))) {
    Vector3D tang_point = particle.position - normal * curr_dist + normal * (particle.radius / 500);
    Vector3D corr_vec = tang_point - particle.last_position;

    particle.position = particle.last_position + (1 - friction)*corr_vec;
//    particle.last_position += (1 - friction)*(normal * (sp.radius));
  }



}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  Vector3f botLeft = Vector3f(this->botLeft.x, this->botLeft.y, this->botLeft.z);
  Vector3f topLeft = Vector3f(this->topLeft.x, this->topLeft.y, this->topLeft.z);
  Vector3f botRight = Vector3f(this->botRight.x, this->botRight.y, this->botRight.z);
  Vector3f topRight = Vector3f(this->topRight.x, this->topRight.y, this->topRight.z);

  positions.col(0) << topRight;
  positions.col(1) << topLeft;
  positions.col(2) << botRight;
  positions.col(3) << botLeft;

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  MatrixXf sand_mat(4, 4);
  sand_mat << -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0;

//  if (shader.attrib("in_is_sand", false) != -1) {
//    shader.uploadAttrib("in_is_sand", sand_mat);
//  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}


// Computes corners of plane by setting points in normalized space with normal (0, 1, 0) and point (0,0,0), then rotating and translating.
// https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
void Plane::_computePoints() {
    Vector3D a(0, 1, 0);
    Vector3D b(normal.unit());
    Vector3D v(cross(a, b));
    double c(dot(a, b));
    double arr[9] = { 0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0 };
    Matrix3x3 v_x(arr);

    Matrix3x3 R = Matrix3x3::identity();
	if (c != -1) {
		R += v_x;
		R += v_x* v_x* (1 / (1 + c));
	}
	botLeft = point + R * Vector3D(-(float)length / 2, 0, -(float)width / 2);
	topLeft = point + R * Vector3D((float)length / 2, 0, -(float)width / 2);
	botRight = point + R * Vector3D(-(float)length / 2, 0, (float)width / 2);
	topRight = point + R * Vector3D((float)length / 2, 0, (float)width / 2);
}


