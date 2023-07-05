#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

typedef struct Context Context;
#include "Vec2.h"

typedef struct SphereCollider {
  Vec2 center;
  float radius;
} SphereCollider;

typedef struct PlaneCollider {
  Vec2 start_pos;
  Vec2 director;
} PlaneCollider;

typedef struct Constraint {
  Vec2 vec_constraint;  
  int particle_id;
} Constraint;

typedef struct Bound {
  int particle1;
  int particle2;
  float target_distance;
  float stiffness;
} Bound;

typedef struct BoundConstraint {
  int num_bounds;
  Bound* bounds;
  int capacity_bounds;
  int num_constraints;
  Constraint* constraints;
  int capacity_constraints;
} BoundConstraint;

typedef struct BoxCollider {
  Vec2 center;
  Vec2 director1;
  Vec2 director2;
} BoxCollider;

typedef struct GroundConstraint {
  int num_constraints;
  Constraint* constraints;
  int capacity_constraints;
} GroundConstraint;

typedef struct ParticleConstraint {
  int num_constraints;
  Constraint* constraints;
  int capacity_constraints;
} ParticleConstraint;


GroundConstraint* initializeGroundConstraint(int capacity);
ParticleConstraint* initializeParticleConstraint(int capacity);
BoundConstraint* initializeBoundConstraint(int capacity_bounds, int capacity_constraints);

void addBound(Context* context, float x, float y, float radius, float mass, int draw_id1, int draw_id2, int draw_id3, int draw_id4);
void addGroundConstraint(Context* context, Vec2 constraint, int origin);
void addParticleConstraint(Context* context, Vec2 constraint, int origin);
void addBoundConstraint(Context* context, Vec2 constraint, int origin);

void checkContactWithPlane(Context* context, int particle_id, PlaneCollider* collider);
void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider);
void checkContactWithParticle(Context* context, int particle_id1, int particle_id2);
void checkBoundConstraint(Context* context, int bound_id);
void checkContactWithBox(Context* context, int particle_id, int box_id);

#endif