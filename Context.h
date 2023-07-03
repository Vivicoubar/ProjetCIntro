#pragma once

#include "Vec2.h"
#include "Particle.h"
#include "Constraint.h"

typedef struct Constraint {
  
  Vec2 constraint;  
  int origin;

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
  int num_constraint;
  Constraint* constraints;
  int capacity_constraints;
} BoundConstraint;

typedef struct GroundConstraint {

  int num_constraint;
  Constraint* constraints;
  int capacity_constraints;

} GroundConstraint;

typedef struct ParticleConstraint {

  int num_constraint;
  Constraint* constraints;
  int capacity_constraints;

} ParticleConstraint;


typedef struct Context {
  int num_particles;
  int capacity_particles;
  Particle* particles;

  // Ground colliders 
  int num_ground_sphere;
  SphereCollider* ground_spheres;

  //Plane colliders
  int num_ground_plane;
  PlaneCollider* ground_planes;

  //Constraints
  GroundConstraint* ground_constraints;
  
  ParticleConstraint* particle_constraints;

  BoundConstraint* bounds_constraints;
} Context;

// --------------------------------------------------

Context* initializeContext(int capacity);


GroundConstraint* initializeGroundConstraint(int capacity);

ParticleConstraint* initializeParticleConstraint(int capacity);

BoundConstraint* initializeBoundsConstraint(int capacity);

void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id);

Particle getParticle(Context* context, int id);

SphereCollider getGroundSphereCollider(Context* context, int id);

PlaneCollider getGroundPlaneCollider(Context* context, int id);

void setDrawId(Context* context, int sphere_id, int draw_id);

void updatePhysicalSystem(Context* context, float dt, int num_constraint_relaxation);
// Methods below are called by updatePhysicalSystem
void applyExternalForce(Context* context, float dt);
void dampVelocities(Context* context);
void updateExpectedPosition(Context* context, float dt);
void addDynamicContactConstraints(Context* context);
void addStaticContactConstraints(Context* context);
void projectConstraints(Context* context);
void updateVelocityAndPosition(Context* context, float dt);
void applyFriction(Context* context, float );
void deleteContactConstraints(Context* context);
void checkContactWithPlane(Context* context, int particle_id, PlaneCollider* collider);
void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider);
void checkContactWithParticle(Context* context, int particle_id1, int particle_id2);
void addParticleConstraint(Context* context, Vec2 constraint, int origin);
void checkBoundConstraint(Context* context, int bound_id);
void addBoundConstraint(Context* context, Vec2 constraint, int origin);
