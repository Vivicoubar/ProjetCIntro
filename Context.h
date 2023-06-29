#pragma once

#include "Vec2.h"
#include "Particle.h"
#include "Constraint.h"

// ------------------------------------------------

typedef struct Constraint {
  
  Vec2 constraint;
  int origin;

} Constraint;

typedef struct GroundConstraint {

  int num_constraint;
  Constraint* constraints;
  int capacity_constraints;
  int origin;

} GroundConstraint;


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
  
} Context;

// --------------------------------------------------

Context* initializeContext(int capacity);


GroundConstraint* initializeGroundConstraint(int capacity);

// ------------------------------------------------

void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id);

// ------------------------------------------------

Particle getParticle(Context* context, int id);

// ------------------------------------------------

SphereCollider getGroundSphereCollider(Context* context, int id);

// ------------------------------------------------

PlaneCollider getGroundPlaneCollider(Context* context, int id);

// ------------------------------------------------

void setDrawId(Context* context, int sphere_id, int draw_id);

// ------------------------------------------------

void updatePhysicalSystem(Context* context, float dt, int num_constraint_relaxation);

// ------------------------------------------------
// Methods below are called by updatePhysicalSystem
// ------------------------------------------------

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
// ------------------------------------------------

