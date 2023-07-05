#pragma once

#include "Vec2.h"
#include "Particle.h"
#include "Constraint.h"

typedef struct Context {
  int num_particles;
  int capacity_particles;
  Particle* particles;

  // Ground colliders 
  int num_ground_spheres;
  SphereCollider* ground_spheres;
  int num_ground_planes;
  PlaneCollider* ground_planes;
  int num_boxes;
  BoxCollider* box_collider;

  // Constraints

  GroundConstraint* ground_constraints;
  ParticleConstraint* particle_constraints;
  BoundConstraint* bound_constraints;

} Context;

// Initializes a new Context with the given capacity.
Context* initializeContext(int capacity);

// Adds a particle to the context with the specified properties.
void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id);

// Adds a particle to the context with the specified properties and returns its ID.
int addParticleWithId(Context* context, float x, float y, float radius, float mass, int draw_id);

// Sets the draw ID of a sphere in the context.
void setDrawId(Context* context, int sphere_id, int draw_id);

// Retrieves the particle with the specified ID from the context.
Particle getParticle(Context* context, int id);

// Retrieves the ground sphere collider with the specified ID from the context.
SphereCollider getGroundSphereCollider(Context* context, int id);

// Retrieves the ground plane collider with the specified ID from the context.
PlaneCollider getGroundPlaneCollider(Context* context, int id);


// Updates the physical system in the context over a time step.
void updatePhysicalSystem(Context* context, float dt, int num_constraint_relaxation);

// Applies external forces to particles in the context.
void applyExternalForce(Context* context, float dt);

// Damps the velocities of particles in the context.
void dampVelocities(Context* context,  float dt);

// Updates the expected positions of particles in the context based on their velocities.
void updateExpectedPosition(Context* context, float dt);

// Adds dynamic contact constraints between particles in the context.
void addDynamicContactConstraints(Context* context);

// Adds static contact constraints between particles and ground colliders in the context.
void addStaticContactConstraints(Context* context);

// Projects the constraints in the context onto the particles' expected positions.
void projectConstraints(Context* context);

// Updates the velocities and positions of particles in the context.
void updateVelocityAndPosition(Context* context, float dt);

// Applies friction to the velocities of particles in the context.
void applyFriction(Context* context, float dt);

// Deletes all contact constraints in the context.
void deleteContactConstraints(Context* context);

// Creates line colliders in the context based on the specified parameters.
void createLineColliders(Context* context, Vec2* start_pos, int length, int sphere_num);

// Creates a Galton box in the context based on the specified parameters.
void createGaltonBox(Context* context, Vec2 start_pos, int lines, int sphere_num);
