#include "Context.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

// ------------------------------------------------

Particle getParticle(Context* context, int id)
{
  return context->particles[id];
}

// ------------------------------------------------

void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id)
{
    assert(context->num_particles<context->capacity_particles); // currently no resize in context
    context->particles[context->num_particles].position.x = x;
    context->particles[context->num_particles].position.y = y;
    context->particles[context->num_particles].velocity.x = 0.0F;
    context->particles[context->num_particles].velocity.y = 0.0F;
    context->particles[context->num_particles].inv_mass = 1.0F/mass;
    context->particles[context->num_particles].radius = radius;
    context->particles[context->num_particles].draw_id = draw_id;
    context->num_particles += 1;
}

// ------------------------------------------------

void setDrawId(Context* context, int sphere_id, int draw_id)
{
  context->particles[sphere_id].draw_id = draw_id;
}

// ------------------------------------------------

SphereCollider getGroundSphereCollider(Context* context, int id)
{
  return context->ground_spheres[id];
}

// ------------------------------------------------

PlaneCollider getGroundPlaneCollider(Context* context, int id)
{
  return context->ground_planes[id];
}

// ------------------------------------------------
Context* initializeContext(int capacity)
{
  Context* context = malloc(sizeof(Context));
  context->num_particles = 0;
  context->capacity_particles = capacity;
  context->particles = malloc(capacity*sizeof(Particle));
  memset(context->particles,0,capacity*sizeof(Particle));
  
  // Spheres
  context->num_ground_sphere = 5;
  context->ground_spheres = malloc(context->num_ground_sphere*sizeof(SphereCollider));
  Vec2 center_spheres[] = {{-5.5F, -1.0F}, {-2.5F, 3.0F}, {0.0F, -0.5F}, {2.5F, 3.0F}, {5.5F, -1.0F}};
  float radius_spheres[] = {1.5F, 1.7F, 0.7F, 1.7F, 1.5F};
  for (int i = 0; i < context->num_ground_sphere; i++) {
    context->ground_spheres[i].center = center_spheres[i];
    context->ground_spheres[i].radius = radius_spheres[i];
  }


  //Planes
  context->num_ground_plane = 1;
  context->ground_planes = malloc(context-> num_ground_plane*sizeof(PlaneCollider));
  Vec2 pstart = {-20.0f, -5.0f};
  Vec2 pend = {-4.0f, 1.0};
  context->ground_planes[0].start_pos = pstart;
  context->ground_planes[0].director = pend;
  

  return context;
}

GroundConstraint* initializeGroundConstraint(int capacity) {
  GroundConstraint* constraint = malloc(sizeof(GroundConstraint));
  return constraint;
}

// ------------------------------------------------

void updatePhysicalSystem(Context* context, float dt, int num_constraint_relaxation)
{
  applyExternalForce(context, dt);
  dampVelocities(context);
  updateExpectedPosition(context, dt);
  addDynamicContactConstraints(context);
  addStaticContactConstraints(context);

 
  for(int k=0; k<num_constraint_relaxation; ++k) {
    projectConstraints(context);
  }

  updateVelocityAndPosition(context, dt);
  applyFriction(context);

  deleteContactConstraints(context);
}

// ------------------------------------------------

void applyExternalForce(Context* context, float dt)
{
  float g = 9.81F;
  Vec2 dv = {0, - g * dt};
  for (int i = 0; i < context->num_particles; i++) {
    context->particles[i].velocity = sumVector(context->particles[i].velocity, dv);
  }
}

void dampVelocities(Context* context)
{
}

void updateExpectedPosition(Context* context, float dt)
{
  for (int i = 0; i < context->num_particles; i++) {
    context->particles[i].next_pos = sumVector( context->particles[i].position, multiplyByScalar( context->particles[i].velocity,dt));
  }

}

void addDynamicContactConstraints(Context* context)
{
}

void addStaticContactConstraints(Context* context)
{
  for (int i = 0; i < context->num_particles; i++) {
    for(int j = 0; j < context->num_ground_plane; j++) {
      checkContactWithPlane(context, i, &(context->ground_planes[j]));
      
    }
  }
}

void projectConstraints(Context* context)
{
}

void updateVelocityAndPosition(Context* context, float dt)
{
  for(int i=0; i < context->num_particles; i++) {
    Vec2 pos = context->particles[i].position;
    Vec2 newPos = context->particles[i].next_pos;
    Vec2 newVelocity = multiplyByScalar(substractVector(newPos,pos), 1.0/dt);
    context->particles[i].velocity = newVelocity;
    context->particles[i].position = newPos;
  }


}

void applyFriction(Context* context)
{
}

void deleteContactConstraints(Context* context)
{
}

void checkContactWithPlane(Context* context, int particle_id, PlaneCollider* collider) {
  Vec2 pos_particle = context->particles[particle_id].position;
  Vec2 pos_plane = collider->start_pos;
  Vec2 director = collider->director;
  Vec2 normal_c = {-director.y, director.x};
  normal_c = normalize(normal_c);
  float scalar_proj_qc = scalarProduct(substractVector(pos_particle, pos_plane), normal_c);
  Vec2 q_c = substractVector(pos_particle,multiplyByScalar(normal_c, scalar_proj_qc));
  float scalar_proj_c = scalarProduct(substractVector(pos_particle, q_c), normal_c);
  
  float c = scalar_proj_c - context->particles[particle_id].radius;
  if(c < 0.0F) {
    //context->particles[particle_id].next_pos = substractVector(context->particles[particle_id].next_pos ,multiplyByScalar(normal_c,c));
  }
}

void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider) {
  Vec2 pos_particle = context->particles[particle_id].position;
  Vec2 center = collider->center;
  float radius = collider->radius;
  Vec2 normal = substractVector(pos_particle, center);
  normal = normalize(normal);
  Vec2 pos_plane = sumVector(center, multiplyByScalar(normal, radius));
  Vec2 director = {normal.y, -normal.x};

  
}

// ------------------------------------------------