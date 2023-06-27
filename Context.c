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

Context* initializeContext(int capacity)
{
  Context* context = malloc(sizeof(Context));
  context->num_particles = 0;
  context->capacity_particles = capacity;
  context->particles = malloc(capacity*sizeof(Particle));
  memset(context->particles,0,capacity*sizeof(Particle));

  context->num_ground_sphere = 4;
  context->ground_spheres = malloc(4*sizeof(SphereCollider));
  Vec2 p0 = {0.0f, 0.0f};
  context->ground_spheres[0].center = p0;
  context->ground_spheres[0].radius = 1.7;

  return context;
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
  float unity_factor = 200.0F;
  float g = 9.81F * unity_factor;
  float dv = -g * dt; // dv = -g*dt
  float dy = dv * dt; // d = dv*dt

  for (int i = 0; i < context->num_particles; i++) {
    context->particles[i].velocity.y += dv;
    context->particles[i].position.y += dy;
  }


  /*
  update Velocity
    v_i+1 = v_i + dt/m * F_ext(...)
  expected position
    p_i+1 = p_i + dt * v_i+1
  constraint(end contact)
  update Velocity And Position
     v_i+1 = 1/dt * (p_i+1 - p_i)
     p_i+1 -> position = new_pos

  
  CONTACT :
      
  
  
  
  */
}

void dampVelocities(Context* context)
{
}

void updateExpectedPosition(Context* context, float dt)
{
  for (int i = 0; i < context->num_particles; i++) {
    context->particles[i].next_pos.y += context->particles[i].velocity.y*dt;
  }
}

void addDynamicContactConstraints(Context* context)
{
}

void addStaticContactConstraints(Context* context)
{
}

void projectConstraints(Context* context)
{
}

void updateVelocityAndPosition(Context* context, float dt)
{
  for (int i = 0; i < context->num_particles; i++) {
    context->particles[i].velocity.y = (context->particles[i].next_pos.y - context->particles[i].position.y)/dt;
  }
}

void applyFriction(Context* context)
{
}

void deleteContactConstraints(Context* context)
{
}

// ------------------------------------------------
