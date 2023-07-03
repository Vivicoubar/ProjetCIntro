#include "Context.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#define NUM_GROUND_PLANES 6
#define NUM_GROUND_SPHERES 21

Context* initializeContext(int capacity) {
  Context* context = malloc(sizeof(Context));
  context->num_particles = 0;
  context->capacity_particles = capacity;
  context->particles = malloc(capacity*sizeof(Particle));
  memset(context->particles,0,capacity*sizeof(Particle));
  

  //initialize planes
  context->num_ground_plane = NUM_GROUND_PLANES;
  context->ground_planes = malloc(context->num_ground_plane*sizeof(PlaneCollider));
  Vec2 start_pos_planes[NUM_GROUND_PLANES] = {{9.5F, -7.5F}, {-11.5F, -5.5F}, {11.5F, 7.5F}, {-11.5F, 7.5F}, {-9.5F, -7.5F}, {11.5F, -5.5F}};
  Vec2 director_planes[NUM_GROUND_PLANES]  = {{-19.0F, 0.0F}, {0.0F, 13.0F}, {0.0F, -13.0F}, {23.0F, 0.0F}, {-2.0F, 2.0F}, {-2.0F, -2.0F}};
  for (int i = 0; i < context->num_ground_plane; i++) {
    context->ground_planes[i].start_pos = start_pos_planes[i];
    context->ground_planes[i].director = director_planes[i];
  }

  //initialize spheres
  context->num_ground_sphere = NUM_GROUND_SPHERES;
  context->ground_spheres = malloc(context->num_ground_sphere*sizeof(SphereCollider));

  Vec2 center_spheres[NUM_GROUND_SPHERES] = {{5.5F, 0.F}};
  float radius_spheres[NUM_GROUND_SPHERES] = {2.5F};
  int num_spheres_placed = 1;
  for (int i = 0; i < 5 ; i++) {
    for (int j = 0; j < 4 ; j++) {
      center_spheres[num_spheres_placed] = (Vec2){-10.F + 2.F * (float) i + 1.F * (float) (j % 2), -2.F + 1.5F * (float) j};
      radius_spheres[num_spheres_placed] = .25F;
      num_spheres_placed++;
    }
  }

  for (int i = 0; i < context->num_ground_sphere; i++) {
    context->ground_spheres[i].center = center_spheres[i];
    context->ground_spheres[i].radius = radius_spheres[i];
  }
  
  context->ground_constraints = initializeGroundConstraint(capacity*100);
  context->particle_constraints = initializeParticleConstraint(capacity*100);
  context->bounds_constraints = initializeBoundsConstraint(capacity*100);
  
  return context;
}

void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id) {
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

int addParticleWithId(Context* context, float x, float y, float radius, float mass, int draw_id) {
    assert(context->num_particles<context->capacity_particles); // currently no resize in context
    context->particles[context->num_particles].position.x = x;
    context->particles[context->num_particles].position.y = y;
    context->particles[context->num_particles].velocity.x = 0.0F;
    context->particles[context->num_particles].velocity.y = 0.0F;
    context->particles[context->num_particles].inv_mass = 1.0F/mass;
    context->particles[context->num_particles].radius = radius;
    context->particles[context->num_particles].draw_id = draw_id;
    context->num_particles += 1;
    return context->num_particles -1;
}

void setDrawId(Context* context, int sphere_id, int draw_id) {
  context->particles[sphere_id].draw_id = draw_id;
}

Particle getParticle(Context* context, int id) {
  return context->particles[id];
}

SphereCollider getGroundSphereCollider(Context* context, int id) {
  return context->ground_spheres[id];
}

PlaneCollider getGroundPlaneCollider(Context* context, int id) {
  return context->ground_planes[id];
}

void updatePhysicalSystem(Context* context, float dt, int num_constraint_relaxation) {
  applyExternalForce(context, dt);
  dampVelocities(context);
  updateExpectedPosition(context, dt);
  addDynamicContactConstraints(context);
  addStaticContactConstraints(context);

 
  for(int k=0; k<num_constraint_relaxation; ++k) { //// V -> Applied once for now, see particle.gui 
    projectConstraints(context); 
  }

  updateVelocityAndPosition(context, dt);
  applyFriction(context,dt);

  deleteContactConstraints(context);
}

void applyExternalForce(Context* context, float dt) {
  float g = 9.81F;
  Vec2 dv = {0, - g * dt};
  for (int i = 0; i < context->num_particles; i++) {
    context->particles[i].velocity = sumVector(context->particles[i].velocity, dv);
  }
}

void dampVelocities(Context* context) {
}

void updateExpectedPosition(Context* context, float dt) {
  for (int i = 0; i < context->num_particles; i++) {
    context->particles[i].next_pos = sumVector( context->particles[i].position, multiplyByScalar( context->particles[i].velocity,dt));
  }

}

void addDynamicContactConstraints(Context* context) {
  for (int i = 0; i < context->num_particles; i++) {
    for (int j = 0; j < context->num_particles; j++) {
        // In order to optimize the code, calculate the distance between two particles. If bigger than sum of radius, skip
        if (i != j) {
        Vec2 vector = substractVector(context->particles[i].position, context->particles[j].position);
        if(sqrt(dotProduct(vector, vector)) <= (context->particles[i].radius + context->particles[j].radius)) {
          checkContactWithParticle(context,i ,j);
        }
      }
    }
  }
  for(int i=0; i < context->bounds_constraints->num_bounds; i++) {
    checkBoundConstraint(context, i);
  }
}

void addStaticContactConstraints(Context* context) {
  for (int i = 0; i < context->num_particles; i++) {
    for(int j = 0; j < context->num_ground_plane; j++) {
      // In order to optimize the code, calculate the orthogonal projection. If bigger than radius, skip
      Vec2 normal = {context->ground_planes[j].director.y, -context->ground_planes[j].director.x};
      Vec2 vector = {context->particles[i].position.x - context->ground_planes[j].start_pos.x, context->particles[i].position.y - context->ground_planes[j].start_pos.y};
      if(abs(dotProduct(normal, vector)) / (sqrt(dotProduct(normal, normal))) <= context->particles[i].radius) {
              checkContactWithPlane(context, i, &(context->ground_planes[j]));
      }
    }
    for(int j=0; j < context->num_ground_sphere; j++) {
      // In order to optimize the code, calculate the distance between the sphere and the particle. If bigger than sum of radius, skip
      Vec2 vector = substractVector(context->particles[i].position, context->ground_spheres[j].center);
      if(sqrt(dotProduct(vector, vector)) <= (context->particles[i].radius + context->ground_spheres[j].radius)) {
        checkContactWithSphere(context, i, &(context->ground_spheres[j]));
      }
    }
  }
}

void projectConstraints(Context* context) {
  for(int i = 0; i < context->ground_constraints->num_constraint; i++) {
    GroundConstraint* ground_constraints = context->ground_constraints;
    context->particles[ground_constraints->constraints[i].origin].next_pos = sumVector(context->particles[ground_constraints->constraints[i].origin].next_pos, ground_constraints->constraints[i].constraint);
  }
  for(int i = 0; i < context->particle_constraints->num_constraint; i++) {
    ParticleConstraint* particle_constraints = context->particle_constraints;
    context->particles[particle_constraints->constraints[i].origin].next_pos = sumVector(context->particles[particle_constraints->constraints[i].origin].next_pos, particle_constraints->constraints[i].constraint);
  }
  for(int i = 0; i < context->bounds_constraints->num_constraint; i++) {
    BoundConstraint* bound_constraint = context->bounds_constraints;
    context->particles[bound_constraint->constraints[i].origin].next_pos = sumVector(context->particles[bound_constraint->constraints[i].origin].next_pos, bound_constraint->constraints[i].constraint);
  }
}

void updateVelocityAndPosition(Context* context, float dt) {
  for(int i=0; i < context->num_particles; i++) {
    Vec2 pos = context->particles[i].position;
    Vec2 newPos = context->particles[i].next_pos;
    Vec2 newVelocity = multiplyByScalar(substractVector(newPos,pos), 1.0/dt);
    context->particles[i].velocity = newVelocity;
    context->particles[i].position = newPos;
  }


}

void applyFriction(Context* context, float dt) {
  for(int i=0; i<context->num_particles; i++) {
    //Apply a constraint for the friction
    context->particles[i].velocity = substractVector(context->particles[i].velocity, multiplyByScalar(context->particles[i].velocity, 0.6*context->particles[i].inv_mass*dt));
  }
}

void deleteContactConstraints(Context* context) {
  context->ground_constraints->num_constraint = 0;
  context->particle_constraints->num_constraint = 0;
  context->bounds_constraints->num_constraint = 0;
}
