#include "Context.h"

#include "Constraint.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

GroundConstraint* initializeGroundConstraint(int capacity) {
  GroundConstraint* constraint = malloc(sizeof(GroundConstraint));
  constraint->num_constraint = 0;
  constraint->capacity_constraints = 200*sizeof(constraint->constraints); //TODO Améliorer la structure
  constraint->constraints = malloc(capacity*sizeof(constraint->constraints));
  return constraint;
}

ParticleConstraint* initializeParticleConstraint(int capacity) {
  ParticleConstraint* constraint = malloc(sizeof(ParticleConstraint));
  constraint->num_constraint = 0;
  constraint->capacity_constraints = 200*sizeof(constraint->constraints); //TODO Améliorer la structure
  constraint->constraints = malloc(constraint->capacity_constraints*sizeof(constraint->constraints));
  return constraint;
}

BoundConstraint* initializeBoundsConstraint(int capacity) {
  BoundConstraint* bounds = malloc(sizeof(BoundConstraint));
  bounds->num_bounds = 0;
  bounds->capacity_bounds = 200*sizeof(bounds->bounds); //TODO Améliorer la structure
  bounds->bounds = malloc(bounds->capacity_bounds*sizeof(bounds->bounds));
  bounds->num_constraint = 0;
  bounds->capacity_constraints = 200*sizeof(bounds->constraints); //TODO Améliorer la structure
  bounds->constraints = malloc(bounds->capacity_constraints*sizeof(bounds->constraints));
  return bounds;
}

void addBound(Context* context, float x, float y, float radius, float mass, int draw_id1, int draw_id2, int draw_id3, int draw_id4) {
  assert(context->bounds_constraints->num_constraint + 5 < context->bounds_constraints->capacity_bounds);
    int num1 = addParticleWithId(context, x-radius, y-radius, radius, mass, draw_id1);
    int num2 = addParticleWithId(context, x-radius, y+radius, radius, mass, draw_id2);
    int num3 = addParticleWithId(context, x+radius, y+radius, radius, mass, draw_id3);
    int num4 = addParticleWithId(context, x+radius, y-radius, radius, mass, draw_id4);
    float stiffness = 0.3;
    Bound bound1 = {num1, num2, radius*2, stiffness};
    Bound bound2 = {num2, num3, radius*2, stiffness};
    Bound bound3 = {num3, num4, radius*2, stiffness};
    Bound bound4 = {num4, num1, radius*2, stiffness};
    Bound bound5 = {num2, num4, radius*2 *sqrt(2), stiffness};
    Bound bound6 = {num1, num3, radius*2 *sqrt(2), stiffness};
    context->bounds_constraints->bounds[context->bounds_constraints->num_bounds] = bound1;
    context->bounds_constraints->num_bounds += 1;
    context->bounds_constraints->bounds[context->bounds_constraints->num_bounds] = bound2;
    context->bounds_constraints->num_bounds += 1;
    context->bounds_constraints->bounds[context->bounds_constraints->num_bounds] = bound3;
    context->bounds_constraints->num_bounds += 1;
    context->bounds_constraints->bounds[context->bounds_constraints->num_bounds] = bound4;
    context->bounds_constraints->num_bounds += 1;
    context->bounds_constraints->bounds[context->bounds_constraints->num_bounds] = bound5;
    context->bounds_constraints->num_bounds += 1;
    context->bounds_constraints->bounds[context->bounds_constraints->num_bounds] = bound6;
    context->bounds_constraints->num_bounds += 1;
}

void addGroundConstraint(Context* context, Vec2 constraint, int origin) {
  //Each constraint is a Vec (the movement), and an origin (the num of the particle where the constraint should be applied) 
  assert(context->ground_constraints->num_constraint < context->ground_constraints->capacity_constraints);
  GroundConstraint* ground_constraints = context->ground_constraints;
  ground_constraints->constraints[ground_constraints->num_constraint].constraint = constraint;
  ground_constraints->constraints[ground_constraints->num_constraint].origin = origin;
  ground_constraints->num_constraint +=1;
}

void addParticleConstraint(Context* context, Vec2 constraint, int origin) {
  //Each constraint is a Vec (the movement), and an origin (the num of the particle where the constraint should be applied) 
  assert(context->particle_constraints->num_constraint < context->particle_constraints->capacity_constraints);
  ParticleConstraint* particle_constraints = context->particle_constraints;
  particle_constraints->constraints[particle_constraints->num_constraint].constraint = constraint;
  particle_constraints->constraints[particle_constraints->num_constraint].origin = origin;
  particle_constraints->num_constraint +=1;
}

void addBoundConstraint(Context* context, Vec2 constraint, int origin) {
  //Each constraint is a Vec (the movement), and an origin (the num of the particle where the constraint should be applied) 
  assert(context->bounds_constraints->num_constraint < context->bounds_constraints->capacity_constraints);
  BoundConstraint* bounds_constraints = context->bounds_constraints;
  bounds_constraints->constraints[bounds_constraints->num_constraint].constraint = constraint;
  bounds_constraints->constraints[bounds_constraints->num_constraint].origin = origin;
  bounds_constraints->num_constraint +=1;
}

void checkContactWithPlane(Context* context, int particle_id, PlaneCollider* collider) {
  Vec2 pos_particle = context->particles[particle_id].next_pos;
  Vec2 pos_plane = collider->start_pos;
  Vec2 director = collider->director;
  Vec2 normal_c = {director.y, -director.x};//Should be pointing to the top
  normal_c = normalize(normal_c); 
  float scalar_proj_qc = dotProduct(substractVector(pos_particle, pos_plane), normal_c);
  Vec2 q_c = substractVector(pos_particle,multiplyByScalar(normal_c, scalar_proj_qc));
  float scalar_proj_c = dotProduct(substractVector(pos_particle, q_c), normal_c);
  
  float c = scalar_proj_c - context->particles[particle_id].radius;
  
  if(c < 0.0F) {
    addGroundConstraint(context, multiplyByScalar(normal_c,-c), particle_id);
  }
}

void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider) {
  Vec2 pos_particle = context->particles[particle_id].next_pos;
  float radius_particle = context->particles[particle_id].radius;
  Vec2 center = collider->center;
  float radius = collider->radius;
  float sdf = sqrt(dotProduct(substractVector(pos_particle,center),substractVector(pos_particle,center))) - radius_particle - radius;
  if(sdf < 0) {
    Vec2 normal = substractVector(pos_particle, center);
    normal = normalize(normal);
    addGroundConstraint(context, multiplyByScalar(normal, -sdf), particle_id);
  }
  
}

void checkContactWithParticle(Context* context, int particle_id1, int particle_id2) {
  Vec2 xij = substractVector(context->particles[particle_id1].position, context->particles[particle_id2].position);
  float c = sqrt(dotProduct(xij,xij)) - context->particles[particle_id1].radius - context->particles[particle_id2].radius;
  if (c < 0) {
      float di = context->particles[particle_id1].inv_mass / (context->particles[particle_id1].inv_mass  + context->particles[particle_id2].inv_mass ) * c;
      Vec2 constraint = multiplyByScalar(xij,-di * sqrt(dotProduct(xij,xij)));
      addParticleConstraint(context, constraint, particle_id1);
  }
}

void checkBoundConstraint(Context* context, int bound_id) {
  int particle_id1 = context->bounds_constraints->bounds[bound_id].particle1;
  int particle_id2 = context->bounds_constraints->bounds[bound_id].particle2;
  Particle particle1 = context->particles[particle_id1];
  Particle particle2 = context->particles[particle_id2];

  float stiffness = context->bounds_constraints->bounds[bound_id].stiffness;
  float target_distance = context->bounds_constraints->bounds[bound_id].target_distance;
  
  float inv_m1 = particle1.inv_mass;
  float inv_m2 = particle2.inv_mass;
  
  Vec2 x_21 = substractVector(particle1.next_pos, particle2.next_pos);
  float norm_x_21 = sqrt(dotProduct(x_21, x_21));
  float c = norm_x_21 - target_distance;
  float beta = stiffness;

  float factor1 = - inv_m1 / (inv_m1 + inv_m2) * beta * c / norm_x_21;
  Vec2 constraint1 = multiplyByScalar(x_21, factor1);
  
  float factor2 = inv_m2 / (inv_m1 + inv_m2) * beta * c / norm_x_21;
  Vec2 constraint2 = multiplyByScalar(x_21, factor2);

  addBoundConstraint(context, constraint1, particle_id1);
  addBoundConstraint(context, constraint2, particle_id2);
}