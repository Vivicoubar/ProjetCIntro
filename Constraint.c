#include "Context.h"

#include "Constraint.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

GroundConstraint* initializeGroundConstraint(int capacity) {
  GroundConstraint* constraint = malloc(sizeof(GroundConstraint));
  constraint->num_constraints = 0;
  constraint->capacity_constraints = capacity;
  constraint->constraints = malloc(capacity * sizeof(Constraint));
  return constraint;
}

ParticleConstraint* initializeParticleConstraint(int capacity) {
  ParticleConstraint* constraint = malloc(sizeof(ParticleConstraint));
  constraint->num_constraints = 0;
  constraint->capacity_constraints = capacity;
  constraint->constraints = malloc(capacity * sizeof(Constraint));
  return constraint;
}

BoundConstraint* initializeBoundConstraint(int capacity_bounds, int capacity_constraints) {
  BoundConstraint* bounds = malloc(sizeof(BoundConstraint));
  bounds->num_bounds = 0;
  bounds->capacity_bounds = capacity_bounds;
  bounds->bounds = malloc(capacity_bounds * sizeof(Bound));
  bounds->num_constraints = 0;
  bounds->capacity_constraints = capacity_constraints;
  bounds->constraints = malloc(capacity_constraints * sizeof(Constraint));
  return bounds;
}

void addBound(Context* context, float x, float y, float radius, float mass, int draw_id1, int draw_id2, int draw_id3, int draw_id4) {
  if (context->bound_constraints->num_constraints + 6 >= context->bound_constraints->capacity_constraints) {
    // Si le tableau est plein, augmenter la capacité
    int new_capacity = context->bound_constraints->capacity_constraints * 2;
    context->bound_constraints->capacity_constraints = new_capacity;
    context->bound_constraints->constraints = realloc(context->bound_constraints->constraints, new_capacity * sizeof(Constraint));
  }
  // Ajouter la nouvelle liaison
    int num1 = addParticleWithId(context, x - radius, y - radius, radius, mass, draw_id1);
    int num2 = addParticleWithId(context, x - radius, y + radius, radius, mass, draw_id2);
    int num3 = addParticleWithId(context, x + radius, y + radius, radius, mass, draw_id3);
    int num4 = addParticleWithId(context, x + radius, y - radius, radius, mass, draw_id4);
    float stiffness = 0.3F;
    Bound bound1 = {num1, num2, radius * 2, stiffness};
    Bound bound2 = {num2, num3, radius * 2, stiffness};
    Bound bound3 = {num3, num4, radius * 2, stiffness};
    Bound bound4 = {num4, num1, radius * 2, stiffness};
    Bound bound5 = {num2, num4, radius * 2 * sqrt(2), stiffness};
    Bound bound6 = {num1, num3, radius * 2 * sqrt(2), stiffness};
    context->bound_constraints->bounds[context->bound_constraints->num_bounds] = bound1;
    context->bound_constraints->num_bounds += 1;
    context->bound_constraints->bounds[context->bound_constraints->num_bounds] = bound2;
    context->bound_constraints->num_bounds += 1;
    context->bound_constraints->bounds[context->bound_constraints->num_bounds] = bound3;
    context->bound_constraints->num_bounds += 1;
    context->bound_constraints->bounds[context->bound_constraints->num_bounds] = bound4;
    context->bound_constraints->num_bounds += 1;
    context->bound_constraints->bounds[context->bound_constraints->num_bounds] = bound5;
    context->bound_constraints->num_bounds += 1;
    context->bound_constraints->bounds[context->bound_constraints->num_bounds] = bound6;
    context->bound_constraints->num_bounds += 1;
}

void addGroundConstraint(Context* context, Vec2 vec_constraint, int particle_id) {
  if (context->ground_constraints->num_constraints >= context->ground_constraints->capacity_constraints) {
    // Si le tableau est plein, augmenter la capacité
    int new_capacity = context->ground_constraints->capacity_constraints * 2;
    context->ground_constraints->capacity_constraints = new_capacity;
    context->ground_constraints->constraints = realloc(context->ground_constraints->constraints, new_capacity * sizeof(Constraint));
  }
  // Ajouter la nouvelle contrainte
  GroundConstraint* ground_constraints = context->ground_constraints;
  ground_constraints->constraints[ground_constraints->num_constraints].vec_constraint = vec_constraint;
  ground_constraints->constraints[ground_constraints->num_constraints].particle_id = particle_id;
  ground_constraints->num_constraints += 1;
}

void addParticleConstraint(Context* context, Vec2 vec_constraint, int particle_id) {
  if (context->particle_constraints->num_constraints >= context->particle_constraints->capacity_constraints) {
    // Si le tableau est plein, augmenter la capacité
    int new_capacity = context->particle_constraints->capacity_constraints * 2;
    context->particle_constraints->capacity_constraints = new_capacity;
    context->particle_constraints->constraints = realloc(context->particle_constraints->constraints, new_capacity * sizeof(Constraint));
  }
  // Ajouter la nouvelle contrainte
  ParticleConstraint* particle_constraints = context->particle_constraints;
  particle_constraints->constraints[particle_constraints->num_constraints].vec_constraint = vec_constraint;
  particle_constraints->constraints[particle_constraints->num_constraints].particle_id = particle_id;
  particle_constraints->num_constraints += 1;
}

void addBoundConstraint(Context* context, Vec2 vec_constraint, int particle_id) {
  if (context->bound_constraints->num_constraints >= context->bound_constraints->capacity_constraints) {
    // Si le tableau est plein, augmenter la capacité
    int new_capacity = context->bound_constraints->capacity_constraints * 2;
    context->bound_constraints->capacity_constraints = new_capacity;
    context->bound_constraints->constraints = realloc(context->bound_constraints->constraints, new_capacity * sizeof(Constraint));
  }
  // Ajouter la nouvelle contrainte
  BoundConstraint* bound_constraints = context->bound_constraints;
  bound_constraints->constraints[bound_constraints->num_constraints].vec_constraint = vec_constraint;
  bound_constraints->constraints[bound_constraints->num_constraints].particle_id = particle_id;
  bound_constraints->num_constraints += 1;
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
    addGroundConstraint(context, multiplyByScalar(normal_c, - c), particle_id);
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
    addGroundConstraint(context, multiplyByScalar(normal, - sdf), particle_id);
  }
}

void checkContactWithParticle(Context* context, int particle_id1, int particle_id2) {
  Vec2 xij = substractVector(context->particles[particle_id1].position, context->particles[particle_id2].position);
  float c = sqrt(dotProduct(xij,xij)) - context->particles[particle_id1].radius - context->particles[particle_id2].radius;
  if (c < 0) {
      float di = context->particles[particle_id1].inv_mass / (context->particles[particle_id1].inv_mass + context->particles[particle_id2].inv_mass) * c;
      Vec2 constraint = multiplyByScalar(xij, - di * sqrt(dotProduct(xij, xij)));
      addParticleConstraint(context, constraint, particle_id1);
  }
}

void checkBoundConstraint(Context* context, int bound_id) {
  int particle_id1 = context->bound_constraints->bounds[bound_id].particle1;
  int particle_id2 = context->bound_constraints->bounds[bound_id].particle2;
  Particle particle1 = context->particles[particle_id1];
  Particle particle2 = context->particles[particle_id2];

  float stiffness = context->bound_constraints->bounds[bound_id].stiffness;
  float target_distance = context->bound_constraints->bounds[bound_id].target_distance;
  
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