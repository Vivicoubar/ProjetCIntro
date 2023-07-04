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
  BoundConstraint* bound_constraints = context->bound_constraints;
  if (bound_constraints->num_constraints + 6 >= bound_constraints->capacity_constraints) {
    // Si le tableau est plein, augmenter la capacité
    int new_capacity = bound_constraints->capacity_constraints * 2;
    bound_constraints->capacity_constraints = new_capacity;
    bound_constraints->constraints = realloc(bound_constraints->constraints, new_capacity * sizeof(Constraint));
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

  bound_constraints->bounds[bound_constraints->num_bounds] = bound1;
  bound_constraints->num_bounds += 1;
  bound_constraints->bounds[bound_constraints->num_bounds] = bound2;
  bound_constraints->num_bounds += 1;
  bound_constraints->bounds[bound_constraints->num_bounds] = bound3;
  bound_constraints->num_bounds += 1;
  bound_constraints->bounds[bound_constraints->num_bounds] = bound4;
  bound_constraints->num_bounds += 1;
  bound_constraints->bounds[bound_constraints->num_bounds] = bound5;
  bound_constraints->num_bounds += 1;
  bound_constraints->bounds[bound_constraints->num_bounds] = bound6;
  bound_constraints->num_bounds += 1;
}

void addGroundConstraint(Context* context, Vec2 vec_constraint, int particle_id) {
  GroundConstraint* ground_constraints = context->ground_constraints;
  if (ground_constraints->num_constraints >= ground_constraints->capacity_constraints) {
    // Si le tableau est plein, augmenter la capacité
    int new_capacity = ground_constraints->capacity_constraints * 2;
    ground_constraints->capacity_constraints = new_capacity;
    ground_constraints->constraints = realloc(ground_constraints->constraints, new_capacity * sizeof(Constraint));
  }
  // Ajouter la nouvelle contrainte
  ground_constraints->constraints[ground_constraints->num_constraints].vec_constraint = vec_constraint;
  ground_constraints->constraints[ground_constraints->num_constraints].particle_id = particle_id;
  ground_constraints->num_constraints += 1;
}

void addParticleConstraint(Context* context, Vec2 vec_constraint, int particle_id) {
  ParticleConstraint* particle_constraints = context->particle_constraints;
  if (particle_constraints->num_constraints >= particle_constraints->capacity_constraints) {
    // Si le tableau est plein, augmenter la capacité
    int new_capacity = particle_constraints->capacity_constraints * 2;
    particle_constraints->capacity_constraints = new_capacity;
    particle_constraints->constraints = realloc(particle_constraints->constraints, new_capacity * sizeof(Constraint));
  }
  // Ajouter la nouvelle contrainte
  particle_constraints->constraints[particle_constraints->num_constraints].vec_constraint = vec_constraint;
  particle_constraints->constraints[particle_constraints->num_constraints].particle_id = particle_id;
  particle_constraints->num_constraints += 1;
}

void addBoundConstraint(Context* context, Vec2 vec_constraint, int particle_id) {
  BoundConstraint* bound_constraints = context->bound_constraints;
  if (bound_constraints->num_constraints >= bound_constraints->capacity_constraints) {
    // Si le tableau est plein, augmenter la capacité
    int new_capacity = bound_constraints->capacity_constraints * 2;
    bound_constraints->capacity_constraints = new_capacity;
    bound_constraints->constraints = realloc(bound_constraints->constraints, new_capacity * sizeof(Constraint));
  }
  // Ajouter la nouvelle contrainte
  bound_constraints->constraints[bound_constraints->num_constraints].vec_constraint = vec_constraint;
  bound_constraints->constraints[bound_constraints->num_constraints].particle_id = particle_id;
  bound_constraints->num_constraints += 1;
}

void checkContactWithPlane(Context* context, int particle_id, PlaneCollider* plane_collider) {
  Particle* particle = &context->particles[particle_id];
  Vec2 particle_pos = particle->next_pos;
  float particle_radius = particle->radius;
  Vec2 plane_start_pos = plane_collider->start_pos;
  Vec2 plane_director = plane_collider->director;

  Vec2 plane_normal = vecClockwiseNormal(plane_director);
  Vec2 plane_unit_normal = vecNormalize(plane_normal);
  float particle_plane_distance  = dotProduct(vecSubstract(particle_pos, plane_start_pos), plane_unit_normal) - particle_radius;

  if(particle_plane_distance < 0.F) {
    Vec2 vec_constraint = vecScale(plane_unit_normal, - particle_plane_distance);
    addGroundConstraint(context, vec_constraint, particle_id);
  }
}

void checkContactWithSphere(Context* context, int particle_id, SphereCollider* sphere_collider) {
  Particle* particle = &context->particles[particle_id];
  Vec2 particle_pos = particle->next_pos;
  float particle_radius = particle->radius;
  Vec2 sphere_center = sphere_collider->center;
  float sphere_radius = sphere_collider->radius;

  float particle_sphere_distance = norm(vecSubstract(particle_pos, sphere_center)) - particle_radius - sphere_radius;

  if(particle_sphere_distance < 0.F) { 
    Vec2 sphere_unit_normal = vecNormalize(vecSubstract(particle_pos, sphere_center));

    Vec2 vec_constraint = vecScale(sphere_unit_normal, - particle_sphere_distance);
    addGroundConstraint(context, vec_constraint, particle_id);  
  }
}


void checkContactWithParticle(Context* context, int particle_id1, int particle_id2) {
  //TODO : Utiliser des noms de variables explicites et revoir les calculs
  Vec2 xij = vecSubstract(context->particles[particle_id1].position, context->particles[particle_id2].position);
  float c = sqrt(dotProduct(xij,xij)) - context->particles[particle_id1].radius - context->particles[particle_id2].radius;
  if (c < 0) {
      float di = context->particles[particle_id1].inv_mass / (context->particles[particle_id1].inv_mass + context->particles[particle_id2].inv_mass) * c;
      Vec2 constraint = vecScale(xij, - di * sqrt(dotProduct(xij, xij)));
      addParticleConstraint(context, constraint, particle_id1);
  }
}

void checkBoundConstraint(Context* context, int bound_id) {
  //TODO : Utiliser des noms de variables explicites et revoir les calculs
  BoundConstraint* bound_constraints = context->bound_constraints;
  int particle_id1 = bound_constraints->bounds[bound_id].particle1;
  int particle_id2 = bound_constraints->bounds[bound_id].particle2;
  Particle particle1 = context->particles[particle_id1];
  Particle particle2 = context->particles[particle_id2];

  float stiffness = bound_constraints->bounds[bound_id].stiffness;
  float target_distance = bound_constraints->bounds[bound_id].target_distance;
  
  float inv_m1 = particle1.inv_mass;
  float inv_m2 = particle2.inv_mass;
  
  Vec2 x_21 = vecSubstract(particle1.next_pos, particle2.next_pos);
  float norm_x_21 = sqrt(dotProduct(x_21, x_21));
  float c = norm_x_21 - target_distance;
  float beta = stiffness;

  float factor1 = - inv_m1 / (inv_m1 + inv_m2) * beta * c / norm_x_21;
  Vec2 constraint1 = vecScale(x_21, factor1);
  
  float factor2 = inv_m2 / (inv_m1 + inv_m2) * beta * c / norm_x_21;
  Vec2 constraint2 = vecScale(x_21, factor2);

  addBoundConstraint(context, constraint1, particle_id1);
  addBoundConstraint(context, constraint2, particle_id2);
}