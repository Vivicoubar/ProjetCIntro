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

// void checkContactWithParticle(Context* context, int particle_id1, int particle_id2) {
//   //TODO : Utiliser des noms de variables explicites et revoir les calculs
//   Vec2 xij = vecSubstract(context->particles[particle_id1].position, context->particles[particle_id2].position);
//   float c = sqrt(dotProduct(xij,xij)) - context->particles[particle_id1].radius - context->particles[particle_id2].radius;
//   if (c < 0) {
//       float di = context->particles[particle_id1].inv_mass / (context->particles[particle_id1].inv_mass + context->particles[particle_id2].inv_mass) * c;
//       Vec2 constraint = vecScale(xij, - di * sqrt(dotProduct(xij, xij)));
//       addParticleConstraint(context, constraint, particle_id1);
//   }
// }

void checkContactWithParticle(Context* context, int particle_id1, int particle_id2) {
  Particle particle1 = context->particles[particle_id1];
  Particle particle2 = context->particles[particle_id2];
  Vec2 particle1_pos = particle1.position;
  float particle1_radius = particle1.radius;
  Vec2 particle2_pos = particle2.position;
  float particle2_radius = particle2.radius;

  float particles_distance = norm(vecSubstract(particle1_pos, particle2_pos)) - particle1_radius - particle2_radius;

  if (particles_distance < 0) {
    float inv_m1 = particle1.inv_mass;
    float inv_m2 = particle2.inv_mass;
    float inv_mass_ratio1 = inv_m1 / (inv_m1  + inv_m2);
    float inv_mass_ratio2 = inv_m2 / (inv_m1  + inv_m2);
    
    Vec2 particle2_unit_normal = vecNormalize(vecSubstract(particle1_pos, particle2_pos));
    

    float particles_center_distance = norm(vecSubstract(particle1_pos, particle2_pos));    
    Vec2 wrong_constraint = vecScale(particle2_unit_normal, - inv_mass_ratio1 * particles_distance * particles_center_distance * particles_center_distance);
    // Vec2 constraint = vecScale(particle2_unit_normal, - inv_mass_ratio1 * particles_distance);

    addParticleConstraint(context, wrong_constraint, particle_id1);
  }
}

void checkBoundConstraint(Context* context, int bound_id) {
  BoundConstraint* bound_constraints = context->bound_constraints;
  int particle_id1 = bound_constraints->bounds[bound_id].particle1;
  int particle_id2 = bound_constraints->bounds[bound_id].particle2;
  Particle particle1 = context->particles[particle_id1];
  Particle particle2 = context->particles[particle_id2];
  
  float stiffness = bound_constraints->bounds[bound_id].stiffness;
  float target_distance = bound_constraints->bounds[bound_id].target_distance;
  Vec2 particle1_pos = particle1.next_pos;
  Vec2 particle2_pos = particle2.next_pos;
  float inv_m1 = particle1.inv_mass;
  float inv_m2 = particle2.inv_mass;
  
  float distance_from_equilibrium = norm(vecSubstract(particle1_pos, particle2_pos)) - target_distance;
  float beta = stiffness;
  float inv_mass_ratio1 = inv_m1 / (inv_m1  + inv_m2);
  float inv_mass_ratio2 = inv_m2 / (inv_m1  + inv_m2);
  
  Vec2 particle2_unit_normal = vecNormalize(vecSubstract(particle1_pos, particle2_pos));
  Vec2 constraint1 = vecScale(particle2_unit_normal, - inv_mass_ratio1 * beta * distance_from_equilibrium);
  Vec2 constraint2 = vecScale(particle2_unit_normal, inv_mass_ratio2 * beta * distance_from_equilibrium);
  
  addBoundConstraint(context, constraint1, particle_id1);
  addBoundConstraint(context, constraint2, particle_id2);
}


///////////////////////////////////


void checkContactWithBox(Context* context, int particle_id, int box_id) {
  // Get the directors and center of the box collider
  Vec2 director1 = context->box_collider[box_id].director1;
  Vec2 director2 = context->box_collider[box_id].director2;
  Vec2 center = context->box_collider[box_id].center;

  // Calculate half width and half height of the box
  float half_width = norm(director2) / 2.0f;
  float half_height = norm(director1) / 2.0f;

  // Get particle radius and position
  float particle_radius = context->particles[particle_id].radius;
  float particle_x = context->particles[particle_id].next_pos.x;
  float particle_y = context->particles[particle_id].next_pos.y;

  // Calculate the distances between particle and box edges
  float dx = fabs(particle_x - center.x) - (2 * half_width + particle_radius);
  float dy = fabs(particle_y - center.y) - (2 * half_height + particle_radius);

  // Check if the particle is colliding with the box
  if (dx <= 0.0f && dy <= 0.0f) {
    // Calculate the constraint vector
    float constraint_x = 0.0f;
    float constraint_y = 0.0f;

    if (dx > dy) {
  // If the collision is primarily in the x-direction
  if (particle_x < center.x) {
    // If the particle is to the left of the box center, move it to the right
    constraint_x = +dx;
  }
  else {
    // If the particle is to the right of the box center, move it to the left
    constraint_x = -dx;
  }
}
else {
  // If the collision is primarily in the y-direction
  if (particle_y < center.y) {
    // If the particle is above the box center, move it downward
    constraint_y = dy;
  }
  else {
    // If the particle is below the box center, move it upward
    constraint_y = -dy;
  }
}


    // Apply the constraint vector to move the particle outside the box
    Vec2 total_constraint = { constraint_x, constraint_y };
    addGroundConstraint(context, total_constraint, particle_id);
  }
}
