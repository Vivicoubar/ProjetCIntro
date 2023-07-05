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
  context->particles = malloc(capacity * sizeof(Particle));
  memset(context->particles, 0, capacity * sizeof(Particle));

  //initialize planes
  context->num_ground_planes = NUM_GROUND_PLANES;
  context->ground_planes = malloc(context->num_ground_planes * sizeof(PlaneCollider));
  Vec2 start_pos_planes[NUM_GROUND_PLANES] = {{-9.5F, -7.5F}, {-11.5F, 7.5F}, {11.5F, -5.5F}, 
                                              {11.5F, 7.5F}, {-11.5F, -5.5F}, {9.5F, -7.5F}};
  Vec2 director_planes[NUM_GROUND_PLANES]  = {{19.F, 0.F}, {0.F, -13.F}, {0.F, 13.F},
                                              {-23.F, 0.F}, {2.F, -2.F}, {2.F, 2.F}};
  for (int i = 0; i < context->num_ground_planes; i++) {
    context->ground_planes[i].start_pos = start_pos_planes[i];
    context->ground_planes[i].director = director_planes[i];
  }

  //initialize spheres
  context->num_ground_spheres = NUM_GROUND_SPHERES;
  context->ground_spheres = malloc(context->num_ground_spheres * sizeof(SphereCollider));
  Vec2 center_spheres[NUM_GROUND_SPHERES] = {{5.5F, 0.F}};
  float radius_spheres[NUM_GROUND_SPHERES] = {2.5F};
  int num_spheres_placed = 1;
  for (int i = 0; i < 5 ; i++) {
    for (int j = 0; j < 4 ; j++) {
      center_spheres[num_spheres_placed] = (Vec2){-10.F + 2.F * (float) i + 1.F * (float) (j % 2), -2.F + 1.5F * (float) j};
      radius_spheres[num_spheres_placed] = 0.25F;
      num_spheres_placed++;
    }
  }
  for (int i = 0; i < context->num_ground_spheres; i++) {
    context->ground_spheres[i].center = center_spheres[i];
    context->ground_spheres[i].radius = radius_spheres[i];
  }
  
  context->ground_constraints = initializeGroundConstraint(capacity);
  context->particle_constraints = initializeParticleConstraint(capacity);
  context->bound_constraints = initializeBoundConstraint(capacity, capacity);
  
  return context;
}

void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id) {
  assert(context->num_particles < context->capacity_particles); // currently no resize in context
  
  //TODO Choisir entre augmenter le nombre de particules ou fermer volontairement la fenetre en liberant la memoire

  // if (context->num_particles >= context->capacity_particles) {
  //     printf("Erreur : capacite maximale de particules atteinte\n");
  //     free_memory(context);
  //     exit(0);
  // }

  context->particles[context->num_particles].position.x = x;
  context->particles[context->num_particles].position.y = y;
  context->particles[context->num_particles].velocity.x = 0.F;
  context->particles[context->num_particles].velocity.y = 0.F;
  context->particles[context->num_particles].inv_mass = 1.F/mass;
  context->particles[context->num_particles].radius = radius;
  context->particles[context->num_particles].draw_id = draw_id;
  context->num_particles += 1;
}

int addParticleWithId(Context* context, float x, float y, float radius, float mass, int draw_id) {
  assert(context->num_particles < context->capacity_particles); // currently no resize in context
  context->particles[context->num_particles].position.x = x;
  context->particles[context->num_particles].position.y = y;
  context->particles[context->num_particles].velocity.x = 0.F;
  context->particles[context->num_particles].velocity.y = 0.F;
  context->particles[context->num_particles].inv_mass = 1.F/mass;
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

  for(int k = 0; k < num_constraint_relaxation; ++k) { //// V -> Applied once for now, see particle.gui 
    projectConstraints(context); 
  }

  updateVelocityAndPosition(context, dt);
  applyFriction(context,dt);

  deleteContactConstraints(context);
}

void applyExternalForce(Context* context, float dt) {
  float g = 9.81F;
  float dv_grav = - g * dt;
  Vec2 dv = {0.F, dv_grav};
  for (int i = 0; i < context->num_particles; i++) {
    context->particles[i].velocity = vecSum(context->particles[i].velocity, dv);
  }
}

void dampVelocities(Context* context) {
}

void updateExpectedPosition(Context* context, float dt) {
  for (int i = 0; i < context->num_particles; i++) {
    Vec2 pos_i = context->particles[i].position;
    Vec2 v_i = context->particles[i].velocity;
    context->particles[i].next_pos = vecSum(pos_i, vecScale(v_i, dt));
  }
}

void addDynamicContactConstraints(Context* context) {
  for (int particle1_id = 0; particle1_id < context->num_particles; particle1_id++) {
    for (int particle2_id = 0; particle2_id < context->num_particles; particle2_id++) {
      if (particle1_id != particle2_id) {
        checkContactWithParticle(context, particle1_id, particle2_id);
      }
    }
  }
  for(int bound_id = 0; bound_id < context->bound_constraints->num_bounds; bound_id++) {
    checkBoundConstraint(context, bound_id);
  }
}

void useGrid(Context* context) {
  // Crer la grille
  IntArrayGrid* grid;
  grid = createGrid(12., 8., 0.4);
  
  // Ajouter les particules dans la grille
  for (int particle_id = 0; particle_id < context->num_particles; particle_id++) {
    Particle* particle = &context->particles[particle_id];
    Vec2 particle_pos = particle->next_pos;
    int particle_row = (int) (particle_pos.x / grid->cell_size);
    int particle_col = (int) (particle_pos.y / grid->cell_size);   
    // addCellValue(grid, particle_row, particle_col, particle_id); //TODO fct a def
  }

  // Parcourir la grille sans parcourir les bords
  for (int x = 1; x < grid->num_rows - 1; x++) {
    for (int y = 1; y < grid->num_cols - 1; y++) {
      int* cell = getCellValues(grid, x, y);
      int num_cell_values = getNumCellValues(grid, x, y);

      // Parcourir les cases autours
      for (int dx = -1; dx <= 1, dx++) {
        for (int dy = -1; dy <= 1, dy++) {
          int* other_cell = getCellValues(grid, x + dx, y + dy);
          int num_other_cell_values = getNumCellValues(grid, x + dx, y + dy);
          
          // Checker les collisions
          for (int particle1_id = 0; particle1_id < num_cell_values; particle1_id++) {
            for (int particle2_id = 0; particle2_id < num_other_cell_values; particle2_id++) {
              if (particle1_id != particle2_id) {
                checkContactWithParticle(context, particle1_id, particle2_id);
              }
            }
          }
        }
      }
    }
  }
}

void addStaticContactConstraints(Context* context) {
  for (int particle_id = 0; particle_id < context->num_particles; particle_id++) {
    for(int plane_id = 0; plane_id < context->num_ground_planes; plane_id++) {
        checkContactWithPlane(context, particle_id, &(context->ground_planes[plane_id]));
    }
    for(int sphere_id = 0; sphere_id < context->num_ground_spheres; sphere_id++) {
      checkContactWithSphere(context, particle_id, &(context->ground_spheres[sphere_id]));
    }
  }
}

void projectConstraints(Context* context) {
  for(int i = 0; i < context->ground_constraints->num_constraints; i++) {
    GroundConstraint* ground_constraints = context->ground_constraints;
    Particle* particle = &context->particles[ground_constraints->constraints[i].particle_id];
    Vec2 vec_constraint = ground_constraints->constraints[i].vec_constraint; 
    context->particles[ground_constraints->constraints[i].particle_id].next_pos = vecSum(particle->next_pos, vec_constraint);
  }
  for(int i = 0; i < context->particle_constraints->num_constraints; i++) {
    ParticleConstraint* particle_constraints = context->particle_constraints;
    Particle* particle = &context->particles[particle_constraints->constraints[i].particle_id];
    Vec2 vec_constraint = particle_constraints->constraints[i].vec_constraint; 
    context->particles[particle_constraints->constraints[i].particle_id].next_pos = vecSum(particle->next_pos, vec_constraint);
  }
  for(int i = 0; i < context->bound_constraints->num_constraints; i++) {
    BoundConstraint* bound_constraints = context->bound_constraints;
    Particle* particle = &context->particles[bound_constraints->constraints[i].particle_id];
    Vec2 vec_constraint = bound_constraints->constraints[i].vec_constraint; 
    context->particles[bound_constraints->constraints[i].particle_id].next_pos = vecSum(particle->next_pos, vec_constraint);
  }
}

void updateVelocityAndPosition(Context* context, float dt) {
  for(int i = 0; i < context->num_particles; i++) {
    Vec2 pos = context->particles[i].position;
    Vec2 next_pos = context->particles[i].next_pos;
    Vec2 new_velocity = vecScale(vecSubstract(next_pos, pos), 1.F / dt);
    context->particles[i].velocity = new_velocity;
    context->particles[i].position = next_pos;
  }
}

void applyFriction(Context* context, float dt) {
  float friction_coef = 0.6F;
  for(int i = 0; i < context->num_particles; i++) {
    context->particles[i].velocity = vecScale(context->particles[i].velocity, 1.F - friction_coef * context->particles[i].inv_mass * dt);
  }
}

void deleteContactConstraints(Context* context) {
  context->ground_constraints->num_constraints = 0;
  context->particle_constraints->num_constraints = 0;
  context->bound_constraints->num_constraints = 0;
}

void free_array(void* array) {
    if (array != NULL) {
      free(array);
      array = NULL;
    }
}



void free_memory(Context* context) {
  // A faire en premier
  free_array(context->bound_constraints->bounds);
  free_array(context->bound_constraints->constraints);
  free_array(context->ground_constraints->constraints);
  free_array(context->particle_constraints->constraints);

  // A faire apres
  free_array(context->particles);
  free_array(context->ground_spheres);
  free_array(context->ground_planes);
  free_array(context->ground_constraints);
  free_array(context->particle_constraints);
  free_array(context->bound_constraints);

  // A faire en dernier
  free_array(context);

}