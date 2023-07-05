#include "Context.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#define NUM_GROUND_PLANES 6
#define NUM_GROUND_SPHERES 21

Context* initializeContext(int capacity) {
  // Allocate memory for a Context structure
  Context* context = malloc(sizeof(Context));

  // Set the initial number of particles to 0
  context->num_particles = 0;

  // Set the capacity of the particles array
  context->capacity_particles = capacity;

  // Allocate memory for the particles array and set all values to 0
  context->particles = malloc(capacity * sizeof(Particle));
  memset(context->particles, 0, capacity * sizeof(Particle));

  // Initialize ground planes
  context->num_ground_planes = NUM_GROUND_PLANES;
  context->ground_planes = malloc(context->num_ground_planes * sizeof(PlaneCollider));
  
  // Define the start positions and directors of the ground planes
  Vec2 start_pos_planes[NUM_GROUND_PLANES] = {{-9.5F, -7.5F}, {-11.5F, 7.5F}, {11.5F, -5.5F}, 
                                              {11.5F, 7.5F}, {-11.5F, -5.5F}, {9.5F, -7.5F}};
  Vec2 director_planes[NUM_GROUND_PLANES]  = {{19.F, 0.F}, {0.F, -13.F}, {0.F, 13.F},
                                              {-23.F, 0.F}, {2.F, -2.F}, {2.F, 2.F}};

  // Assign start positions and directors to the ground planes
  for (int i = 0; i < context->num_ground_planes; i++) {
    context->ground_planes[i].start_pos = start_pos_planes[i];
    context->ground_planes[i].director = director_planes[i];
  }

  // Initialize ground spheres
  context->num_ground_spheres = NUM_GROUND_SPHERES;
  context->ground_spheres = malloc(context->num_ground_spheres * sizeof(SphereCollider));

  // Define the center positions and radii of the ground spheres
  Vec2 center_spheres[NUM_GROUND_SPHERES] = {{5.5F, 0.F}};
  float radius_spheres[NUM_GROUND_SPHERES] = {2.5F};
  int num_spheres_placed = 1;

  // Place additional spheres in a grid pattern
  for (int i = 0; i < 5 ; i++) {
    for (int j = 0; j < 4 ; j++) {
      center_spheres[num_spheres_placed] = (Vec2){-10.F + 2.F * (float) i + 1.F * (float) (j % 2), -2.F + 1.5F * (float) j};
      radius_spheres[num_spheres_placed] = 0.25F;
      num_spheres_placed++;
    }
  }

  // Assign center positions and radii to the ground spheres
  for (int i = 0; i < context->num_ground_spheres; i++) {
    context->ground_spheres[i].center = center_spheres[i];
    context->ground_spheres[i].radius = radius_spheres[i];
  }
  
  // Initialize ground constraints
  context->ground_constraints = initializeGroundConstraint(capacity);

  // Initialize particle constraints
  context->particle_constraints = initializeParticleConstraint(capacity);

  // Initialize bound constraints
  context->bound_constraints = initializeBoundConstraint(capacity, capacity);
  
   //initialize Boxes.
  
  int num_boxes = 11;
  context->num_boxes = num_boxes;
  context->box_collider = malloc(context->num_boxes*sizeof(BoxCollider));
  
  Vec2 directors_1[] = {{0.0f,1.0f} ,  {-4.5f,-0.3f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f},  {-4.5f,0.3f} };
  Vec2 directors_2[] = {{0.10f, 0.0f}, {0.0f, 0.2f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.0f, 0.2f}};
  Vec2 centers[] = {{-8.0f,-6.5f} , {6.0f, 5.5f}, {-6.0f,-6.5f}, {-4.0f,-6.5f}, {-2.0f,-6.5f}, {-0.0f,-6.5f}, {2.0f,-6.5f}, {4.0f,-6.5f}, {6.0f,-6.5f}, {8.0f,-6.5f}, {-6.0f, 5.5f}};
  for (int i=0; i<num_boxes; i++) {
    context->box_collider[i].center = centers[i];
    context->box_collider[i].director1 = directors_1[i];
    context->box_collider[i].director2 = directors_2[i];
  }
  
  // Return the initialized context
  return context;
}

void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id) {
  // Check if the number of particles has reached the capacity
  assert(context->num_particles < context->capacity_particles);

  // TODO: Choose between increasing the number of particles or deliberately closing the window by freeing memory
  // if (context->num_particles >= context->capacity_particles) {
  //     printf("Erreur : capacite maximale de particules atteinte\n");
  //     free_memory(context);
  //     exit(0);
  // }


  // Set the properties of the new particle at the next available index in the particles array
  context->particles[context->num_particles].position.x = x;
  context->particles[context->num_particles].position.y = y;
  context->particles[context->num_particles].velocity.x = 0.F;
  context->particles[context->num_particles].velocity.y = 0.F;
  context->particles[context->num_particles].inv_mass = 1.F / mass;
  context->particles[context->num_particles].radius = radius;
  context->particles[context->num_particles].draw_id = draw_id;

  // Increment the number of particles
  context->num_particles += 1;
}


int addParticleWithId(Context* context, float x, float y, float radius, float mass, int draw_id) {
  assert(context->num_particles < context->capacity_particles); // Check if the number of particles has reached the capacity

  // Set the properties of the new particle at the next available index in the particles array
  context->particles[context->num_particles].position.x = x;
  context->particles[context->num_particles].position.y = y;
  context->particles[context->num_particles].velocity.x = 0.F;
  context->particles[context->num_particles].velocity.y = 0.F;
  context->particles[context->num_particles].inv_mass = 1.F / mass;
  context->particles[context->num_particles].radius = radius;
  context->particles[context->num_particles].draw_id = draw_id;

  // Increment the number of particles
  context->num_particles += 1;

  // Return the index of the newly added particle
  return context->num_particles - 1;
}

void setDrawId(Context* context, int sphere_id, int draw_id) {
  context->particles[sphere_id].draw_id = draw_id;
}

Particle getParticle(Context* context, int id) {
  return context->particles[id];
}
  
  BoxCollider getBoxCollider(Context* context, int id)
{
  return context->box_collider[id];
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

  for(int k = 0; k < num_constraint_relaxation; ++k) {
    projectConstraints(context); 
  }

  updateVelocityAndPosition(context, dt);
  applyFriction(context,dt);

  deleteContactConstraints(context);
}

void applyExternalForce(Context* context, float dt) {
  float g = 9.81F;                      // Gravitational acceleration
  float dv_grav = -g * dt;              // Change in velocity due to gravity in the y-axis
  Vec2 dv = {0.F, dv_grav};             // Velocity change vector in the y-axis

  // Apply the external force (gravity) to each particle
  for (int i = 0; i < context->num_particles; i++) {
    context->particles[i].velocity = vecSum(context->particles[i].velocity, dv);
  }
}

void dampVelocities(Context* context) {
}

void updateExpectedPosition(Context* context, float dt) {
  for (int i = 0; i < context->num_particles; i++) {
    Vec2 pos_i = context->particles[i].position;            // Current position of the particle i
    Vec2 v_i = context->particles[i].velocity;              // Velocity of the particle i
    context->particles[i].next_pos = vecSum(pos_i, vecScale(v_i, dt));  // Calculate the expected position of the particle i after time dt
  }
}

void addDynamicContactConstraints(Context* context) {
  // Check for contact constraints between particles
  for (int particle1_id = 0; particle1_id < context->num_particles; particle1_id++) {
    for (int particle2_id = 0; particle2_id < context->num_particles; particle2_id++) {
      if (particle1_id != particle2_id) {
        checkContactWithParticle(context, particle1_id, particle2_id);  // Check for contact between particle1 and particle2
      }
    }
  }

  // Check for contact constraints with bounds
  for (int bound_id = 0; bound_id < context->bound_constraints->num_bounds; bound_id++) {
    checkBoundConstraint(context, bound_id);  // Check for contact between particles and the specified bound
  }
}

void addStaticContactConstraints(Context* context) {
  // Check for contact constraints with ground planes
  for (int particle_id = 0; particle_id < context->num_particles; particle_id++) {
    for (int plane_id = 0; plane_id < context->num_ground_planes; plane_id++) {
      checkContactWithPlane(context, particle_id, &(context->ground_planes[plane_id])); // Check for contact between particle and ground plane
    }
  }

  // Check for contact constraints with ground spheres
  for (int particle_id = 0; particle_id < context->num_particles; particle_id++) {
    for (int sphere_id = 0; sphere_id < context->num_ground_spheres; sphere_id++) {
      checkContactWithSphere(context, particle_id, &(context->ground_spheres[sphere_id])); // Check for contact between particle and ground sphere
    }
    for(int j=0; j < context->num_boxes; j++) {
      checkContactWithBox(context, particle_id, j);
    }
  }
}

void projectConstraints(Context* context) {
  // Project ground constraints
  for (int i = 0; i < context->ground_constraints->num_constraints; i++) {
    GroundConstraint* ground_constraints = context->ground_constraints;
    Particle* particle = &context->particles[ground_constraints->constraints[i].particle_id];
    Vec2 vec_constraint = ground_constraints->constraints[i].vec_constraint;
    // Update the next position of the particle by adding the constraint vector
    context->particles[ground_constraints->constraints[i].particle_id].next_pos = vecSum(particle->next_pos, vec_constraint);
  }

  // Project particle constraints
  for (int i = 0; i < context->particle_constraints->num_constraints; i++) {
    ParticleConstraint* particle_constraints = context->particle_constraints;
    Particle* particle = &context->particles[particle_constraints->constraints[i].particle_id];
    Vec2 vec_constraint = particle_constraints->constraints[i].vec_constraint;
    // Update the next position of the particle by adding the constraint vector
    context->particles[particle_constraints->constraints[i].particle_id].next_pos = vecSum(particle->next_pos, vec_constraint);
  }

  // Project bound constraints
  for (int i = 0; i < context->bound_constraints->num_constraints; i++) {
    BoundConstraint* bound_constraints = context->bound_constraints;
    Particle* particle = &context->particles[bound_constraints->constraints[i].particle_id];
    Vec2 vec_constraint = bound_constraints->constraints[i].vec_constraint;
    // Update the next position of the particle by adding the constraint vector
    context->particles[bound_constraints->constraints[i].particle_id].next_pos = vecSum(particle->next_pos, vec_constraint);
  }
}

void updateVelocityAndPosition(Context* context, float dt) {
  for (int i = 0; i < context->num_particles; i++) {
    // Get the current position and next position of the particle
    Vec2 pos = context->particles[i].position;
    Vec2 next_pos = context->particles[i].next_pos;

    // Calculate the new velocity based on the difference between next position and current position
    Vec2 new_velocity = vecScale(vecSubstract(next_pos, pos), 1.F / dt);

    // Update the velocity and position of the particle
    context->particles[i].velocity = new_velocity;
    context->particles[i].position = next_pos;
  }
}

void applyFriction(Context* context, float dt) {
  float friction_coef = 0.6F;
  for (int i = 0; i < context->num_particles; i++) {
    // Apply friction to the particle's velocity
    context->particles[i].velocity = vecScale(context->particles[i].velocity, 1.F - friction_coef * context->particles[i].inv_mass * dt);
  }
}

void deleteContactConstraints(Context* context) {
  // Reset the number of contact constraints to zero
  context->ground_constraints->num_constraints = 0;
  context->particle_constraints->num_constraints = 0;
  context->bound_constraints->num_constraints = 0;
}

void createGaltonBox(Context* context, Vec2 start_pos, int lines, int sphere_num) {
  //Build the GaltonBox
  float radius = 0.2F;
  float distance = 1.0F;
  int count = 0;
  for (int i = 0; i < lines; i++) {
    for (int j = 0; j <= i; j++) {
      int index = sphere_num + count;
      count++;
      Vec2 pos = {start_pos.x - distance * i + distance * 2 * j, start_pos.y - distance * i};
      // Set the center and radius of the ground sphere
      context->ground_spheres[index].center = pos;
      context->ground_spheres[index].radius = radius;
    }
  }
}

void createLineColliders(Context* context, Vec2* start_pos, int length, int sphere_num) {
  float radius = 0.2F;
  int line_height = 5;
  for (int i = 0; i < length; i++) {
    for (int j = 0; j < line_height; j++) {
      int index = sphere_num + i * line_height + j;
      Vec2 pos = {start_pos[i].x, start_pos[i].y + j * 2 * radius + radius};
      // Set the center and radius of the ground sphere
      context->ground_spheres[index].center = pos;
      context->ground_spheres[index].radius = radius;
    }
  }
}

void free_array(void* array) {
  if (array != NULL) {
    // Free the memory and set the array pointer to NULL
    free(array);
    array = NULL;
  }
}

void free_memory(Context* context) {
  // First
  free_array(context->bound_constraints->bounds);
  free_array(context->bound_constraints->constraints);
  free_array(context->ground_constraints->constraints);
  free_array(context->particle_constraints->constraints);

  // Then
  free_array(context->particles);
  free_array(context->ground_spheres);
  free_array(context->ground_planes);
  free_array(context->ground_constraints);
  free_array(context->particle_constraints);
  free_array(context->bound_constraints);

  // Finally
  free_array(context);

}
