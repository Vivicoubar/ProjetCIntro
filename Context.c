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

int addParticleWithId(Context* context, float x, float y, float radius, float mass, int draw_id)
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
    return context->num_particles -1;
}


void addBound(Context* context, float x, float y, float radius, float mass, int draw_id1, int draw_id2, int draw_id3, int draw_id4) {
  assert(context->bounds_constraints->num_constraint + 5 < context->bounds_constraints->capacity_bounds);
    int num1 = addParticleWithId(context, x-radius, y-radius, radius, mass, draw_id1);
    int num2 = addParticleWithId(context, x-radius, y+radius, radius, mass, draw_id2);
    int num3 = addParticleWithId(context, x+radius, y+radius, radius, mass, draw_id3);
    int num4 = addParticleWithId(context, x+radius, y-radius, radius, mass, draw_id4);
    Bound bound1 = {num1, num2, radius*2, 0.5};

    Bound bound2 = {num2, num3, radius*2, 0.5};
    Bound bound3 = {num3, num4, radius*2, 0.5};
    Bound bound4 = {num4, num1, radius*2, 0.5};
    Bound bound5 = {num2, num4, radius*2 *sqrt(2), 0.5};
    Bound bound6 = {num1, num3, radius*2 *sqrt(2), 0.5};
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

BoxCollider getBoxCollider(Context* context, int id)
{
  return context->box_collider[id];
}
// ------------------------------------------------
Context* initializeContext(int capacity)
{
  Context* context = malloc(sizeof(Context));
  context->num_particles = 0;
  context->capacity_particles = capacity;
  context->particles = malloc(capacity*sizeof(Particle));
  memset(context->particles,0,capacity*sizeof(Particle));
  
  //initialize spheres
  
  int num_simple_sphere = 0;
  int num_line = 0;
  
  int num_galton_spheres= 45;
  context->num_ground_sphere = num_simple_sphere + 5 * num_line + num_galton_spheres;
  context->ground_spheres = malloc(context->num_ground_sphere*sizeof(SphereCollider));
  /*
  Vec2 center_spheres[] = {{-5.5F, -1.0F}, {-2.5F, 3.0F}, {0.0F, 0.0F}, {2.5F, 3.0F}, {5.5F, -1.0F}};
  float radius_spheres[] = {0.5F, 0.7F, 0.2F, 0.7F, 0.5F};
  for (int i = 0; i < num_simple_sphere; i++) {
    context->ground_spheres[i].center = center_spheres[i];
    context->ground_spheres[i].radius = radius_spheres[i];
  }*/

  /*
  Vec2 start_pos[] = {{-8.0F,-5.0F},{-6.0F,-5.0F}, {-4.0F,-5.0F}, {-2.0F,-5.0F},
                       {-0.0F,-5.0F}, {2.0F,-5.0F}, {4.0F,-5.0F}, {6.0F,-5.0F}, {8.0F,-5.0F}};
  createLineColliders(context, start_pos, num_line, num_simple_sphere);
  */
  Vec2 origin = {0.0f, 7.0f};
  int num_floors = 9;
  createGaltonBox(context, origin, num_floors, num_simple_sphere + num_line*5);
  

  //initialize planes
  context->num_ground_plane = 3;
  context->ground_planes = malloc(context->num_ground_plane*sizeof(PlaneCollider));
  Vec2 start_pos_planes[] = {{9.5F, -5.0F}, {-9.5F, -5.0F},  {9.5F, 9.5F}};
  Vec2 director_planes[]  = {{-19.0F, 0.0F}, {0.0F, 14.5F}, {0.0F, -14.5F}};
  for (int i = 0; i < context->num_ground_plane; i++) {
    context->ground_planes[i].start_pos = start_pos_planes[i];
    context->ground_planes[i].director = director_planes[i];
  }

    //initialize Boxes.
  
  int num_boxes = 11;
  context->num_boxes = num_boxes;
  context->box_collider = malloc(context->num_boxes*sizeof(BoxCollider));
  
  Vec2 directors_1[] = {{0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f}, {0.0f,1.0f},  {-4.5f,0.2f} ,  {4.5f,0.2f}};
  Vec2 directors_2[] = {{0.10f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.0f}, {0.0f, 0.2f}, {0.0f, 0.2f}};
  Vec2 centers[] = {{-8.0f,-4.0f}, {-6.0f,-4.0f}, {-4.0f,-4.0f}, {-2.0f,-4.0f}, {-0.0f,-4.0f}, {2.0f,-4.0f}, {4.0f,-4.0f}, {6.0f,-4.0f}, {8.0f,-4.0f}, {-5.0f, 8.5f}, {5.0f, 8.5f}};
  for (int i=0; i<num_boxes; i++) {
    context->box_collider[i].center = centers[i];
    context->box_collider[i].director1 = directors_1[i];
    context->box_collider[i].director2 = directors_2[i];
  }

 //initialize constraints 
  context->ground_constraints = initializeGroundConstraint(capacity*100);
  context->particle_constraints = initializeParticleConstraint(capacity*100);
  context->bounds_constraints = initializeBoundsConstraint(capacity*100);

  


  
  return context;
}

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

// ------------------------------------------------

void updatePhysicalSystem(Context* context, float dt, int num_constraint_relaxation)
{
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

void addStaticContactConstraints(Context* context)
{
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
    for(int j=0; j < context->num_boxes; j++) {
      checkContactWithBox(context, i, j);
    }
  }
}

void projectConstraints(Context* context)
{
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

void applyFriction(Context* context, float dt)
{
  for(int i=0; i<context->num_particles; i++) {
    //Apply a constraint for the friction
    context->particles[i].velocity = substractVector(context->particles[i].velocity, multiplyByScalar(context->particles[i].velocity, 0.6*context->particles[i].inv_mass*dt));
  }
}

void deleteContactConstraints(Context* context)
{
  context->ground_constraints->num_constraint = 0;
  context->particle_constraints->num_constraint = 0;
  context->bounds_constraints->num_constraint = 0;
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
  Vec2 center = collider->center;
  float radius = collider->radius;
  float sdf = sqrt(dotProduct(substractVector(pos_particle,center),substractVector(pos_particle,center))) - radius - context->particles[particle_id].radius;
  if(sdf < 0) {
    Vec2 normal = substractVector(pos_particle, center);
    normal = normalize(normal);
    addGroundConstraint(context, multiplyByScalar(normal, -sdf), particle_id);
  }
  
}

void checkContactWithParticle(Context* context, int particle_id1, int particle_id2) {
  Vec2 xij = substractVector(context->particles[particle_id1].next_pos, context->particles[particle_id2].next_pos);
  float c = sqrt(dotProduct(xij,xij)) - context->particles[particle_id1].radius - context->particles[particle_id2].radius;
  if (c < 0) {
      float di = context->particles[particle_id2].inv_mass / (context->particles[particle_id2].inv_mass  + context->particles[particle_id1].inv_mass ) * c;
      Vec2 constraint = multiplyByScalar(xij,-di * (c +  context->particles[particle_id1].radius + context->particles[particle_id2].radius));
      addParticleConstraint(context, constraint, particle_id1);
  }
}

void checkContactWithBox(Context* context, int particle_id, int box_id) {
  Vec2 director1 = (context->box_collider[box_id].director1);
  Vec2 director2 = (context->box_collider[box_id].director2);
  Vec2 center = context->box_collider[box_id].center;

  float half_width = length(director2) / 2.0f;
  float half_height = length(director1) / 2.0f;
  float particle_radius = context->particles[particle_id].radius;
  float particle_x = context->particles[particle_id].next_pos.x;
  float particle_y = context->particles[particle_id].next_pos.y;

  float dx = fabs(particle_x - center.x) - (2*half_width + particle_radius);
  float dy = fabs(particle_y - center.y) - (2*half_height + particle_radius);

  // Check if the particle is colliding with the box
  if (dx <= 0.0f && dy <= 0.0f) {
    // Calculate the constraint vector
    float constraint_x = 0.0f;
    float constraint_y = 0.0f;

    if (dx > dy) {
      if (particle_x < center.x) {
        constraint_x = +dx;
      }
      else {
        constraint_x = -dx;
      }
      constraint_y = 0.0f;
    }
    else {
      constraint_x = 0.0f;
      if (particle_y < center.y) {
        constraint_y = dy;
      }
      else {
        constraint_y = -dy;
      }
    }

    // Apply the constraint vector to move the particle outside the box
    Vec2 total_constraint = { constraint_x, constraint_y };
    addGroundConstraint(context, total_constraint, particle_id);
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


void createLineColliders(Context* context, Vec2 * start_pos, int length, int sphere_num) {
  float radius = 0.2F;
  int line_height = 5;
  for (int i =0; i<length; i++) {
    for (int j =0; j<line_height; j++) {
      int index = sphere_num + i*line_height + j; 
      Vec2 pos = {start_pos[i].x, start_pos[i].y + j*2*radius + radius};
      context->ground_spheres[index].center = pos;
      context->ground_spheres[index].radius = radius;
    }
  }
}

void createGaltonBox(Context* context, Vec2 start_pos, int lines, int sphere_num) {
  float radius = 0.2F;
  float distance = 1.0F;
  int count = 0;
  for (int i =0; i < lines; i++) {
    for(int j =0; j <= i; j++) {
      int index = sphere_num + count;
      count ++;
      Vec2 pos = {start_pos.x - distance*i + distance*2*j, start_pos.y - distance*i};
      context->ground_spheres[index].center = pos;
      context->ground_spheres[index].radius = radius;
    }

  }
}



// ------------------------------------------------