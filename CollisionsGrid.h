#ifndef COLLISIONSGRID_H_
#define COLLISIONSGRID_H_

typedef struct Context Context;
#include "IntArrayGrid.h"
#include "Constraint.h"

IntArrayGrid* initializeCollisionsGrid(float len_x, float len_y, float cell_size);
void addParticlesGrid(Context* context);
void findNeighboringParticlesAndCheckContact(Context* context);
void checkParticlesCollisions(Context* context);

#endif