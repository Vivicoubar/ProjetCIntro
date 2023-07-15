#include "Context.h"

#include "CollisionsGrid.h"

#include <stdio.h>
#include <stdlib.h>

IntArrayGrid* initializeCollisionsGrid(float len_x, float len_y, float cell_size) {
  IntArrayGrid* grid;
  grid = createGrid(len_x, len_y, cell_size);
}

void addParticlesGrid(Context* context) {
  IntArrayGrid* grid = context->collisions_grid;
  for (int particle_id = 0; particle_id < context->num_particles; particle_id++) {
    Particle* particle = &context->particles[particle_id];
    Vec2 particle_pos = particle->next_pos;
    int particle_col = (int) ((grid->len_x / 2.F + particle_pos.x) / grid->cell_size);
    int particle_row = (int) ((grid->len_y / 2.F + particle_pos.y) / grid->cell_size);   
    addCellValue(grid, particle_row, particle_col, particle_id);
  }
}

void findNeighboringParticlesAndCheckContact(Context* context) {
  IntArrayGrid* grid = context->collisions_grid;
  for (int x = 1; x < grid->num_rows - 1; x++) {
    for (int y = 1; y < grid->num_cols - 1; y++) {
      int* cell = getCellValues(grid, x, y);
      int num_cell_values = getNumberCellValues(grid, x, y);
      // Parcourir les cases autours
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          int* other_cell = getCellValues(grid, x + dx, y + dy);
          int num_other_cell_values = getNumberCellValues(grid, x + dx, y + dy);
          // Checker les collisions
          for (int cell_index = 1; cell_index <= num_cell_values; cell_index++) {
            for (int other_cell_index = 1; other_cell_index <= num_other_cell_values; other_cell_index++) {
              int particle1_id = cell[cell_index];
              int particle2_id = other_cell[other_cell_index];
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


void checkParticlesCollisions(Context* context) {
  IntArrayGrid* grid = context->collisions_grid;
  clearGrid(grid);
  addParticlesGrid(context);
  findNeighboringParticlesAndCheckContact(context);
}