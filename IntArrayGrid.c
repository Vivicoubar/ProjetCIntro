#include "IntArrayGrid.h"

#include <stdlib.h>
#include <stdio.h>

IntArrayGrid* createGrid(float len_x, float len_y, float cell_size) {
    IntArrayGrid* grid = (IntArrayGrid*)malloc(sizeof(IntArrayGrid));
    grid->len_x = len_x;
    grid->len_y = len_y;
    grid->cell_size = cell_size;
    grid->num_rows = (int)(len_y / cell_size);
    grid->num_cols = (int)(len_x / cell_size);
    
    grid->gridData = (int***)malloc(sizeof(int**) * grid->num_rows);
    for (int i = 0; i < grid->num_rows; i++) {
        grid->gridData[i] = (int**)malloc(sizeof(int*) * grid->num_cols);
        for (int j = 0; j < grid->num_cols; j++) {
            grid->gridData[i][j] = NULL;
        }
    }
    return grid;
}

void destroyGrid(IntArrayGrid* grid) {
    for (int i = 0; i < grid->num_rows; i++) {
        for (int j = 0; j < grid->num_cols; j++) {
            // Libérer la mémoire de chaque celulle (tableau dynamique d'entiers)
            free(grid->gridData[i][j]);
        }
        // Libérer la mémoire de la ligne (tableau de pointeurs)
        free(grid->gridData[i]);
    }
    // Libérer la mémoire allouée pour la grille (tableau de pointeurs de pointeurs)
    free(grid->gridData);
    // Libérer la mémoire allouée pour la structure Grid
    free(grid);
}

void setCellValues(IntArrayGrid* grid, int row, int col, int* values) {
    int num_values = values[0];
    if (row >= 0 && row < grid->num_rows && col >= 0 && col < grid->num_cols) {
        // Libérer la mémoire de l'ancien tableau dynamique (si il existe)
        if (grid->gridData[row][col] != NULL) {
            free(grid->gridData[row][col]);
        }
        // Allouer un nouveau tableau dynamique de la taille appropriée, en ajoutant 1 pour stocker la taille
        int new_num_values = num_values + 1;
        int* new_values = (int*)malloc(sizeof(int) * (new_num_values + 1));
        // Stocker la taille du tableau dans la première case
        new_values[0] = new_num_values;
        // Copier les valeurs dans le nouveau tableau
        for (int i = 0; i < num_values; i++) {
            new_values[i + 1] = values[i];
        }
        // Affecter le nouveau tableau à la cellule de la grille
        grid->gridData[row][col] = new_values;
    }
}

void addCellValue(IntArrayGrid* grid, int row, int col, int value) {
    if (row >= 0 && row < grid->num_rows && col >= 0 && col < grid->num_cols) {
        int* cell_values = grid->gridData[row][col];
        int num_values = getNumCellValues(grid, row, col);
        
        int* new_values = (int*)realloc(cell_values, sizeof(int) * (num_values + 2));
        new_values[num_values] = value;
        new_values[0] = num_values + 1;
        grid->gridData[row][col] = new_values;
    }
}


int* getCellValues(IntArrayGrid* grid, int row, int col) {
    if (row >= 0 && row < grid->num_rows && col >= 0 && col < grid->num_cols) {
        return grid->gridData[row][col];
    }
    return NULL;
}

int getNumCellValues(IntArrayGrid* grid, int row, int col) {
    if (row >= 0 && row < grid->num_rows && col >= 0 && col < grid->num_cols) {
        int* cell_values = grid->gridData[row][col];
        if (cell_values != NULL) {
            return cell_values[0]; // La première case contient la taille du tableau
        }
    }
    return 0;
}

void printGrid(IntArrayGrid* grid) {
    for (int row = 0; row < grid->num_rows; row++) {
        for (int col = 0; col < grid->num_cols; col++) {
            int* cell_values = grid->gridData[row][col];
            int num_values = getNumCellValues(grid, row, col);
            printf("[%d]", num_values);
        }
        printf("\n");
    }
    printf("\n");
}