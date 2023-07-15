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
    
    grid->gridData = (int**)calloc(grid->num_rows * grid->num_cols, sizeof(int*));
    return grid;
}

void destroyGrid(IntArrayGrid* grid) {
    for (int i = 0; i < grid->num_rows; i++) {
        for (int j = 0; j < grid->num_cols; j++) {
            // Libérer la mémoire de chaque celulle (tableau dynamique d'entiers)
            free(grid->gridData[i * grid->num_cols + j]);
        }
    }
    // Libérer la mémoire allouée pour la grille (tableau de pointeurs de pointeurs)
    free(grid->gridData);
    // Libérer la mémoire allouée pour la structure Grid
    free(grid);
}

void clearGrid(IntArrayGrid* grid) {
    for (int row = 0; row < grid->num_rows; row++) {
        for (int col = 0; col < grid->num_cols; col++) {
            int* cell_values = grid->gridData[row * grid->num_cols + col];
            if (cell_values != NULL) {
                free(cell_values);
                grid->gridData[row * grid->num_cols + col] = NULL;
            }
        }
    }
}


int* getCellValues(IntArrayGrid* grid, int row, int col) {
    if (row >= 0 && row < grid->num_rows && col >= 0 && col < grid->num_cols) {
        return grid->gridData[row * grid->num_cols + col];
    }
    return NULL;
}

int getNumberCellValues(IntArrayGrid* grid, int row, int col) {
    if (row >= 0 && row < grid->num_rows && col >= 0 && col < grid->num_cols) {
        int* cell_values = grid->gridData[row * grid->num_cols + col];
        if (cell_values != NULL) {
            return cell_values[0]; // La première case contient la taille du tableau
        }
    }
    return 0;
}

void setCellValues(IntArrayGrid* grid, int row, int col, int* values) {
    int num_values = values[0];
    if (row >= 0 && row < grid->num_rows && col >= 0 && col < grid->num_cols) {
        int index = row * grid->num_cols + col;
        int* cell_values = grid->gridData[index];
        // Libérer la mémoire de l'ancien tableau dynamique (si il existe)
        if (cell_values != NULL) {
            free(grid->gridData[index]);
        }
        // Allouer un nouveau tableau dynamique de la taille appropriée, en ajoutant 1 pour stocker la taille
        int* new_values = (int*)malloc(sizeof(int) * (num_values + 1));
        // Stocker la taille du tableau dans la première case
        // Copier les valeurs dans le nouveau tableau
        for (int i = 0; i < num_values + 1; i++) {
            new_values[i] = values[i];
        }
        // Affecter le nouveau tableau à la cellule de la grille
        grid->gridData[index] = new_values;
    }
}

void addCellValue(IntArrayGrid* grid, int row, int col, int value) {
    if (row >= 0 && row < grid->num_rows && col >= 0 && col < grid->num_cols) {
        int index = row * grid->num_cols + col;
        int* cell_values = grid->gridData[index];
        
        if (cell_values == NULL) {
            cell_values = (int*)malloc(sizeof(int) * 2);
            cell_values[0] = 1;
            cell_values[1] = value;
        } else {
            int num_values = cell_values[0];
            int* new_values = (int*)realloc(cell_values, sizeof(int) * (num_values + 2));
            new_values[num_values + 1] = value;
            new_values[0] = num_values + 1;
            cell_values = new_values;
        }

        grid->gridData[index] = cell_values;
    }
}

void printGridNumberValues(IntArrayGrid* grid) {
    for (int row = 0; row < grid->num_rows; row++) {
        for (int col = 0; col < grid->num_cols; col++) {
            int* cell_values = grid->gridData[row * grid->num_cols + col];
            int num_values = getNumberCellValues(grid, row, col);
            printf("[%d]", num_values);
        }
        printf("\n");
    }
    printf("\n");
}

void printGrid(IntArrayGrid* grid) {
    for (int row = 0; row < grid->num_rows; row++) {
        for (int col = 0; col < grid->num_cols; col++) {
            int* cell_values = grid->gridData[row * grid->num_cols + col];
            int num_values = getNumberCellValues(grid, row, col);
            printf("[");
            if (cell_values != NULL) {
                for (int i = 1; i <= num_values; i++) {
                    printf("[%d]", cell_values[i]);
                }
            }
            printf("]");
        }
        printf("\n");
    }
    printf("\n");
}