#ifndef INTARRAYGRID_H_
#define INTARRAYGRID_H_

typedef struct IntArrayGrid {
    float len_x;
    float len_y;
    float cell_size;
    int** gridData;
    int num_rows;
    int num_cols;
} IntArrayGrid;

IntArrayGrid* createGrid(float len_x, float len_y, float cell_size);
void destroyGrid(IntArrayGrid* grid);
void clearGrid(IntArrayGrid* grid);
int* getCellValues(IntArrayGrid* grid, int row, int col);
int getNumberCellValues(IntArrayGrid* grid, int row, int col);
void setCellValues(IntArrayGrid* grid, int row, int col, int* values);
void addCellValue(IntArrayGrid* grid, int row, int col, int value);
void printGridNumberValues(IntArrayGrid* grid);
void printGrid(IntArrayGrid* grid);

#endif