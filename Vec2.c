#include "Vec2.h"
#include <math.h>

// ------------------------------------------------


Vec2 multiplyByScalar(Vec2 vector, float scalar) {
    Vec2 res = {vector.x*scalar, vector.y*scalar};
    return res;
}

Vec2 sumVector(Vec2 vector1, Vec2 vector2) {
    Vec2 res = {vector1.x+vector2.x, vector1.y+vector2.y};
    return res;
}

Vec2 substractVector(Vec2 vector1, Vec2 vector2) {
    Vec2 res = {vector1.x-vector2.x, vector1.y-vector2.y};
    return res;
}

float scalarProduct(Vec2 vector1, Vec2 vector2) {
    return vector1.x*vector2.x + vector1.y*vector2.y;
}

Vec2 normalize(Vec2 vector) {
    Vec2 res = multiplyByScalar(vector, 1.0F/sqrt(scalarProduct(vector, vector)));
    return res;
}


// ------------------------------------------------
