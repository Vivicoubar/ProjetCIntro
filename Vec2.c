#include "Vec2.h"
#include <math.h>

Vec2 multiplyByScalar(Vec2 vector, float scalar) {
  return (Vec2){vector.x * scalar, vector.y * scalar};
}

Vec2 sumVector(Vec2 vector1, Vec2 vector2) {
  return (Vec2){vector1.x + vector2.x, vector1.y + vector2.y};
}

Vec2 substractVector(Vec2 vector1, Vec2 vector2) {
  return (Vec2){vector1.x - vector2.x, vector1.y - vector2.y};
}

float dotProduct(Vec2 vector1, Vec2 vector2) {
  return (vector1.x * vector2.x) + (vector1.y * vector2.y);
}

float norm(Vec2 vector) {
  return sqrt(dotProduct(vector, vector));
}

Vec2 normalize(Vec2 vector) {
  float vec_norm = norm(vector);
  return (Vec2){vector.x / vec_norm, vector.y / vec_norm};
}

Vec2 clockwiseNormal(Vec2 vector) {
  return (Vec2){-vector.y, vector.x};
}
