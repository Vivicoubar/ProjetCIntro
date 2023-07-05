#include "Vec2.h"
#include <math.h>

Vec2 vecScale(Vec2 vector, float scalar) {
  return (Vec2){vector.x * scalar, vector.y * scalar};
}

Vec2 vecSum(Vec2 vector1, Vec2 vector2) {
  return (Vec2){vector1.x + vector2.x, vector1.y + vector2.y};
}

Vec2 vecSubstract(Vec2 vector1, Vec2 vector2) {
  return (Vec2){vector1.x - vector2.x, vector1.y - vector2.y};
}

float dotProduct(Vec2 vector1, Vec2 vector2) {
  return (vector1.x * vector2.x) + (vector1.y * vector2.y);
}

float norm(Vec2 vector) {
  return sqrt(dotProduct(vector, vector));
}

Vec2 vecNormalize(Vec2 vector) {
  float vec_norm = norm(vector);
  return (Vec2){vector.x / vec_norm, vector.y / vec_norm};
}

Vec2 vecClockwiseNormal(Vec2 vector) {
  return (Vec2){-vector.y, vector.x};
}
