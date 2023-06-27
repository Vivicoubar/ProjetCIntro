#ifndef VEC2_H_
#define VEC2_H_

// ------------------------------------------------

typedef struct Vec2 {
  float x;
  float y;
} Vec2;

// ------------------------------------------------

Vec2 multiplyByScalar(Vec2 vector, int scalar);
Vec2 sumVector(Vec2 vector1, Vec2 vector2);
Vec2 substractVector(Vec2 vector1, Vec2 vector2);
int scalarProduct(Vec2 vector1, Vec2 vector2);
Vec2 normalize(Vec2 vector);

// ------------------------------------------------

#endif