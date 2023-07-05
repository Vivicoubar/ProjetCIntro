#ifndef VEC2_H_
#define VEC2_H_

typedef struct Vec2 {
  float x;
  float y;
} Vec2;

Vec2 vecScale(Vec2 vector, float scalar);
Vec2 vecSum(Vec2 vector1, Vec2 vector2);
Vec2 vecSubstract(Vec2 vector1, Vec2 vector2);
float dotProduct(Vec2 vector1, Vec2 vector2);
float norm(Vec2 vector);
Vec2 vecNormalize(Vec2 vector);
Vec2 vecClockwiseNormal(Vec2 vector);

#endif