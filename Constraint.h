#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

// ------------------------------------------------

typedef struct SphereCollider {
  Vec2 center;
  float radius;
} SphereCollider;

typedef struct PlaneCollider {
  Vec2 start_pos;
  Vec2 director;
} PlaneCollider;





// ------------------------------------------------

#endif