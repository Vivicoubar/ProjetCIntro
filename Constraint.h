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

typedef struct BoxCollider {
  Vec2 center;
  Vec2 director1;
  Vec2 director2;
} BoxCollider;


// ------------------------------------------------

#endif