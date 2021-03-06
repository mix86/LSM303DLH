#include <myvector.h>
#include <math.h>

void vector_cross(const myvector *a,const myvector *b, myvector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float vector_dot(const myvector *a,const myvector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void vector_normalize(myvector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}