#ifndef vector_h
#define vector_h
typedef struct myvector
{
  float x, y, z;
} myvector;

extern void vector_cross(const myvector *a, const myvector *b, myvector *out);
extern float vector_dot(const myvector *a,const myvector *b);
extern void vector_normalize(myvector *a);
#endif