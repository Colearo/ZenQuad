#ifndef vector_h
#define vector_h
typedef struct vectors
{
  float x, y, z;
} vectors;

extern void vector_cross(const vectors *a, const vectors *b, vectors *out);
extern float vector_dot(const vectors *a,const vectors *b);
extern void vector_normalize(vectors *a);
extern void vector_norm_xz(vectors*a);
extern void vector_norm_xy(vectors*a);
extern void vector_norm_yz(vectors*a);
#endif