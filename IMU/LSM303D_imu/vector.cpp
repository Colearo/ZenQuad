#include <vector.h>
#include <math.h>

void vector_cross(const vectors *a,const vectors *b, vectors *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float vector_dot(const vectors *a,const vectors *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void vector_normalize(vectors *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

void vector_norm_xz(vectors*a)
{
 float mag = sqrt((a->x*a->x) + (a->z*a->z));
 a->x /= mag;
 a->y  = 0;
 a->z /= mag;
 
}

void vector_norm_xy(vectors*a)
{
 float mag = sqrt(a->x*a->x + a->y*a->y);
 a->x /= mag;
 a->y /= mag;
 a->z  = 0;
} 

void vector_norm_yz(vectors*a)
{
 float mag = sqrt(a->z*a->z + a->y*a->y);
 a->x  = 0;
 a->y /= mag;
 a->z /= mag;
}   