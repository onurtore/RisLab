/**************************************************************************
	Developed by: Chih-Hao Ho and Cagatay Basdogan
	Purpose: The math head file.
	Date: Apr. 22, 1997
	Laboratory for Human and Machine Haptics
	Massachusetts Institute of Technology
 **************************************************************************/
#ifndef _MATH_H_
#define _MATH_H_

#include <float.h>

#define XX 0
#define YY 1
#define ZZ 2
#define gravity 9.80665
#ifndef PI 
#define PI  3.1415926535897932384626433832795f
#endif
#define MINIMUM_DIVISOR (0.0000001)
#define ROUNDOFF_ERROR2 (0.00001)
#define zero_vector(force) { \
	force[0] = 0.0; \
	force[1] = 0.0; \
	force[2] = 0.0; \
}

// This looks like it should always be true, but it's false if x is a NaN.
#define IsNumber(x) (x == x)

#define IsFiniteNumber(x) (x <= DBL_MAX && x >= -DBL_MAX)

static double sqrarg;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)
#define SIGN(a,b) ((b) > 0.0 ? fabs(a) : -fabs(b))
#define SABS(x) ((x) > 0 ? (x):-(x))
#define DABS(a) ((a) > (double)0.0 ? (a):(-(a)))
#define SQUARE(x) ((x)*(x))
#define STRLEN(p) (strlen(p)+1)
#define SSIGN(a) ((a)>=0?(1.0):(-1.0))
#define MAKE_3DVECTOR(pt1,pt2,pt3) {pt3[XX]=pt2[XX]-pt1[XX];pt3[YY]=pt2[YY]-pt1[YY];pt3[ZZ]=pt2[ZZ]-pt1[ZZ];}
#define DOT3(u,v) (u[2]*v[2]+u[1]*v[1]+u[0]*v[0])
#define VECADD3(r,u,v) {r[2]=u[2]+v[2];r[1]=u[1]+v[1];r[0]=u[0]+v[0];}
#define VECSUB3(r,u,v) {r[2]=u[2]-v[2];r[1]=u[1]-v[1];r[0]=u[0]-v[0];}
#define CPVECTOR3 (u,v) {u[2]=v[2];u[1]=v[1];u[0]=v[0];}
#define VECNEGATE3(u) {u[XX]=(-u[XX]);u[YY]=(-u[YY]);u[ZZ]=(-u[ZZ]);}
#define MAX(a,b) ((a)>=(b)?(a):(b))
#define MIN(a,b) ((a)<=(b)?(a):(b))
#define VECTOR_MAGNITUDE(vec) (sqrt((vec[0]*vec[0])+(vec[1]*vec[1])+(vec[2]*vec[2])))
#define EQUAL_WITHIN_ERROR2(a,b) (DABS(((a)-(b)))<=ROUNDOFF_ERROR2)
#define DISTANCE3(u,v) (sqrt((u[0]-v[0])*(u[0]-v[0])+(u[1]-v[1])*(u[1]-v[1])+(u[2]-v[2])*(u[2]-v[2])))
#define SCALEVEC3(a,v) {v[2]*=a;v[1]*=a;v[0]*=a;}
#define VECCROSS3(r,u,v) {r[0]=u[1]*v[2]-u[2]*v[1];r[1]=u[2]*v[0]-u[0]*v[2];r[2]=u[0]*v[1]-u[1]*v[0];}
// ayse - add to radians here
#define TO_RADIANS(d) (d * PI /180.0f)
#define EUC_DIST(x1, y1, x2, y2) (sqrtf(SQUARE(x1 - x2) + SQUARE(y1 - y2)))

#endif