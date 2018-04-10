#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

/*  C++ detection  */
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <math.h>

typedef struct Quaternion_t {
        float w;
        float x;
        float y;
        float z;
} Quaternion_t;
	
		
typedef struct VectorInt16_t {
        int16_t x;
        int16_t y;
        int16_t z;
} VectorInt16_t;

typedef struct VectorFloat_t {
        float x;
        float y;
        float z;
} VectorFloat_t;

void Quaternion_reset (Quaternion_t *Quaternion);
void Quaternion_set(Quaternion_t *Quaternion,float nw, float nx, float ny, float nz);
void Quaternion_getProduct(Quaternion_t *Quaternion, Quaternion_t q);
void Quaternion_getConjugate(Quaternion_t * Quaternion);
float Quaternion_getMagnitude(Quaternion_t *Quaternion);
void Quaternion_normalize(Quaternion_t *Quaternion);
Quaternion_t Quaternion_getNormalized(Quaternion_t Quaternion);
void VectorInt16_reset (VectorInt16_t *VectorInt16);
void VectorInt16_set(VectorInt16_t *VectorInt16,int16_t nx, int16_t ny, int16_t nz);
float VectorInt16_getMagnitude(VectorInt16_t *VectorInt16);
void VectorInt16_normalize(VectorInt16_t *VectorInt16);
VectorInt16_t VectorInt16_getNormalized(VectorInt16_t VectorInt16);
void VectorFloat_reset (VectorFloat_t *VectorFloat);
void VectorFloat_set(VectorFloat_t *VectorFloat,int16_t nx, int16_t ny, int16_t nz);
float VectorFloat_getMagnitude(VectorFloat_t *VectorFloat);
void VectorFloat_normalize(VectorFloat_t *VectorFloat);
VectorFloat_t VectorFloat_getNormalized(VectorFloat_t VectorFloat) ;


	



/*   C++ detection  */
#ifdef __cplusplus
}
#endif

#endif
