#include "helper_3dmath.h"


void Quaternion_reset (Quaternion_t *Quaternion) {
            Quaternion->w = 1.0f;
            Quaternion->x = 0.0f;
            Quaternion->y = 0.0f;
            Quaternion->z = 0.0f;
        }
void Quaternion_set(Quaternion_t *Quaternion,float nw, float nx, float ny, float nz) {
            Quaternion->w = nw;
            Quaternion->x = nx;
            Quaternion->y = ny;
            Quaternion->z = nz;
        }	
void Quaternion_getProduct(Quaternion_t *Quaternion, Quaternion_t q) {
            // Quaternion multiplication is defined by:
            //     (Q1  *  Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1  *  Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1  *  Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1  *  Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
						Quaternion_t q1 = *Quaternion;
            Quaternion_set(Quaternion,
                q1.w * q.w - q1.x * q.x - q1.y * q.y - q1.z * q.z,  // new w
                q1.w * q.x + q1.x * q.w + q1.y * q.z - q1.z * q.y,  // new x
                q1.w * q.y - q1.x * q.z + q1.y * q.w + q1.z * q.x,  // new y
                q1.w * q.z + q1.x * q.y - q1.y * q.x + q1.z * q.w); // new z
        }
void Quaternion_getConjugate(Quaternion_t * Quaternion) {
						Quaternion_t q1 = *Quaternion;
            Quaternion_set(Quaternion,q1.w, -q1.x, -q1.y, -q1.z);
        }	
float Quaternion_getMagnitude(Quaternion_t *Quaternion) {
            return sqrt(Quaternion->w * Quaternion->w + Quaternion->x * Quaternion->x + Quaternion->y * Quaternion->y + Quaternion->z * Quaternion->z);
        }				
void Quaternion_normalize(Quaternion_t *Quaternion) {
            float m = Quaternion_getMagnitude(Quaternion);
            Quaternion->w /= m;
            Quaternion->x /= m;
            Quaternion->y /= m;
            Quaternion->z /= m;
        }	
Quaternion_t Quaternion_getNormalized(Quaternion_t Quaternion) {
            Quaternion_normalize(&Quaternion);
						return Quaternion;
        }


	
void VectorInt16_reset (VectorInt16_t *VectorInt16) {
            VectorInt16->x = 0;
            VectorInt16->y = 0;
						VectorInt16->z = 0;
        }
void VectorInt16_set(VectorInt16_t *VectorInt16,int16_t nx, int16_t ny, int16_t nz) {
            VectorInt16->x = nx;
            VectorInt16->y = ny;
            VectorInt16->z = nz;
        }	
float VectorInt16_getMagnitude(VectorInt16_t *VectorInt16) {
            return sqrt(VectorInt16->x*VectorInt16->x + VectorInt16->y*VectorInt16->y + VectorInt16->z*VectorInt16->z);
        }

void VectorInt16_normalize(VectorInt16_t *VectorInt16) {
            float m = VectorInt16_getMagnitude(VectorInt16);
            VectorInt16->x /= m;
            VectorInt16->y /= m;
            VectorInt16->z /= m;
        }
        	
VectorInt16_t VectorInt16_getNormalized(VectorInt16_t VectorInt16) {
            VectorInt16_normalize(&VectorInt16);
						return VectorInt16;
        }
/* 
void VectorInt16_(VectorInt16_t * v , Quaternion_t *q) {
            // http://www.cprogramming.com/tutorial/3d/quaternions.html
            // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
            // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
            // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1
        
            // P_out = q * P_in * conj(q)
            // - P_out is the output vector
            // - q is the orientation quaternion
            // - P_in is the input vector (a*aReal)
            // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
						Quaternion_t p;
						Quaternion_set(&p,0,v->x,v->y,v->z);

            // quaternion multiplication: q * p, stored back in p
            p = q -> getProduct(p);

            // quaternion multiplication: p * conj(q), stored back in p
            p = p.getProduct(q -> getConjugate());

            // p quaternion is now [0, x', y', z']
            x = p.x;
            y = p.y;
            z = p.z;
        }				
				

*/


	
void VectorFloat_reset (VectorFloat_t *VectorFloat) {
            VectorFloat->x = 0;
            VectorFloat->y = 0;
						VectorFloat->z = 0;
        }
void VectorFloat_set(VectorFloat_t *VectorFloat,int16_t nx, int16_t ny, int16_t nz) {
            VectorFloat->x = nx;
            VectorFloat->y = ny;
            VectorFloat->z = nz;
        }	
float VectorFloat_getMagnitude(VectorFloat_t *VectorFloat) {
            return sqrt(VectorFloat->x*VectorFloat->x + VectorFloat->y*VectorFloat->y + VectorFloat->z*VectorFloat->z);
        }

void VectorFloat_normalize(VectorFloat_t *VectorFloat) {
            float m = VectorFloat_getMagnitude(VectorFloat);
            VectorFloat->x /= m;
            VectorFloat->y /= m;
            VectorFloat->z /= m;
        }
        	
VectorFloat_t VectorFloat_getNormalized(VectorFloat_t VectorFloat) {
            VectorFloat_normalize(&VectorFloat);
						return VectorFloat;
        }
/* 
void VectorFloat_(VectorFloat_t * v , Quaternion_t *q) {
            // http://www.cprogramming.com/tutorial/3d/quaternions.html
            // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
            // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
            // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1
        
            // P_out = q * P_in * conj(q)
            // - P_out is the output vector
            // - q is the orientation quaternion
            // - P_in is the input vector (a*aReal)
            // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
						Quaternion_t p;
						Quaternion_set(&p,0,v->x,v->y,v->z);

            // quaternion multiplication: q * p, stored back in p
            p = q -> getProduct(p);

            // quaternion multiplication: p * conj(q), stored back in p
            p = p.getProduct(q -> getConjugate());

            // p quaternion is now [0, x', y', z']
            x = p.x;
            y = p.y;
            z = p.z;
        }				
				

*/
				

