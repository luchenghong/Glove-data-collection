#ifndef QUAT_H
#define QUAT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  QUAT_SUCCESS = 0, 			   /**< No error */
} quat_status;

typedef struct{
	float roll; 
	float pitch; 
	float yaw; 
}euler_f32; //define euler angles

typedef struct{
	float R00,R01,R02; 
	float R10,R11,R12; 
	float R20,R21,R22; 
}rotm_f32; //define rotation of matrix, 3x3

typedef struct{
	float x; 
	float y; 
	float z; 
}vec_f32;

typedef struct{
	float w; //q0
	float x; //q1
	float y; //q2
	float z; //q3
}quat_f32;

typedef struct{
	double roll; 
	double pitch; 
	double yaw; 
}euler_f64; //define euler angles

typedef struct{
	double R00,R01,R02; 
	double R10,R11,R12; 
	double R20,R21,R22;  
}rotm_f64; //define rotation of matrix, 3x3

typedef struct{
	double x; 
	double y; 
	double z; 
}vec_f64;

typedef struct{
	double w; //q0
	double x; //q1
	double y; //q2
	double z; //q3
}quat_f64;

extern quat_status vec2quat_f32(const vec_f32* pSrc, quat_f32* pDst);
extern quat_status vec2quat_f64(const vec_f64* pSrc, quat_f64* pDst);
extern quat_status quat2vec_f32(const quat_f32* pSrc, vec_f32* pDst);
extern quat_status quat2vec_f64(const quat_f64* pSrc, vec_f64* pDst);
extern quat_status quat_normlise_f32(quat_f32* pSrc);
extern quat_status quat_normlise_f64(quat_f64* pSrc);
extern quat_status quat_conjugate_f32(const quat_f32* pSrc, quat_f32* pDst);
extern quat_status quat_conjugate_f64(const quat_f64* pSrc, quat_f64* pDst);
extern quat_status quat_inverse_f32(const quat_f32* pSrc, quat_f32* pDst);
extern quat_status quat_inverse_f64(const quat_f64* pSrc, quat_f64* pDst);
extern quat_status quat_mult_f32(const quat_f32* pSrcA, const quat_f32* pSrcB, quat_f32* pDst);
extern quat_status quat_mult_f64(const quat_f64* pSrcA, const quat_f64* pSrcB, quat_f64* pDst);
extern quat_status rotm2quat_f32(const rotm_f32* pSrc, quat_f32* pDst);
extern quat_status rotm2quat_f64(const rotm_f64* pSrc, quat_f64* pDst);
extern quat_status quat2rotm_f32(const quat_f32* pSrc, rotm_f32* pDst);
extern quat_status quat2rotm_f64(const quat_f64* pSrc, rotm_f64* pDst);
extern quat_status euler2quat_f32(const euler_f32* pSrc, quat_f32* pDst);
extern quat_status euler2quat_f64(const euler_f64* pSrc, quat_f64* pDst);
extern quat_status quat2euler_f32(const quat_f32* pSrc, euler_f32* pDst);
extern quat_status quat2euler_f64(const quat_f64* pSrc, euler_f64* pDst);
extern quat_status quat_rotate_f32(const quat_f32* pSrcA, const vec_f32* pSrcB, vec_f32* pDst);
extern quat_status quat_rotate_f64(const quat_f64* pSrc, const vec_f64* pSrcB, vec_f64* pDst);

#ifdef __cplusplus
}
#endif

#endif
