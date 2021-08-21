#include <stdint.h>
#include "quat.h"

quat_status quat_rotate_f64(
	const quat_f64* pSrcA, 
	const vec_f64* pSrcB, 
	vec_f64* pDst)
{
  quat_status status = QUAT_SUCCESS;

  double R00, R01, R02, R10, R11, R12, R20, R21, R22;

  R00 = pSrcA->w*pSrcA->w + pSrcA->x*pSrcA->x - pSrcA->y*pSrcA->y - pSrcA->z*pSrcA->z;
  R01 = 2.0*(pSrcA->x*pSrcA->y - pSrcA->w*pSrcA->z);
  R02 = 2.0*(pSrcA->w*pSrcA->y - pSrcA->x*pSrcA->z);

  R10 = 2.0*(pSrcA->x*pSrcA->y + pSrcA->w*pSrcA->z);
  R11 = pSrcA->w*pSrcA->w - pSrcA->x*pSrcA->x + pSrcA->y*pSrcA->y - pSrcA->z*pSrcA->z;
  R12 = 2.0*(pSrcA->y*pSrcA->z - pSrcA->w*pSrcA->x);

  R20 = 2.0*(pSrcA->x*pSrcA->y - pSrcA->w*pSrcA->y);
  R21 = 2.0*(pSrcA->w*pSrcA->x + pSrcA->y*pSrcA->z);
  R22 = pSrcA->w*pSrcA->w - pSrcA->x*pSrcA->x - pSrcA->y*pSrcA->y + pSrcA->z*pSrcA->z;

  pDst->x = R00 * pSrcB->x + R01 * pSrcB->y + R02 * pSrcB->z;
  pDst->y = R10 * pSrcB->x + R11 * pSrcB->y + R12 * pSrcB->z;
  pDst->z = R20 * pSrcB->x + R21 * pSrcB->y + R22 * pSrcB->z;

  return (status);
}
