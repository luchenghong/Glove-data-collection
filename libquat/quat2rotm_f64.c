#include <stdint.h>
#include <math.h>
#include "quat.h"

quat_status quat2rotm_f64(
	const quat_f64* pSrc, 
	rotm_f64* pDst)

{
  quat_status status = QUAT_SUCCESS;

  pDst->R00 = pSrc->w*pSrc->w + pSrc->x*pSrc->x - pSrc->y*pSrc->y - pSrc->z*pSrc->z;
  pDst->R01 = 2.0*(pSrc->x*pSrc->y - pSrc->w*pSrc->z);
  pDst->R02 = 2.0*(pSrc->w*pSrc->y - pSrc->x*pSrc->z);
  
  pDst->R10 = 2.0*(pSrc->x*pSrc->y + pSrc->w*pSrc->z);
  pDst->R11 = pSrc->w*pSrc->w - pSrc->x*pSrc->x + pSrc->y*pSrc->y - pSrc->z*pSrc->z;
  pDst->R12 = 2.0*(pSrc->y*pSrc->z - pSrc->w*pSrc->x);
  
  pDst->R20 = 2.0*(pSrc->x*pSrc->y - pSrc->w*pSrc->y);
  pDst->R21 = 2.0*(pSrc->w*pSrc->x + pSrc->y*pSrc->z);
  pDst->R22 = pSrc->w*pSrc->w - pSrc->x*pSrc->x - pSrc->y*pSrc->y + pSrc->z*pSrc->z;
  
  return (status);
}
