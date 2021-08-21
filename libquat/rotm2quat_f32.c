#include <stdint.h>
#include <math.h>
#include "quat.h"

quat_status rotm2quat_f32(
	const rotm_f32* pSrc, 
	quat_f32* pDst)
{
  quat_status status = QUAT_SUCCESS;

  pDst->w = sqrt(1.0 + pSrc->R00 + pSrc->R11 + pSrc->R22) / 2.0;
  pDst->x = (pSrc->R21 - pSrc->R12)/ (4.0*pDst->w);
  pDst->y = (pSrc->R02 - pSrc->R20)/ (4.0*pDst->w);
  pDst->z = (pSrc->R10 - pSrc->R01)/ (4.0*pDst->w);

  return (status);
}
