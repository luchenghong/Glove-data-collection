#include <stdint.h>
#include "quat.h"

quat_status quat_inverse_f32(
	const quat_f32* pSrc, 
	quat_f32* pDst)
{
  quat_status status = QUAT_SUCCESS;

  float qnorm2 = pSrc->w*pSrc->w + pSrc->x*pSrc->x + pSrc->y*pSrc->y + pSrc->z*pSrc->z;

  quat_conjugate_f32(pSrc,pDst);

  pDst->w /= qnorm2;
  pDst->x /= qnorm2;
  pDst->y /= qnorm2;
  pDst->z /= qnorm2;

  return (status);
}
