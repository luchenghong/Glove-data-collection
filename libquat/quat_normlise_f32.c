#include <stdint.h>
#include <math.h>
#include "quat.h"

quat_status quat_normlise_f32(
	quat_f32* pSrc)
{
  quat_status status = QUAT_SUCCESS;

  float qnorm = sqrt(pSrc->w*pSrc->w + pSrc->x*pSrc->x + pSrc->y*pSrc->y + pSrc->z*pSrc->z);

  pSrc->w /= qnorm;
  pSrc->x /= qnorm;
  pSrc->y /= qnorm;
  pSrc->z /= qnorm;
  
  return (status);
}
