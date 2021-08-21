#include <stdint.h>
#include <math.h>
#include "quat.h"

quat_status quat_normlise_f64(
	quat_f64* pSrc)
{
  quat_status status = QUAT_SUCCESS;

  double qnorm = sqrt(pSrc->w*pSrc->w + pSrc->x*pSrc->x + pSrc->y*pSrc->y + pSrc->z*pSrc->z);

  pSrc->w /= qnorm;
  pSrc->x /= qnorm;
  pSrc->y /= qnorm;
  pSrc->z /= qnorm;
  
  return (status);
}
