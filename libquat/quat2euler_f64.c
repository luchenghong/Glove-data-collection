#include <stdint.h>
#include <math.h>
#include "quat.h"

quat_status quat2euler_f64(
	const quat_f64* pSrc, 
	euler_f64* pDst)
{
  quat_status status = QUAT_SUCCESS;

  pDst->roll = 180.0/M_PI * atan2(2.0*(pSrc->w*pSrc->x + pSrc->y*pSrc->z), 1.0 - 2.0*(pSrc->x*pSrc->x + pSrc->y*pSrc->y));

  pDst->pitch = 180.0/M_PI * asin(2.0*(pSrc->w*pSrc->y - pSrc->z*pSrc->x));

  pDst->yaw = 180.0/M_PI * atan2(2.0*(pSrc->w*pSrc->z + pSrc->x*pSrc->y), 1.0 - 2.0*(pSrc->y*pSrc->y + pSrc->z*pSrc->z));
  
  return (status);
}
