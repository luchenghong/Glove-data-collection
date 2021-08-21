#include <stdint.h>
#include "quat.h"

quat_status quat_conjugate_f64(
	const quat_f64* pSrc, 
	quat_f64* pDst)
{
  quat_status status = QUAT_SUCCESS;

  pDst->w = pSrc->w;
  pDst->x = -pSrc->x;
  pDst->y = -pSrc->y;
  pDst->z = -pSrc->z;

  return (status);
}
