#include <stdint.h>
#include "quat.h"

quat_status quat_mult_f64(
	const quat_f64* pSrcA, 
	const quat_f64* pSrcB, 
	quat_f64* pDst)
{
  quat_status status = QUAT_SUCCESS;

  pDst->w = pSrcA->w*pSrcB->w - pSrcA->x*pSrcB->x - pSrcA->y*pSrcB->y - pSrcA->z*pSrcB->z;
  pDst->x = pSrcA->w*pSrcB->x + pSrcA->x*pSrcB->w + pSrcA->y*pSrcB->z - pSrcA->z*pSrcB->y;
  pDst->y = pSrcA->w*pSrcB->y - pSrcA->x*pSrcB->z + pSrcA->y*pSrcB->w + pSrcA->z*pSrcB->x;
  pDst->z = pSrcA->w*pSrcB->z + pSrcA->x*pSrcB->y - pSrcA->y*pSrcB->x + pSrcA->z*pSrcB->w;

  return (status);
}
