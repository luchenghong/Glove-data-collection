#include <stdint.h>
#include <math.h>
#include "quat.h"

quat_status quat2vec_f64(
	const quat_f64* pSrc, 
	vec_f64* pDst)
{
  quat_status status = QUAT_SUCCESS;

  double qnorm = sqrt(pSrc->w*pSrc->w + pSrc->x*pSrc->x + pSrc->y*pSrc->y + pSrc->z*pSrc->z);
  double qvnorm = sqrt(pSrc->x*pSrc->x + pSrc->y*pSrc->y + pSrc->z*pSrc->z);

  if(qvnorm == 0.0)
  {
  	 pDst->x = 0.0;
	 pDst->y = 0.0;
	 pDst->z = 0.0;
  }
  else
  {
  	 pDst->x = (double) pSrc->x /qvnorm * acos(pSrc->w/qnorm) * 2.0;
	 pDst->y = (double) pSrc->y /qvnorm * acos(pSrc->w/qnorm) * 2.0;
	 pDst->z = (double) pSrc->z /qvnorm * acos(pSrc->w/qnorm) * 2.0;
  }

  return (status);
}
