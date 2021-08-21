#include <stdint.h>
#include <math.h>
#include "quat.h"

quat_status quat2vec_f32(
	const quat_f32* pSrc, 
	vec_f32* pDst)
{
  quat_status status = QUAT_SUCCESS;

  float qnorm = sqrt(pSrc->w*pSrc->w + pSrc->x*pSrc->x + pSrc->y*pSrc->y + pSrc->z*pSrc->z);
  float qvnorm = sqrt(pSrc->x*pSrc->x + pSrc->y*pSrc->y + pSrc->z*pSrc->z);

  if(qvnorm == 0.0)
  {
  	 pDst->x = 0.0;
	 pDst->y = 0.0;
	 pDst->z = 0.0;
  }
  else
  {
  	 pDst->x = (float) pSrc->x /qvnorm * acos(pSrc->w/qnorm) * 2.0;
	 pDst->y = (float) pSrc->y /qvnorm * acos(pSrc->w/qnorm) * 2.0;
	 pDst->z = (float) pSrc->z /qvnorm * acos(pSrc->w/qnorm) * 2.0;
  }

  return (status);
}
