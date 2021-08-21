#include <stdint.h>
#include <math.h>
#include "quat.h"

quat_status vec2quat_f32(
	const vec_f32* pSrc, 
	quat_f32* pDst)
{
  quat_status status = QUAT_SUCCESS;
  quat_f32 result;
  quat_f32 q;

  q.w = 0.0;
  q.x = pSrc->x/2.0;
  q.y = pSrc->y/2.0;
  q.z = pSrc->z/2.0;

  float qvnorm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
  float z0 = (float) exp(q.w) * cos(qvnorm);

  pDst->w = q.w;

  if(qvnorm == 0.0)
  {
  	 pDst->x = 0.0;
	 pDst->y = 0.0;
	 pDst->z = 0.0;
  }
  else
  {
  	 pDst->x = (float) exp(q.w) * q.x/qvnorm * sin(qvnorm);
	 pDst->y = (float) exp(q.w) * q.y/qvnorm * sin(qvnorm);
	 pDst->z = (float) exp(q.w) * q.z/qvnorm * sin(qvnorm);
  }
  
  return (status);
}
