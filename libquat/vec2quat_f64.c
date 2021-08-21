#include <stdint.h>
#include <math.h>
#include "quat.h"

quat_status vec2quat_f64(
	const vec_f64* pSrc, 
	quat_f64* pDst)
{
  quat_status status = QUAT_SUCCESS;
  quat_f64 result;
  quat_f64 q;

  q.w = 0.0;
  q.x = pSrc->x/2.0;
  q.y = pSrc->y/2.0;
  q.z = pSrc->z/2.0;

  double qvnorm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
  double z0 = (double) exp(q.w) * cos(qvnorm);

  pDst->w = q.w;

  if(qvnorm == 0.0)
  {
  	 pDst->x = 0.0;
	 pDst->y = 0.0;
	 pDst->z = 0.0;
  }
  else
  {
  	 pDst->x = (double) exp(q.w) * q.x/qvnorm * sin(qvnorm);
	 pDst->y = (double) exp(q.w) * q.y/qvnorm * sin(qvnorm);
	 pDst->z = (double) exp(q.w) * q.z/qvnorm * sin(qvnorm);
  }
  
  return (status);
}
