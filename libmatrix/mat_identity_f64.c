#include <stdint.h>
#include "matrix.h"

mat_status mat_identity_f64(
  mat_instance_f64 * pSrc)
{                 
  mat_status status = MAT_SUCCESS;
  int i;
  
  for(i=0; i<pSrc->rows; i++)
		pSrc->pData[i*pSrc->rows + i] = 1.0;

  return (status);
}


