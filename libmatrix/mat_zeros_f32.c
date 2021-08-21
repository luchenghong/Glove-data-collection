#include <stdint.h>
#include "matrix.h"

mat_status mat_zeros_f32(
  mat_instance_f32 * pSrc)
{                 
  mat_status status = MAT_SUCCESS;
  int i,j;
  
  for(i=0; i<pSrc->rows; i++)
  	for(j=0; j<pSrc->cols; j++)
		pSrc->pData[i*pSrc->rows + j] = 0.0;

  return (status);
}


