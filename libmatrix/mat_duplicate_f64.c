#include <stdint.h>
#include "matrix.h"

mat_status mat_duplicate_f64(
  const mat_instance_f64 * pSrc,
  mat_instance_f64 * pDst)
{
  double *pIn = pSrc->pData;                  /* input data matrix pointer */
  double *pOut = pDst->pData;                 /* output data matrix pointer */
  uint32_t numSamples;                           /* total number of elements in the matrix */
  uint32_t blkCnt;                               /* loop counters */
  mat_status status;                             /* status of matrix scaling     */

  /* Check for matrix mismatch condition */
  if((pSrc->rows != pDst->rows) || (pSrc->cols != pDst->cols))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = MAT_SIZE_MISMATCH;
  }
  else
  {
    /* Total number of samples in the input matrix */
    numSamples = (uint32_t) pSrc->rows * pSrc->cols;

    /* Initialize blkCnt with number of samples */
    blkCnt = numSamples;

    while(blkCnt > 0u)
    {
      /* C(m,n) = A(m,n) * scale */
      /* The results are stored in the destination buffer. */
      *pOut++ = (*pIn++);

      /* Decrement the loop counter */
      blkCnt--;
    }

    /* Set status as ARM_MATH_SUCCESS */
    status = MAT_SUCCESS;
  }

  return (status);
}
