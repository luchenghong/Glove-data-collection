#include <stdint.h>
#include "matrix.h"

mat_status mat_sub_f64(
  const mat_instance_f64 * pSrcA,
  const mat_instance_f64 * pSrcB,
  mat_instance_f64 * pDst)
{
  double *pIn1 = pSrcA->pData;                /* input data matrix pointer A */
  double *pIn2 = pSrcB->pData;                /* input data matrix pointer B */
  double *pOut = pDst->pData;                 /* output data matrix pointer  */

  uint32_t numSamples;                           /* total number of elements in the matrix  */
  uint32_t blkCnt;                               /* loop counters */
  mat_status status;                             /* status of matrix subtraction */

  /* Check for matrix mismatch condition */
  if((pSrcA->rows != pSrcB->rows ) ||
     (pSrcA->cols != pSrcB->cols) ||
     (pSrcA->rows  != pDst->rows ) || (pSrcA->cols != pDst->cols))
  {
    status = MAT_SIZE_MISMATCH;
  }
  else
  {
    /* Total number of samples in the input matrix */
    numSamples = (uint32_t) pSrcA->rows * pSrcA->cols;

    /* Initialize blkCnt with number of samples */
    blkCnt = numSamples;

    while(blkCnt > 0u)
    {
      /* C(m,n) = A(m,n) - B(m,n) */
      /* Subtract and then store the results in the destination buffer. */
      *pOut++ = (*pIn1++) - (*pIn2++);

      /* Decrement the loop counter */
      blkCnt--;
    }

    status = MAT_SUCCESS;
  }

  return (status);
}
