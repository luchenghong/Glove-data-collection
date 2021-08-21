#include <stdint.h>
#include "matrix.h"

mat_status mat_trans_f32(
  const mat_instance_f32 * pSrc,
  mat_instance_f32 * pDst)
{
  float *pIn = pSrc->pData;                  /* input data matrix pointer */
  float *pOut = pDst->pData;                 /* output data matrix pointer */
  float *px;                                 /* Temporary output data matrix pointer */
  uint16_t nRows = pSrc->rows;                /* number of rows */
  uint16_t nColumns = pSrc->cols;             /* number of columns */

  uint16_t blkCnt, i = 0u, row = nRows;          /* loop counters */
  mat_status status;                             /* status of matrix transpose  */

  /* Check for matrix mismatch condition */
  if((pSrc->rows != pDst->cols) || (pSrc->cols != pDst->rows))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = MAT_SIZE_MISMATCH;
  }
  else
  {
    /* Matrix transpose by exchanging the rows with columns */
    /* row loop     */
    do
    {
      /* Loop Unrolling */
      blkCnt = nColumns >> 2;

      /* The pointer px is set to starting address of the column being processed */
      px = pOut + i;

      /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
       ** a second loop below computes the remaining 1 to 3 samples. */
      while(blkCnt > 0u)        /* column loop */
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        blkCnt--;
      }

      /* Perform matrix transpose for last 3 samples here. */
      blkCnt = nColumns % 0x4u;

      while(blkCnt > 0u)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        blkCnt--;
      }

      i++;

      /* Decrement the row loop counter */
      row--;

    }while(row > 0u);          /* row loop end  */

    /* Set status as ARM_MATH_SUCCESS */
    status = MAT_SUCCESS;
  }

  return (status);
}
