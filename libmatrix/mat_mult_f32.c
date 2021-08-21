#include <stdint.h>
#include "matrix.h"

mat_status mat_mult_f32(
  const mat_instance_f32 * pSrcA,
  const mat_instance_f32 * pSrcB,
  mat_instance_f32 * pDst)
{
  float *pIn1 = pSrcA->pData;                /* input data matrix pointer A */
  float *pIn2 = pSrcB->pData;                /* input data matrix pointer B */
  float *pInA = pSrcA->pData;                /* input data matrix pointer A  */
  float *pOut = pDst->pData;                 /* output data matrix pointer */
  float *px;                                 /* Temporary output data matrix pointer */
  float sum;                                 /* Accumulator */
  uint16_t numRowsA = pSrcA->rows;            /* number of rows of input matrix A */
  uint16_t numColsB = pSrcB->cols;            /* number of columns of input matrix B */
  uint16_t numColsA = pSrcA->cols;            /* number of columns of input matrix A */


  float *pInB = pSrcB->pData;                /* input data matrix pointer B */
  uint16_t col, i = 0u, row = numRowsA, colCnt;  /* loop counters */
  mat_status status;                             /* status of matrix multiplication */

  /* Check for matrix mismatch condition */
  if((pSrcA->cols != pSrcB->rows) ||
     (pSrcA->rows != pDst->rows) || (pSrcB->cols != pDst->cols))
  {
    status = MAT_SIZE_MISMATCH;
  }
  else
  {
    /* The following loop performs the dot-product of each row in pInA with each column in pInB */
    /* row loop */
    do
    {
      /* Output pointer is set to starting address of the row being processed */
      px = pOut + i;

      /* For every row wise process, the column loop counter is to be initiated */
      col = numColsB;

      /* For every row wise process, the pIn2 pointer is set
       ** to the starting address of the pSrcB data */
      pIn2 = pSrcB->pData;

      /* column loop */
      do
      {
        /* Set the variable sum, that acts as accumulator, to zero */
        sum = 0.0f;

        /* Initialize the pointer pIn1 to point to the starting address of the row being processed */
        pIn1 = pInA;

        /* Matrix A columns number of MAC operations are to be performed */
        colCnt = numColsA;

        while(colCnt > 0u)
        {
          /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;

          /* Decrement the loop counter */
          colCnt--;
        }

        /* Store the result in the destination buffer */
        *px++ = sum;

        /* Decrement the column loop counter */
        col--;

        /* Update the pointer pIn2 to point to the  starting address of the next column */
        pIn2 = pInB + (numColsB - col);

      } while(col > 0u);

      /* Update the pointer pInA to point to the  starting address of the next row */
      i = i + numColsB;
      pInA = pInA + numColsA;

      /* Decrement the row loop counter */
      row--;

    } while(row > 0u);
    /* Set status as ARM_MATH_SUCCESS */
    status = MAT_SUCCESS;
  }

  return (status);
}
