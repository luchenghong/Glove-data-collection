#include <stdint.h>
#include "matrix.h"

mat_status mat_inverse_f32(
  const mat_instance_f32 * pSrc,
  mat_instance_f32 * pDst)
{
    float *pIn = pSrc->pData;                  /* input data matrix pointer */
    float *pOut = pDst->pData;                 /* output data matrix pointer */
    float *pInT1, *pInT2;                      /* Temporary input data matrix pointer */
    float *pOutT1, *pOutT2;                    /* Temporary output data matrix pointer */
    float *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst;  /* Temporary input and output data matrix pointer */
    uint32_t numRows = pSrc->rows;              /* Number of rows in the matrix  */
    uint32_t numCols = pSrc->cols;              /* Number of Cols in the matrix  */

    float Xchg, in = 0.0f;                     /* Temporary input values  */
    uint32_t i, rowCnt, flag = 0u, j, loopCnt, k, l;      /* loop counters */
    mat_status status;                             /* status of matrix inverse */

    /* Check for matrix mismatch condition */
    if((pSrc->rows != pSrc->cols) || (pDst->rows != pDst->cols)
      || (pSrc->rows != pDst->rows))
    {
      /* Set status as ARM_MATH_SIZE_MISMATCH */
      status = MAT_SIZE_MISMATCH;
    }
    else
    {

     /*--------------------------------------------------------------------------------------------------------------
      * Matrix Inverse can be solved using elementary row operations.
      *
      *	Gauss-Jordan Method:
      *
      *	   1. First combine the identity matrix and the input matrix separated by a bar to form an
      *        augmented matrix as follows:
      *				        _  _	      _	    _	   _   _         _	       _
      *					   |  |  a11  a12  | | | 1   0  |   |       |  X11 X12  |
      *					   |  |            | | |        |   |   =   |           |
      *					   |_ |_ a21  a22 _| | |_0   1 _|  _|       |_ X21 X21 _|
      *
      *		2. In our implementation, pDst Matrix is used as identity matrix.
      *
      *		3. Begin with the first row. Let i = 1.
      *
      *	    4. Check to see if the pivot for row i is zero.
      *		   The pivot is the element of the main diagonal that is on the current row.
      *		   For instance, if working with row i, then the pivot element is aii.
      *		   If the pivot is zero, exchange that row with a row below it that does not
      *		   contain a zero in column i. If this is not possible, then an inverse
      *		   to that matrix does not exist.
      *
      *	    5. Divide every element of row i by the pivot.
      *
      *	    6. For every row below and  row i, replace that row with the sum of that row and
      *		   a multiple of row i so that each new element in column i below row i is zero.
      *
      *	    7. Move to the next row and column and repeat steps 2 through 5 until you have zeros
      *		   for every element below and above the main diagonal.
      *
      *		8. Now an identical matrix is formed to the left of the bar(input matrix, src).
      *		   Therefore, the matrix to the right of the bar is our solution(dst matrix, dst).
      *----------------------------------------------------------------------------------------------------------------*/

     /* Working pointer for destination matrix */
     pOutT1 = pOut;

     /* Loop over the number of rows */
     rowCnt = numRows;

     /* Making the destination matrix as identity matrix */
     while(rowCnt > 0u)
     {
       /* Writing all zeroes in lower triangle of the destination matrix */
       j = numRows - rowCnt;
       while(j > 0u)
       {
         *pOutT1++ = 0.0f;
         j--;
       }

       /* Writing all ones in the diagonal of the destination matrix */
       *pOutT1++ = 1.0f;

       /* Writing all zeroes in upper triangle of the destination matrix */
       j = rowCnt - 1u;
       while(j > 0u)
       {
         *pOutT1++ = 0.0f;
         j--;
       }

       /* Decrement the loop counter */
       rowCnt--;
     }

     /* Loop over the number of columns of the input matrix.
        All the elements in each column are processed by the row operations */
     loopCnt = numCols;

     /* Index modifier to navigate through the columns */
     l = 0u;
     //for(loopCnt = 0u; loopCnt < numCols; loopCnt++)
     while(loopCnt > 0u)
     {
       /* Check if the pivot element is zero..
        * If it is zero then interchange the row with non zero row below.
        * If there is no non zero element to replace in the rows below,
        * then the matrix is Singular. */

       /* Working pointer for the input matrix that points
        * to the pivot element of the particular row  */
       pInT1 = pIn + (l * numCols);

       /* Working pointer for the destination matrix that points
        * to the pivot element of the particular row  */
       pOutT1 = pOut + (l * numCols);

       /* Temporary variable to hold the pivot value */
       in = *pInT1;

       /* Destination pointer modifier */
       k = 1u;

       /* Check if the pivot element is zero */
       if(*pInT1 == 0.0f)
       {
         /* Loop over the number rows present below */
         for (i = (l + 1u); i < numRows; i++)
         {
           /* Update the input and destination pointers */
           pInT2 = pInT1 + (numCols * l);
           pOutT2 = pOutT1 + (numCols * k);

           /* Check if there is a non zero pivot element to
            * replace in the rows below */
           if(*pInT2 != 0.0f)
           {
             /* Loop over number of columns
              * to the right of the pilot element */
             for (j = 0u; j < (numCols - l); j++)
             {
               /* Exchange the row elements of the input matrix */
               Xchg = *pInT2;
               *pInT2++ = *pInT1;
               *pInT1++ = Xchg;
             }

             for (j = 0u; j < numCols; j++)
             {
               Xchg = *pOutT2;
               *pOutT2++ = *pOutT1;
               *pOutT1++ = Xchg;
             }

             /* Flag to indicate whether exchange is done or not */
             flag = 1u;

             /* Break after exchange is done */
             break;
           }

           /* Update the destination pointer modifier */
           k++;
         }
       }

       /* Update the status if the matrix is singular */
       if((flag != 1u) && (in == 0.0f))
       {
         return MAT_SINGULAR;
       }

       /* Points to the pivot row of input and destination matrices */
       pPivotRowIn = pIn + (l * numCols);
       pPivotRowDst = pOut + (l * numCols);

       /* Temporary pointers to the pivot row pointers */
       pInT1 = pPivotRowIn;
       pOutT1 = pPivotRowDst;

       /* Pivot element of the row */
       in = *(pIn + (l * numCols));

       /* Loop over number of columns
        * to the right of the pilot element */
       for (j = 0u; j < (numCols - l); j++)
       {
         /* Divide each element of the row of the input matrix
          * by the pivot element */
         *pInT1 = *pInT1 / in;
         pInT1++;
       }
       for (j = 0u; j < numCols; j++)
       {
         /* Divide each element of the row of the destination matrix
          * by the pivot element */
         *pOutT1 = *pOutT1 / in;
         pOutT1++;
       }

       /* Replace the rows with the sum of that row and a multiple of row i
        * so that each new element in column i above row i is zero.*/

       /* Temporary pointers for input and destination matrices */
       pInT1 = pIn;
       pOutT1 = pOut;

       for (i = 0u; i < numRows; i++)
       {
         /* Check for the pivot element */
         if(i == l)
         {
           /* If the processing element is the pivot element,
              only the columns to the right are to be processed */
           pInT1 += numCols - l;
           pOutT1 += numCols;
         }
         else
         {
           /* Element of the reference row */
           in = *pInT1;

           /* Working pointers for input and destination pivot rows */
           pPRT_in = pPivotRowIn;
           pPRT_pDst = pPivotRowDst;

           /* Loop over the number of columns to the right of the pivot element,
              to replace the elements in the input matrix */
           for (j = 0u; j < (numCols - l); j++)
           {
             /* Replace the element by the sum of that row
                and a multiple of the reference row  */
             *pInT1 = *pInT1 - (in * *pPRT_in++);
             pInT1++;
           }
           /* Loop over the number of columns to
              replace the elements in the destination matrix */
           for (j = 0u; j < numCols; j++)
           {
             /* Replace the element by the sum of that row
                and a multiple of the reference row  */
             *pOutT1 = *pOutT1 - (in * *pPRT_pDst++);
             pOutT1++;
           }

         }
         /* Increment the temporary input pointer */
         pInT1 = pInT1 + l;
       }
       /* Increment the input pointer */
       pIn++;

       /* Decrement the loop counter */
       loopCnt--;
       /* Increment the index modifier */
       l++;
     }

     /* Set status as ARM_MATH_SUCCESS */
     status = MAT_SUCCESS;

     if((flag != 1u) && (in == 0.0))
     {
       pIn = pSrc->pData;
       for (i = 0; i < numRows * numCols; i++)
       {
         if (pIn[i] != 0.0f)
             break;
       }

       if (i == numRows * numCols)
         status = MAT_SINGULAR;
     }
    }

    return (status);
}
