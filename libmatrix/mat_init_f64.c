#include <stdint.h>
#include "matrix.h"

void mat_init_f64(
  mat_instance_f64 * S,
  uint16_t nRows,
  uint16_t nColumns,
  double * pData)
{
  /* Assign Number of Rows */
  S->rows = nRows;

  /* Assign Number of Columns */
  S->cols = nColumns;

  /* Assign Data pointer */
  S->pData = pData;
}
