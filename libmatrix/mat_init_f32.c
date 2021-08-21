#include <stdint.h>
#include "matrix.h"

void mat_init_f32(
  mat_instance_f32 * S,
  uint16_t nRows,
  uint16_t nColumns,
  float * pData)
{
  /* Assign Number of Rows */
  S->rows = nRows;

  /* Assign Number of Columns */
  S->cols = nColumns;

  /* Assign Data pointer */
  S->pData = pData;
}
