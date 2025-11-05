/*
 * File: fk_with_ofst_initialize.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-May-2025 11:02:17
 */

/* Include Files */
#include "fk_with_ofst_initialize.h"
#include "fk_with_ofst_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void fk_with_ofst_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_fk_with_ofst = true;
}

/*
 * File trailer for fk_with_ofst_initialize.c
 *
 * [EOF]
 */
