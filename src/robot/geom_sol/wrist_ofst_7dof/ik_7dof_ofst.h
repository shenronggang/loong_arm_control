/*
 * File: ik_7dof_ofst.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-May-2025 11:02:17
 */

#ifndef IK_7DOF_OFST_H
#define IK_7DOF_OFST_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void ik_7dof_ofst(double z_alpha, double y_beta, double x_gamma,
                         double p_x, double p_y, double p_z, double bet,
                         double cur_theta[7], int lt_or_rt, int LOrR,
                         int FOrB, double jntLim[14], double a[3],
                         double d[2], double theta[7], int *ik_state);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ik_7dof_ofst.h
 *
 * [EOF]
 */
