/*
 * File: fk_with_ofst.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-May-2025 11:02:17
 */

/* Include Files */
#include "fk_with_ofst.h"
#include "fk_with_ofst_data.h"
#include "fk_with_ofst_initialize.h"
#include "fk_with_ofst_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double theta[7]
 *                int b_lt_or_rt
 *                const double a_arr[7]
 *                const double alpha_arr[7]
 *                const double d_arr[7]
 *                const double theta_arr[7]
 *                double carte[3]
 *                double eulVal[3]
 *                double *bet
 *                int *LOrR
 *                int *FOrB
 * Return Type  : void
 */
void fk_with_ofst(double theta[7], int b_lt_or_rt, double a_arr[7],
                  double alpha_arr[7], double d_arr[7],
                  double theta_arr[7], double carte[3], double eulVal[3],
                  double *bet, int *LOrR, int *FOrB)
{
  static const double dv[7] = {0.0,
                               -1.5707963267948966,
                               1.5707963267948966,
                               -1.5707963267948966,
                               1.5707963267948966,
                               -1.5707963267948966,
                               1.5707963267948966};
  static const double dv1[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963267948966,
                                0.0};
  static const signed char b_iv[4] = {0, 0, -1, 0};
  static const signed char b_iv1[4] = {0, 0, 0, 1};
  double TP6_inv[16];
  double Tz_theta_tmp[16];
  double forwardMat[16];
  double forwardMatWoOfst[16];
  double b_forwardMat[9];
  double m[9];
  double a_arr_woOfst[7];
  double cur_theta[7];
  double d_arr_woOfst[7];
  double BE;
  double BW;
  double EW;
  double a4;
  double a5;
  double a6_tmp;
  double absxk;
  double carte_idx_0;
  double carte_idx_1;
  double carte_idx_2;
  double dd2;
  double dd4;
  double p_idx_1;
  double scale;
  double t;
  double theta1;
  double theta2;
  double theta3;
  int TP6_inv_tmp;
  int b_i;
  int i;
  int i1;
  int i2;
  signed char forwardMat_tmp[16];
  (void)b_lt_or_rt;
  if (!isInitialized_fk_with_ofst) {
    fk_with_ofst_initialize();
  }
  for (i = 0; i < 7; i++) {
    cur_theta[i] = theta[i];
  }
  /*  if (b_lt_or_rt == int32(0)) */
  /*      theta(1) = theta(1) - pi; */
  /*      % 左臂关节1翻-pi直接是让关节2的值翻过来，与原来方向对应 */
  /*  else */
  /*      theta(1) = theta(1) + pi; */
  /*      % 右臂关节相对于左臂加上2*pi直接是为了避免负角度，从而超过设定的限位值
   */
  /*  end */
  *FOrB = -1;
  *LOrR = !(theta[3] < 0.0);
  if ((theta[4] >= -1.5707963267948966) && (theta[4] <= 1.5707963267948966)) {
    *FOrB = 1;
  } else if (((theta[4] > 1.5707963267948966) &&
              (theta[4] <= 3.1415926535897931)) ||
             ((theta[4] >= -3.1415926535897931) &&
              (theta[4] < -1.5707963267948966))) {
    *FOrB = 0;
  }
  for (i = 0; i < 16; i++) {
    forwardMat_tmp[i] = 0;
  }
  forwardMat_tmp[0] = 1;
  forwardMat_tmp[5] = 1;
  forwardMat_tmp[10] = 1;
  forwardMat_tmp[15] = 1;
  for (i = 0; i < 16; i++) {
    forwardMat[i] = forwardMat_tmp[i];
  }
  for (b_i = 0; b_i < 7; b_i++) {
    t = alpha_arr[b_i];
    BW = sin(t);
    EW = cos(t);
    BE = theta_arr[b_i] + theta[b_i];
    scale = sin(BE);
    BE = cos(BE);
    Tz_theta_tmp[1] = 0.0;
    Tz_theta_tmp[5] = EW;
    Tz_theta_tmp[9] = -BW;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[6] = BW;
    Tz_theta_tmp[10] = EW;
    Tz_theta_tmp[14] = 0.0;
    Tz_theta_tmp[0] = 1.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[4] = 0.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[12] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = forwardMat[i];
      EW = forwardMat[i + 4];
      BW = forwardMat[i + 8];
      absxk = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        TP6_inv[i + i2] = ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
                           BW * Tz_theta_tmp[i2 + 2]) +
                          absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    Tz_theta_tmp[0] = 1.0;
    Tz_theta_tmp[4] = 0.0;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[12] = a_arr[b_i];
    Tz_theta_tmp[1] = 0.0;
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[5] = 1.0;
    Tz_theta_tmp[6] = 0.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[9] = 0.0;
    Tz_theta_tmp[10] = 1.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[14] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = TP6_inv[i];
      EW = TP6_inv[i + 4];
      BW = TP6_inv[i + 8];
      absxk = TP6_inv[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
             BW * Tz_theta_tmp[i2 + 2]) +
            absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    Tz_theta_tmp[0] = BE;
    Tz_theta_tmp[4] = -scale;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[12] = 0.0;
    Tz_theta_tmp[1] = scale;
    Tz_theta_tmp[5] = BE;
    Tz_theta_tmp[9] = 0.0;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[6] = 0.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[10] = 1.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[14] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = forwardMat[i];
      EW = forwardMat[i + 4];
      BW = forwardMat[i + 8];
      absxk = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        TP6_inv[i + i2] = ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
                           BW * Tz_theta_tmp[i2 + 2]) +
                          absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[6] = 0.0;
    Tz_theta_tmp[10] = 1.0;
    Tz_theta_tmp[14] = d_arr[b_i];
    Tz_theta_tmp[0] = 1.0;
    Tz_theta_tmp[1] = 0.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[4] = 0.0;
    Tz_theta_tmp[5] = 1.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[9] = 0.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[12] = 0.0;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = TP6_inv[i];
      EW = TP6_inv[i + 4];
      BW = TP6_inv[i + 8];
      absxk = TP6_inv[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
             BW * Tz_theta_tmp[i2 + 2]) +
            absxk * Tz_theta_tmp[i2 + 3];
      }
    }
  }
  carte_idx_0 = forwardMat[12];
  carte[0] = forwardMat[12];
  carte_idx_1 = forwardMat[13];
  carte[1] = forwardMat[13];
  carte_idx_2 = forwardMat[14];
  carte[2] = forwardMat[14];
  /*  定义常量 */
  /*  初始化结果向量 */
  /*  将3x3矩阵按列优先展开为一维数组（MATLAB默认列优先存储） */
  if (fabs(forwardMat[2]) < 0.999999) {
    /*  对应原m[2]，MATLAB索引从1开始 */
    /*  正常情况（无万向节锁） */
    eulVal[0] = rt_atan2d_snf(forwardMat[1], forwardMat[0]);
    /*  Z轴旋转（原m[1]->m(2)） */
    eulVal[2] = rt_atan2d_snf(forwardMat[6], forwardMat[10]);
    /*  X轴旋转（原m[5]->m(6)） */
    if (fabs(forwardMat[6]) > 1.0E-6) {
      /*  原m[5]->m(6) */
      /*  使用正弦分量计算Y轴 */
      eulVal[1] = rt_atan2d_snf(-forwardMat[2], forwardMat[6] / sin(eulVal[2]));
    } else {
      /*  使用余弦分量计算Y轴 */
      eulVal[1] =
          rt_atan2d_snf(-forwardMat[2], forwardMat[10] / cos(eulVal[2]));
    }
  } else {
    /*  万向节锁情况（Y接近±90度） */
    eulVal[1] = asin(-forwardMat[2]);
    /*  Y轴旋转 */
    eulVal[0] = 0.0;
    /*  Z轴置零 */
    if (forwardMat[2] > 0.0) {
      eulVal[2] = rt_atan2d_snf(-forwardMat[9], forwardMat[5]);
      /*  原m[7]->m(8) */
    } else {
      eulVal[2] = -rt_atan2d_snf(forwardMat[9], forwardMat[5]);
      /*  原m[7]->m(8) */
    }
  }
  /*  调整输出顺序为[X, Y, Z] */
  BE = eulVal[2];
  EW = eulVal[1];
  BW = eulVal[0];
  eulVal[0] = BE;
  eulVal[1] = EW;
  eulVal[2] = BW;
  /*  计算臂型角 */
  theta[5] = -theta[5];
  a_arr_woOfst[0] = 0.0;
  a_arr_woOfst[1] = 0.0;
  a_arr_woOfst[2] = 0.0;
  a_arr_woOfst[3] = a_arr[3];
  a_arr_woOfst[4] = a_arr[4];
  a_arr_woOfst[5] = 0.0;
  a_arr_woOfst[6] = a_arr[6];
  d_arr_woOfst[0] = 0.0;
  d_arr_woOfst[1] = 0.0;
  d_arr_woOfst[2] = d_arr[2];
  d_arr_woOfst[3] = 0.0;
  d_arr_woOfst[4] = d_arr[4];
  d_arr_woOfst[5] = 0.0;
  d_arr_woOfst[6] = 0.0;
  for (i = 0; i < 16; i++) {
    forwardMatWoOfst[i] = forwardMat_tmp[i];
  }
  for (b_i = 0; b_i < 6; b_i++) {
    t = dv[b_i];
    BW = sin(t);
    EW = cos(t);
    BE = dv1[b_i] + theta[b_i];
    scale = sin(BE);
    BE = cos(BE);
    Tz_theta_tmp[1] = 0.0;
    Tz_theta_tmp[5] = EW;
    Tz_theta_tmp[9] = -BW;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[6] = BW;
    Tz_theta_tmp[10] = EW;
    Tz_theta_tmp[14] = 0.0;
    Tz_theta_tmp[0] = 1.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[4] = 0.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[12] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = forwardMatWoOfst[i];
      EW = forwardMatWoOfst[i + 4];
      BW = forwardMatWoOfst[i + 8];
      absxk = forwardMatWoOfst[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        TP6_inv[i + i2] = ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
                           BW * Tz_theta_tmp[i2 + 2]) +
                          absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    Tz_theta_tmp[0] = 1.0;
    Tz_theta_tmp[4] = 0.0;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[12] = a_arr_woOfst[b_i];
    Tz_theta_tmp[1] = 0.0;
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[5] = 1.0;
    Tz_theta_tmp[6] = 0.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[9] = 0.0;
    Tz_theta_tmp[10] = 1.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[14] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = TP6_inv[i];
      EW = TP6_inv[i + 4];
      BW = TP6_inv[i + 8];
      absxk = TP6_inv[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        forwardMatWoOfst[i + i2] =
            ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
             BW * Tz_theta_tmp[i2 + 2]) +
            absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    Tz_theta_tmp[0] = BE;
    Tz_theta_tmp[4] = -scale;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[12] = 0.0;
    Tz_theta_tmp[1] = scale;
    Tz_theta_tmp[5] = BE;
    Tz_theta_tmp[9] = 0.0;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[6] = 0.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[10] = 1.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[14] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = forwardMatWoOfst[i];
      EW = forwardMatWoOfst[i + 4];
      BW = forwardMatWoOfst[i + 8];
      absxk = forwardMatWoOfst[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        TP6_inv[i + i2] = ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
                           BW * Tz_theta_tmp[i2 + 2]) +
                          absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[6] = 0.0;
    Tz_theta_tmp[10] = 1.0;
    Tz_theta_tmp[14] = d_arr_woOfst[b_i];
    Tz_theta_tmp[0] = 1.0;
    Tz_theta_tmp[1] = 0.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[4] = 0.0;
    Tz_theta_tmp[5] = 1.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[9] = 0.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[12] = 0.0;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = TP6_inv[i];
      EW = TP6_inv[i + 4];
      BW = TP6_inv[i + 8];
      absxk = TP6_inv[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        forwardMatWoOfst[i + i2] =
            ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
             BW * Tz_theta_tmp[i2 + 2]) +
            absxk * Tz_theta_tmp[i2 + 3];
      }
    }
  }
  /* 先根据1~6轴关节角计算出不带7轴偏置部分的笛卡尔位姿 */
  /*     %% 解臂型角的思路，将7Dof带ofset的构型，重新逆解回不带offset的构型 */
  a6_tmp = fabs(a_arr[6]);
  forwardMat[0] = 1.0;
  forwardMat[4] = 0.0;
  forwardMat[8] = 0.0;
  forwardMat[12] = a6_tmp;
  forwardMat[1] = 0.0;
  forwardMat[2] = 0.0;
  forwardMat[3] = 0.0;
  forwardMat[5] = 1.0;
  forwardMat[6] = 0.0;
  forwardMat[7] = 0.0;
  forwardMat[9] = 0.0;
  forwardMat[10] = 1.0;
  forwardMat[11] = 0.0;
  forwardMat[13] = 0.0;
  forwardMat[14] = 0.0;
  forwardMat[15] = 1.0;
  /* 末端相当于沿着6系x方向前移了71，6系x方向是朝里的 */
  for (i = 0; i < 16; i++) {
    TP6_inv[i] = iv[i];
  }
  for (i = 0; i < 3; i++) {
    b_forwardMat[3 * i] = forwardMat[i];
    b_forwardMat[3 * i + 1] = forwardMat[i + 4];
    b_forwardMat[3 * i + 2] = forwardMat[i + 8];
  }
  memcpy(&m[0], &b_forwardMat[0], 9U * sizeof(double));
  for (i = 0; i < 3; i++) {
    TP6_inv_tmp = i << 2;
    TP6_inv[TP6_inv_tmp] = m[3 * i];
    TP6_inv[TP6_inv_tmp + 1] = m[3 * i + 1];
    TP6_inv[TP6_inv_tmp + 2] = m[3 * i + 2];
  }
  for (i = 0; i < 9; i++) {
    b_forwardMat[i] = -m[i];
  }
  for (i = 0; i < 3; i++) {
    TP6_inv[i + 12] = (b_forwardMat[i] * a6_tmp + b_forwardMat[i + 3] * 0.0) +
                      b_forwardMat[i + 6] * 0.0;
  }
  for (i = 0; i < 4; i++) {
    t = forwardMatWoOfst[i];
    EW = forwardMatWoOfst[i + 4];
    BW = forwardMatWoOfst[i + 8];
    absxk = forwardMatWoOfst[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      forwardMat[i + i2] =
          ((t * TP6_inv[i2] + EW * TP6_inv[i2 + 1]) + BW * TP6_inv[i2 + 2]) +
          absxk * TP6_inv[i2 + 3];
    }
  }
  /* P系(基于臂型角后撤的坐标系)转化为原始的W系 */
  /* 沿着6轴伸长的x方向加入偏执71mm，得到非偏置构型(初始青龙手臂)的末端位姿 */
  /* 下面就是根据初始青龙手臂构型的末端位姿来求取其臂型角 */
  dd2 = fabs(d_arr[4]) + a6_tmp;
  dd4 = fabs(d_arr[2]);
  a4 = fabs(a_arr[3]);
  a5 = fabs(a_arr[4]);
  scale = 3.3121686421112381E-170;
  absxk = fabs(forwardMat[12]);
  if (absxk > 3.3121686421112381E-170) {
    BW = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    BW = t * t;
  }
  absxk = fabs(forwardMat[13]);
  if (absxk > scale) {
    t = scale / absxk;
    BW = BW * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    BW += t * t;
  }
  absxk = fabs(forwardMat[14]);
  if (absxk > scale) {
    t = scale / absxk;
    BW = BW * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    BW += t * t;
  }
  BW = scale * sqrt(BW);
  BE = sqrt(dd2 * dd2 + a5 * a5);
  EW = sqrt(dd4 * dd4 + a4 * a4);
  if ((BE + EW) - BW > 1.0E-6) {
    theta3 = -(((6.2831853071795862 - rt_atan2d_snf(dd2, a5)) -
                acos(((BE * BE + EW * EW) - BW * BW) / (2.0 * BE * EW))) -
               rt_atan2d_snf(dd4, a4));
  } else {
    theta3 = cur_theta[3];
    /* 注意，这里就要考虑正反对应了.加起来正好等于7 */
    /*  这边是6Dof逆解时肘部奇异 */
  }
  /* 以上求得 theta3 */
  for (i = 0; i < 16; i++) {
    forwardMatWoOfst[i] = iv[i];
  }
  for (i = 0; i < 3; i++) {
    b_forwardMat[3 * i] = forwardMat[i];
    b_forwardMat[3 * i + 1] = forwardMat[i + 4];
    b_forwardMat[3 * i + 2] = forwardMat[i + 8];
  }
  memcpy(&m[0], &b_forwardMat[0], 9U * sizeof(double));
  for (i = 0; i < 3; i++) {
    TP6_inv_tmp = i << 2;
    forwardMatWoOfst[TP6_inv_tmp] = m[3 * i];
    forwardMatWoOfst[TP6_inv_tmp + 1] = m[3 * i + 1];
    forwardMatWoOfst[TP6_inv_tmp + 2] = m[3 * i + 2];
  }
  for (i = 0; i < 9; i++) {
    b_forwardMat[i] = -m[i];
  }
  t = forwardMat[12];
  EW = forwardMat[13];
  BW = forwardMat[14];
  for (i = 0; i < 3; i++) {
    forwardMatWoOfst[i + 12] =
        (b_forwardMat[i] * t + b_forwardMat[i + 3] * EW) +
        b_forwardMat[i + 6] * BW;
  }
  for (i = 0; i < 4; i++) {
    t = forwardMatWoOfst[i];
    EW = forwardMatWoOfst[i + 4];
    BW = forwardMatWoOfst[i + 8];
    absxk = forwardMatWoOfst[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      forwardMat[i + i2] = ((t * (double)iv1[i2] + EW * (double)iv1[i2 + 1]) +
                            BW * (double)iv1[i2 + 2]) +
                           absxk * (double)iv1[i2 + 3];
    }
  }
  scale = sin(theta3);
  p_idx_1 = cos(theta3);
  EW = a4 * p_idx_1;
  BE = forwardMat[14] / ((EW - a5) + dd4 * scale);
  t = fabs(BE);
  if ((t >= -1.0) && (t <= 1.0)) {
    theta2 = asin(BE);
    /* 这里有2组解 */
    /*  theta2 = pi-theta2;  % TODO */
  } else {
    /*  这边是6Dof逆解z方向上的肘部奇异 */
    theta2 = cur_theta[4];
    /*  error('with singularity, no solution'); */
  }
  /*  以上求得 theta2 */
  BW = cos(theta2);
  theta1 = rt_atan2d_snf(forwardMat[13], forwardMat[12]) +
           rt_atan2d_snf((a4 * scale - dd4 * p_idx_1) - dd2,
                         BW * ((-dd4 * scale + a5) - EW));
  BE = sin(theta1);
  EW = cos(theta1);
  Tz_theta_tmp[0] = EW;
  Tz_theta_tmp[4] = -BE;
  Tz_theta_tmp[8] = 0.0;
  Tz_theta_tmp[12] = 0.0;
  Tz_theta_tmp[1] = BE;
  Tz_theta_tmp[5] = EW;
  Tz_theta_tmp[9] = 0.0;
  Tz_theta_tmp[13] = 0.0;
  TP6_inv[0] = BW;
  BE = -sin(theta2);
  TP6_inv[4] = BE;
  TP6_inv[8] = 0.0;
  TP6_inv[12] = 0.0;
  TP6_inv[1] = 0.0;
  TP6_inv[5] = 0.0;
  TP6_inv[9] = 1.0;
  TP6_inv[13] = dd2;
  TP6_inv[2] = BE;
  TP6_inv[6] = -BW;
  TP6_inv[10] = 0.0;
  TP6_inv[14] = 0.0;
  Tz_theta_tmp[2] = 0.0;
  Tz_theta_tmp[3] = 0.0;
  TP6_inv[3] = 0.0;
  Tz_theta_tmp[6] = 0.0;
  Tz_theta_tmp[7] = 0.0;
  TP6_inv[7] = 0.0;
  Tz_theta_tmp[10] = 1.0;
  Tz_theta_tmp[11] = 0.0;
  TP6_inv[11] = 0.0;
  Tz_theta_tmp[14] = 0.0;
  Tz_theta_tmp[15] = 1.0;
  TP6_inv[15] = 1.0;
  for (i = 0; i < 4; i++) {
    t = Tz_theta_tmp[i];
    EW = Tz_theta_tmp[i + 4];
    BW = Tz_theta_tmp[i + 8];
    absxk = Tz_theta_tmp[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      forwardMatWoOfst[i + i2] =
          ((t * TP6_inv[i2] + EW * TP6_inv[i2 + 1]) + BW * TP6_inv[i2 + 2]) +
          absxk * TP6_inv[i2 + 3];
    }
  }
  TP6_inv[0] = p_idx_1;
  TP6_inv[4] = -scale;
  TP6_inv[8] = 0.0;
  TP6_inv[12] = a5;
  TP6_inv[2] = scale;
  TP6_inv[6] = p_idx_1;
  TP6_inv[10] = 0.0;
  TP6_inv[14] = 0.0;
  for (i = 0; i < 4; i++) {
    TP6_inv_tmp = i << 2;
    i1 = b_iv[i];
    TP6_inv[TP6_inv_tmp + 1] = i1;
    i2 = b_iv1[i];
    TP6_inv[TP6_inv_tmp + 3] = i2;
    t = TP6_inv[TP6_inv_tmp];
    EW = TP6_inv[TP6_inv_tmp + 2];
    for (TP6_inv_tmp = 0; TP6_inv_tmp < 4; TP6_inv_tmp++) {
      Tz_theta_tmp[i + (TP6_inv_tmp << 2)] =
          ((forwardMatWoOfst[TP6_inv_tmp] * t +
            forwardMatWoOfst[TP6_inv_tmp + 4] * (double)i1) +
           forwardMatWoOfst[TP6_inv_tmp + 8] * EW) +
          forwardMatWoOfst[TP6_inv_tmp + 12] * (double)i2;
    }
  }
  for (i = 0; i < 3; i++) {
    t = Tz_theta_tmp[i];
    EW = Tz_theta_tmp[i + 4];
    BW = Tz_theta_tmp[i + 8];
    for (i1 = 0; i1 < 3; i1++) {
      i2 = i1 << 2;
      b_forwardMat[i + 3 * i1] =
          (t * forwardMat[i2] + EW * forwardMat[i2 + 1]) +
          BW * forwardMat[i2 + 2];
    }
  }
  memcpy(&m[0], &b_forwardMat[0], 9U * sizeof(double));
  /* 以上求得 theta5 */
  EW = acos(-m[7]);
  if (fabs(EW - 3.1415926535897931) < 1.0E-6) {
    scale = cur_theta[2];
    /* 注意，这里就要考虑正反对应了.加起来正好等于7 */
    BW = rt_atan2d_snf(-m[3], -m[5]) - cur_theta[2];
  } else {
    BE = sin(EW - 3.1415926535897931);
    scale = rt_atan2d_snf(m[8] / BE, m[6] / BE);
    BW = rt_atan2d_snf(-m[4] / BE, m[1] / BE);
  }
  /* 以上求得 theta4,theta6 */
  scale = 3.1415926535897931 - scale;
  /* TODO */
  BE = theta[6];
  theta[0] = BW;
  theta[4] = theta2;
  theta[5] = -theta1;
  theta[6] = BE;
  /*   */
  theta[1] = EW - 3.1415926535897931;
  theta[2] = scale - 3.1415926535897931;
  theta[3] = theta3;
  cur_theta[0] = 0.0;
  cur_theta[1] = 0.0;
  cur_theta[2] = 0.0;
  cur_theta[3] = a_arr[3];
  cur_theta[4] = a_arr[4];
  cur_theta[5] = 0.0;
  cur_theta[6] = 0.0;
  a_arr_woOfst[0] = 0.0;
  a_arr_woOfst[1] = 0.0;
  a_arr_woOfst[2] = d_arr[2];
  a_arr_woOfst[3] = 0.0;
  a_arr_woOfst[4] = d_arr[2] + a6_tmp;
  a_arr_woOfst[5] = 0.0;
  a_arr_woOfst[6] = 0.0;
  for (i = 0; i < 16; i++) {
    forwardMat[i] = forwardMat_tmp[i];
  }
  memset(&m[0], 0, 9U * sizeof(double));
  for (b_i = 0; b_i < 6; b_i++) {
    t = dv[b_i];
    BW = sin(t);
    EW = cos(t);
    BE = dv1[b_i] + theta[b_i];
    scale = sin(BE);
    BE = cos(BE);
    Tz_theta_tmp[1] = 0.0;
    Tz_theta_tmp[5] = EW;
    Tz_theta_tmp[9] = -BW;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[6] = BW;
    Tz_theta_tmp[10] = EW;
    Tz_theta_tmp[14] = 0.0;
    Tz_theta_tmp[0] = 1.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[4] = 0.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[12] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = forwardMat[i];
      EW = forwardMat[i + 4];
      BW = forwardMat[i + 8];
      absxk = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        TP6_inv[i + i2] = ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
                           BW * Tz_theta_tmp[i2 + 2]) +
                          absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    Tz_theta_tmp[0] = 1.0;
    Tz_theta_tmp[4] = 0.0;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[12] = cur_theta[b_i];
    Tz_theta_tmp[1] = 0.0;
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[5] = 1.0;
    Tz_theta_tmp[6] = 0.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[9] = 0.0;
    Tz_theta_tmp[10] = 1.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[14] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = TP6_inv[i];
      EW = TP6_inv[i + 4];
      BW = TP6_inv[i + 8];
      absxk = TP6_inv[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
             BW * Tz_theta_tmp[i2 + 2]) +
            absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    Tz_theta_tmp[0] = BE;
    Tz_theta_tmp[4] = -scale;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[12] = 0.0;
    Tz_theta_tmp[1] = scale;
    Tz_theta_tmp[5] = BE;
    Tz_theta_tmp[9] = 0.0;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[6] = 0.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[10] = 1.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[14] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = forwardMat[i];
      EW = forwardMat[i + 4];
      BW = forwardMat[i + 8];
      absxk = forwardMat[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        TP6_inv[i + i2] = ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
                           BW * Tz_theta_tmp[i2 + 2]) +
                          absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    Tz_theta_tmp[2] = 0.0;
    Tz_theta_tmp[6] = 0.0;
    Tz_theta_tmp[10] = 1.0;
    Tz_theta_tmp[14] = a_arr_woOfst[b_i];
    Tz_theta_tmp[0] = 1.0;
    Tz_theta_tmp[1] = 0.0;
    Tz_theta_tmp[3] = 0.0;
    Tz_theta_tmp[4] = 0.0;
    Tz_theta_tmp[5] = 1.0;
    Tz_theta_tmp[7] = 0.0;
    Tz_theta_tmp[8] = 0.0;
    Tz_theta_tmp[9] = 0.0;
    Tz_theta_tmp[11] = 0.0;
    Tz_theta_tmp[12] = 0.0;
    Tz_theta_tmp[13] = 0.0;
    Tz_theta_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      t = TP6_inv[i];
      EW = TP6_inv[i + 4];
      BW = TP6_inv[i + 8];
      absxk = TP6_inv[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        forwardMat[i + i2] =
            ((t * Tz_theta_tmp[i2] + EW * Tz_theta_tmp[i2 + 1]) +
             BW * Tz_theta_tmp[i2 + 2]) +
            absxk * Tz_theta_tmp[i2 + 3];
      }
    }
    if (b_i + 1 == 4) {
      for (i = 0; i < 3; i++) {
        TP6_inv_tmp = i << 2;
        b_forwardMat[3 * i] = forwardMat[TP6_inv_tmp];
        b_forwardMat[3 * i + 1] = forwardMat[TP6_inv_tmp + 1];
        b_forwardMat[3 * i + 2] = forwardMat[TP6_inv_tmp + 2];
      }
      memcpy(&m[0], &b_forwardMat[0], 9U * sizeof(double));
    }
  }
  a4 = carte_idx_2;
  theta2 = -carte_idx_0;
  p_idx_1 = -carte_idx_1;
  carte_idx_0 = -carte_idx_1 * -0.0 - carte_idx_2 * -0.0;
  carte_idx_1 = -carte_idx_2 - theta2 * -0.0;
  carte_idx_2 = theta2 * -0.0 - (-p_idx_1);
  /*  零位面法向量为n */
  scale = 3.3121686421112381E-170;
  t = carte_idx_0 / 3.3121686421112381E-170;
  BE = t * t;
  absxk = fabs(carte_idx_1);
  if (absxk > 3.3121686421112381E-170) {
    t = 3.3121686421112381E-170 / absxk;
    BE = BE * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    BE += t * t;
  }
  absxk = fabs(carte_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    BE = BE * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    BE += t * t;
  }
  BE = scale * sqrt(BE);
  scale = 3.3121686421112381E-170;
  carte_idx_0 /= BE;
  absxk = fabs(theta2);
  if (absxk > 3.3121686421112381E-170) {
    EW = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    EW = t * t;
  }
  carte_idx_1 /= BE;
  absxk = fabs(p_idx_1);
  if (absxk > scale) {
    t = scale / absxk;
    EW = EW * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    EW += t * t;
  }
  carte_idx_2 /= BE;
  absxk = fabs(a4);
  if (absxk > scale) {
    t = scale / absxk;
    EW = EW * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    EW += t * t;
  }
  EW = scale * sqrt(EW);
  theta2 /= EW;
  p_idx_1 /= EW;
  a4 /= EW;
  /*  归一化的p */
  /* 接下来通过旋转轴p_e、旋转前向量n，旋转后向量n4求取轴角bet */
  BE = theta2 * theta2 * carte_idx_0;
  scale = theta2 * p_idx_1;
  EW = scale * carte_idx_1;
  BW = theta2 * a4 * carte_idx_2;
  theta1 = ((carte_idx_0 - BE) - EW) - BW;
  dd4 = p_idx_1 * carte_idx_2 - carte_idx_1 * a4;
  t = ((m[6] - BE) - EW) - BW;
  BW = p_idx_1 * p_idx_1 * carte_idx_1;
  EW = scale * carte_idx_0;
  BE = p_idx_1 * a4 * carte_idx_2;
  absxk = ((carte_idx_1 - BW) - EW) - BE;
  scale = carte_idx_0 * a4 - theta2 * carte_idx_2;
  BW = ((m[7] - BW) - EW) - BE;
  BE = scale * theta1;
  EW = dd4 * absxk;
  *bet = rt_atan2d_snf((t * absxk - BW * theta1) / (EW - BE),
                       (t * scale - BW * dd4) / (BE - EW));
}

/*
 * File trailer for fk_with_ofst.c
 *
 * [EOF]
 */
