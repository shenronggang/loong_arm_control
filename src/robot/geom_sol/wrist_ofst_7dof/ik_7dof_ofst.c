/*
 * File: ik_7dof_ofst.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-May-2025 11:02:17
 */

/* Include Files */
#include "ik_7dof_ofst.h"
#include "fk_with_ofst_data.h"
#include "fk_with_ofst_initialize.h"
#include "fk_with_ofst_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * global mid_theta
 * ------------以下求解辅助臂-----------------------------
 *
 * Arguments    : double z_alpha
 *                double y_beta
 *                double x_gamma
 *                double p_x
 *                double p_y
 *                double p_z
 *                double bet
 *                const double cur_theta[7]
 *                int lt_or_rt
 *                int LOrR
 *                int FOrB
 *                const double jntLim[14]
 *                const double a[3]
 *                const double d[2]
 *                double theta[7]
 *                int *ik_state
 * Return Type  : void
 */
void ik_7dof_ofst(double z_alpha, double y_beta, double x_gamma, double p_x,
                  double p_y, double p_z, double bet, double cur_theta[7],
                  int lt_or_rt, int LOrR, int FOrB, double jntLim[14],
                  double a[3], double d[2], double theta[7],
                  int *ik_state)
{
  static const signed char b[16] = {1, 0, 0, 0, 0,  1, 0, 0,
                                    0, 0, 1, 0, 54, 0, 0, 1};
  static const signed char b_iv[4] = {0, 0, 1, 0};
  static const signed char b_iv1[4] = {0, 0, 0, 1};
  static const signed char iv2[4] = {0, 0, -1, 0};
  double T01_tmp[16];
  double T07[16];
  double T0P[16];
  double T12_tmp[16];
  double b_T0P[16];
  double R47[9];
  double cos_DBW[9];
  double n4[3];
  double p_e[3];
  double L_BE_tmp;
  double L_EW;
  double L_EW_tmp;
  double R_end_tmp;
  double T07_tmp;
  double a_tmp;
  double absxk;
  double an_DEB;
  double an_DEB_tmp;
  double b_L_BE_tmp;
  double b_L_EW_tmp;
  double b_a;
  double b_a_tmp;
  double b_theta4_tmp;
  double d5;
  double n_idx_0;
  double n_idx_1;
  double n_idx_2;
  double scale;
  double shldr_sing_cond;
  double t;
  double theta1;
  double theta2;
  double theta3;
  double theta4;
  double theta4_tmp;
  int T12_tmp_tmp;
  int i;
  int i1;
  int i2;
  short lowLim[7];
  short upLim[7];
  boolean_T guard1;
  if (!isInitialized_fk_with_ofst) {
    fk_with_ofst_initialize();
  }
  *ik_state = 0;
  /*  0-normal,  */
  /*  2000-left arm joint 1 lower limit; 2001-left arm joint 1 upper limit;
   * 2100-right arm joint 1 lower limit; 2101-right arm joint 1 upper limit */
  /*  2060-left arm joint 7 lower limit; 2061-left arm joint 7 upper limit;
   * 2160-right arm joint 7 lower limit; 2161-right arm joint 7 upper limit */
  /*  25-elbow singularity 26-shoulder singularity  27-wrist singularity
   * 28-rotation singularity */
  /*  29-elbow singularity in z-axis */
  d5 = d[1] + a[2];
  /*  % 内旋ZYX旋转 */
  theta1 = cos(z_alpha);
  shldr_sing_cond = sin(z_alpha);
  scale = sin(x_gamma);
  absxk = cos(x_gamma);
  t = sin(y_beta);
  R_end_tmp = cos(y_beta);
  /*  R_end =[ cos(y_beta)*cos(z_alpha), -cos(y_beta)*sin(z_alpha), sin(y_beta);
   */
  /*            cos(x_gamma)*sin(z_alpha) +
   * cos(z_alpha)*sin(x_gamma)*sin(y_beta), cos(x_gamma)*cos(z_alpha) -
   * sin(x_gamma)*sin(y_beta)*sin(z_alpha), -cos(y_beta)*sin(x_gamma); */
  /*            sin(x_gamma)*sin(z_alpha) -
   * cos(x_gamma)*cos(z_alpha)*sin(y_beta), cos(z_alpha)*sin(x_gamma) +
   * cos(x_gamma)*sin(y_beta)*sin(z_alpha),  cos(x_gamma)*cos(y_beta)]; */
  T07[0] = theta1 * R_end_tmp;
  T07_tmp = absxk * shldr_sing_cond;
  T07[4] = theta1 * t * scale - T07_tmp;
  theta4 = theta1 * absxk;
  T07[8] = shldr_sing_cond * scale + theta4 * t;
  T07[1] = R_end_tmp * shldr_sing_cond;
  T07[5] = theta4 + shldr_sing_cond * t * scale;
  T07[9] = T07_tmp * t - theta1 * scale;
  T07[2] = -t;
  T07[6] = R_end_tmp * scale;
  T07[10] = R_end_tmp * absxk;
  T07[12] = p_x;
  T07[13] = p_y;
  T07[14] = p_z;
  T07[15] = 1.0;
  /*  AA = rotm2Eul(R_end, "xyz"); */
  T07[3] = 0.0;
  T07[7] = 0.0;
  T07[11] = 0.0;
  /* 记得把这里的坐标反一下 */
  shldr_sing_cond = a[0] * a[0];
  theta3 = d[0] * d[0];
  L_BE_tmp = theta3 + shldr_sing_cond;
  b_L_BE_tmp = sqrt(L_BE_tmp);
  L_EW_tmp = d5 * d5;
  b_L_EW_tmp = a[1] * a[1];
  L_EW = sqrt(L_EW_tmp + b_L_EW_tmp);
  scale = 3.3121686421112381E-170;
  absxk = fabs(-p_x);
  if (absxk > 3.3121686421112381E-170) {
    T07_tmp = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    T07_tmp = t * t;
  }
  absxk = fabs(-p_y);
  if (absxk > scale) {
    t = scale / absxk;
    T07_tmp = T07_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    T07_tmp += t * t;
  }
  absxk = fabs(p_z);
  if (absxk > scale) {
    t = scale / absxk;
    T07_tmp = T07_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    T07_tmp += t * t;
  }
  T07_tmp = scale * sqrt(T07_tmp);
  an_DEB_tmp = b_L_BE_tmp * b_L_BE_tmp;
  an_DEB = acos(((shldr_sing_cond + an_DEB_tmp) - theta3) /
                (2.0 * a[0] * b_L_BE_tmp));
  guard1 = false;
  if ((b_L_BE_tmp + L_EW) - T07_tmp > 1.0E-6) {
    /*  两边之和大于第三边，表示还没有伸直 */
    theta4_tmp = T07_tmp * T07_tmp;
    b_theta4_tmp = L_EW * L_EW;
    /* 明明我看着正角度是对的，怎么就变成了负角度 */
    /* 求得theta4后，就重定义一下机械臂 */
    a_tmp = theta3 + theta4_tmp;
    b_a_tmp = 2.0 * d[0] * T07_tmp;
    b_a = sqrt(a_tmp -
               b_a_tmp * cos((1.5707963267948966 - an_DEB) +
                             acos(((an_DEB_tmp + theta4_tmp) - b_theta4_tmp) /
                                  (2.0 * T07_tmp * b_L_BE_tmp))));
    /* 与p向量组成零面位 */
    n_idx_0 = -p_y * -0.0 - -0.0 * p_z;
    R_end_tmp = -p_x * -0.0;
    n_idx_1 = -p_z - R_end_tmp;
    n_idx_2 = R_end_tmp - p_y;
    /*  零位面法向量为n */
    scale = 3.3121686421112381E-170;
    t = n_idx_0 / 3.3121686421112381E-170;
    R_end_tmp = t * t;
    absxk = fabs(n_idx_1);
    if (absxk > 3.3121686421112381E-170) {
      t = 3.3121686421112381E-170 / absxk;
      R_end_tmp = R_end_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      R_end_tmp += t * t;
    }
    absxk = fabs(n_idx_2);
    if (absxk > scale) {
      t = scale / absxk;
      R_end_tmp = R_end_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      R_end_tmp += t * t;
    }
    R_end_tmp = scale * sqrt(R_end_tmp);
    n_idx_0 /= R_end_tmp;
    p_e[0] = -p_x / T07_tmp;
    n_idx_1 /= R_end_tmp;
    p_e[1] = -p_y / T07_tmp;
    n_idx_2 /= R_end_tmp;
    p_e[2] = p_z / T07_tmp;
    /*  归一化的p */
    bet += 3.1415926535897931;
    /*  绕p旋转bet角的轴角表示 */
    R_end_tmp = cos(bet);
    T07_tmp = sin(bet);
    R47[0] = p_e[0] * p_e[0] * (1.0 - R_end_tmp) + R_end_tmp;
    theta4 = p_e[0] * p_e[1] * (1.0 - R_end_tmp);
    theta1 = p_e[2] * T07_tmp;
    R47[3] = theta4 - theta1;
    shldr_sing_cond = p_e[0] * p_e[2] * (1.0 - R_end_tmp);
    scale = p_e[1] * T07_tmp;
    R47[6] = shldr_sing_cond + scale;
    R47[1] = theta4 + theta1;
    R47[4] = p_e[1] * p_e[1] * (1.0 - R_end_tmp) + R_end_tmp;
    theta4 = p_e[1] * p_e[2] * (1.0 - R_end_tmp);
    theta1 = p_e[0] * T07_tmp;
    R47[7] = theta4 - theta1;
    R47[2] = shldr_sing_cond - scale;
    R47[5] = theta4 + theta1;
    R47[8] = p_e[2] * p_e[2] * (1.0 - R_end_tmp) + R_end_tmp;
    for (i = 0; i < 3; i++) {
      n4[i] = (R47[i] * n_idx_0 + R47[i + 3] * n_idx_1) + R47[i + 6] * n_idx_2;
    }
    /* Xk */
    /* Zk */
    n_idx_0 = n4[1] * p_e[2] - p_e[1] * n4[2];
    n_idx_1 = p_e[0] * n4[2] - n4[0] * p_e[2];
    n_idx_2 = n4[0] * p_e[1] - p_e[0] * n4[1];
    /* 得到k轴方向的y在0中的描述 */
    scale = 3.3121686421112381E-170;
    absxk = fabs(n_idx_0);
    if (absxk > 3.3121686421112381E-170) {
      R_end_tmp = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      R_end_tmp = t * t;
    }
    absxk = fabs(n_idx_1);
    if (absxk > scale) {
      t = scale / absxk;
      R_end_tmp = R_end_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      R_end_tmp += t * t;
    }
    absxk = fabs(n_idx_2);
    if (absxk > scale) {
      t = scale / absxk;
      R_end_tmp = R_end_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      R_end_tmp += t * t;
    }
    R_end_tmp = scale * sqrt(R_end_tmp);
    /* 归一化，真正得到k轴的y在0中的描述 */
    absxk = (a_tmp - b_a * b_a) / b_a_tmp;
    t = sqrt(1.0 - absxk * absxk);
    /*  k_R_k'  由k'变换到k的旋转矩阵 */
    /*  D点在k'坐标系下的表示 */
    cos_DBW[0] = absxk;
    cos_DBW[3] = -t;
    cos_DBW[6] = 0.0;
    cos_DBW[1] = t;
    cos_DBW[4] = absxk;
    cos_DBW[7] = 0.0;
    R47[0] = p_e[0];
    R47[3] = n_idx_0 / R_end_tmp;
    R47[6] = n4[0];
    cos_DBW[2] = 0.0;
    R47[1] = p_e[1];
    R47[4] = n_idx_1 / R_end_tmp;
    R47[7] = n4[1];
    cos_DBW[5] = 0.0;
    R47[2] = p_e[2];
    R47[5] = n_idx_2 / R_end_tmp;
    R47[8] = n4[2];
    cos_DBW[8] = 1.0;
    absxk = sqrt(theta3);
    p_e[0] = absxk;
    p_e[1] = 0.0;
    p_e[2] = 0.0;
    for (i = 0; i < 3; i++) {
      n_idx_2 = 0.0;
      shldr_sing_cond = R47[i];
      scale = R47[i + 3];
      b_a = R47[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        n_idx_2 +=
            ((shldr_sing_cond * cos_DBW[3 * i1] + scale * cos_DBW[3 * i1 + 1]) +
             b_a * cos_DBW[3 * i1 + 2]) *
            p_e[i1];
      }
      n4[i] = n_idx_2;
    }
    /*  D点坐标在0系下的表示, xD=pD0(1), yD=pD0(2), zD=pD0(3) */
    theta2 = acos(n4[2] / absxk);
    /* 这里涉及到选哪个解 */
    /*  以上求得theta2  */
    /*  theta2 = -theta2; */
    T07_tmp = sin(theta2);
    shldr_sing_cond = d[0] * T07_tmp;
    if (fabs(shldr_sing_cond) > 1.0E-6) {
      theta1 = rt_atan2d_snf(n4[1] / shldr_sing_cond, n4[0] / shldr_sing_cond);
    } else {
      theta1 = cur_theta[0];
      *ik_state = 26;
      /*  代表肩奇异 */
    }
    /*  pD0 -- D点坐标在0系下的表示, xD=pD0(1), yD=pD0(2), zD=pD0(3) */
    n4[0] = -p_x - n4[0];
    n4[1] = -p_y - n4[1];
    n4[2] = p_z - n4[2];
    n_idx_0 = -p_y * n4[2] - n4[1] * p_z;
    n_idx_1 = n4[0] * p_z - -p_x * n4[2];
    n_idx_2 = -p_x * n4[1] - n4[0] * -p_y;
    /*  p和DW都是基于0系的，所以z4也是基于0系的 */
    scale = 3.3121686421112381E-170;
    absxk = fabs(n_idx_0);
    if (absxk > 3.3121686421112381E-170) {
      R_end_tmp = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      R_end_tmp = t * t;
    }
    absxk = fabs(n_idx_1);
    if (absxk > scale) {
      t = scale / absxk;
      R_end_tmp = R_end_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      R_end_tmp += t * t;
    }
    absxk = fabs(n_idx_2);
    if (absxk > scale) {
      t = scale / absxk;
      R_end_tmp = R_end_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      R_end_tmp += t * t;
    }
    R_end_tmp = scale * sqrt(R_end_tmp);
    n_idx_0 /= R_end_tmp;
    n_idx_1 /= R_end_tmp;
    n_idx_2 /= R_end_tmp;
    if (T07_tmp <= 1.0E-6) {
      theta3 = rt_atan2d_snf(-n_idx_0, n_idx_1) - theta1;
    } else if (T07_tmp > 1.0E-6) {
      R_end_tmp = cos(theta1);
      if (fabs(R_end_tmp) < 1.0E-6) {
        theta3 = rt_atan2d_snf(
            n_idx_2 / T07_tmp,
            -(n_idx_0 + R_end_tmp * cos(theta2) * n_idx_2 / T07_tmp) /
                sin(theta1));
      } else {
        theta3 = rt_atan2d_snf(
            n_idx_2 / T07_tmp,
            (n_idx_1 + sin(theta1) * cos(theta2) * n_idx_2 / T07_tmp) /
                cos(theta1));
      }
    } else {
      theta3 = rt_atan2d_snf(
          n_idx_2 / T07_tmp,
          (n_idx_1 + sin(theta1) * cos(theta2) * n_idx_2 / T07_tmp) /
              cos(theta1));
    }
    /*  以上求得theta3 */
    theta3 += 3.1415926535897931;
    /*  为啥theta3要翻180度？ */
    if (theta3 > 3.1415926535897931) {
      theta3 -= 6.2831853071795862;
    }
    theta2 = -theta2;
    theta4 =
        ((6.2831853071795862 -
          acos((((L_BE_tmp + L_EW_tmp) + b_L_EW_tmp) - theta4_tmp) /
               (2.0 * b_L_BE_tmp * L_EW))) -
         an_DEB) -
        acos(((b_L_EW_tmp + b_theta4_tmp) - L_EW_tmp) / (2.0 * a[1] * L_EW));
    R_end_tmp = sin(theta1);
    T07_tmp = cos(theta1);
    absxk = cos(theta2);
    a_tmp = sin(theta3);
    theta1 = cos(theta3);
    t = cos(theta4);
    T01_tmp[0] = T07_tmp;
    T01_tmp[4] = -R_end_tmp;
    T01_tmp[8] = 0.0;
    T01_tmp[12] = 0.0;
    T01_tmp[1] = R_end_tmp;
    T01_tmp[5] = T07_tmp;
    T01_tmp[9] = 0.0;
    T01_tmp[13] = 0.0;
    T12_tmp[0] = absxk;
    b_a_tmp = -sin(theta2);
    T12_tmp[4] = b_a_tmp;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 0.0;
    T12_tmp[2] = b_a_tmp;
    T12_tmp[6] = -absxk;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T01_tmp[2] = 0.0;
    T01_tmp[3] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[3] = 0.0;
    T01_tmp[6] = 0.0;
    T01_tmp[7] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[7] = 0.0;
    T01_tmp[10] = 1.0;
    T01_tmp[11] = 0.0;
    T12_tmp[9] = 1.0;
    T12_tmp[11] = 0.0;
    T01_tmp[14] = 0.0;
    T01_tmp[15] = 1.0;
    T12_tmp[13] = 0.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      n_idx_2 = T01_tmp[i];
      shldr_sing_cond = T01_tmp[i + 4];
      scale = T01_tmp[i + 8];
      b_a = T01_tmp[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        T0P[i + i2] =
            ((n_idx_2 * T12_tmp[i2] + shldr_sing_cond * T12_tmp[i2 + 1]) +
             scale * T12_tmp[i2 + 2]) +
            b_a * T12_tmp[i2 + 3];
      }
    }
    T12_tmp[0] = theta1;
    T12_tmp[4] = -a_tmp;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = a[0];
    T12_tmp[1] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[9] = -1.0;
    T12_tmp[13] = -d[0];
    T12_tmp[2] = a_tmp;
    T12_tmp[6] = theta1;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[11] = 0.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      n_idx_2 = T0P[i];
      shldr_sing_cond = T0P[i + 4];
      scale = T0P[i + 8];
      b_a = T0P[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        T01_tmp[i + i2] =
            ((n_idx_2 * T12_tmp[i2] + shldr_sing_cond * T12_tmp[i2 + 1]) +
             scale * T12_tmp[i2 + 2]) +
            b_a * T12_tmp[i2 + 3];
      }
    }
    T12_tmp[0] = t;
    b_a_tmp = -sin(theta4);
    T12_tmp[4] = b_a_tmp;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 0.0;
    T12_tmp[2] = b_a_tmp;
    T12_tmp[6] = -t;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    for (i = 0; i < 4; i++) {
      T12_tmp_tmp = i << 2;
      i1 = b_iv[i];
      T12_tmp[T12_tmp_tmp + 1] = i1;
      i2 = b_iv1[i];
      T12_tmp[T12_tmp_tmp + 3] = i2;
      n_idx_2 = T12_tmp[T12_tmp_tmp];
      shldr_sing_cond = T12_tmp[T12_tmp_tmp + 2];
      for (T12_tmp_tmp = 0; T12_tmp_tmp < 4; T12_tmp_tmp++) {
        b_T0P[i + (T12_tmp_tmp << 2)] =
            ((T01_tmp[T12_tmp_tmp] * n_idx_2 +
              T01_tmp[T12_tmp_tmp + 4] * (double)i1) +
             T01_tmp[T12_tmp_tmp + 8] * shldr_sing_cond) +
            T01_tmp[T12_tmp_tmp + 12] * (double)i2;
      }
    }
    for (i = 0; i < 3; i++) {
      n_idx_2 = b_T0P[i];
      shldr_sing_cond = b_T0P[i + 4];
      scale = b_T0P[i + 8];
      for (i1 = 0; i1 < 3; i1++) {
        i2 = i1 << 2;
        R47[i + 3 * i1] = (n_idx_2 * T07[i2] + shldr_sing_cond * T07[i2 + 1]) +
                          scale * T07[i2 + 2];
      }
    }
    absxk = acos(-R47[7]);
    /* 注意这里有选解问题，左右臂可能吧不同 */
    /*  以上求得theta6 */
    if (fabs(absxk) > 1.0E-6) {
      shldr_sing_cond = sin(absxk);
      scale = rt_atan2d_snf(R47[8] / shldr_sing_cond, R47[6] / shldr_sing_cond);
      n_idx_0 =
          rt_atan2d_snf(-R47[4] / shldr_sing_cond, R47[1] / shldr_sing_cond);
    } else {
      scale = cur_theta[4];
      /*  真实机器人赋予上一时刻值 */
      n_idx_0 = rt_atan2d_snf(-R47[3], R47[5]) - cur_theta[4];
      *ik_state = 27;
      /*  代表腕奇异 */
    }
    /*  disp("theta1"); */
    /*  [theta1 theta2 theta3 theta4 theta5 theta6 theta7] */
    /*  theta7 = 0; */
    /*     %% 这边的关节6的 */
    /* -------------------从现在开始求解倒置臂------------------------ */
    theta4 = sin(scale);
    R_end_tmp = cos(scale);
    T07_tmp = cos(absxk);
    /*  6轴x轴方向朝上 */
    /*  T56 =[ -sin(theta6), -cos(theta6), 0, 0; */
    /*                    0,            0, 1, 0; */
    /*         -cos(theta6),  sin(theta6), 0, 0; */
    /*                    0,            0, 0, 1];  % 6轴x轴方向朝内 */
    /*  T56 =[ sin(theta6), cos(theta6), 0, 0; */
    /*                   0,            0, 1, 0; */
    /*         cos(theta6), -sin(theta6), 0, 0; */
    /*                   0,            0, 0, 1];  % 6轴x轴方向朝外 */
    /* 注意，以后右边是上标 */
    /*  p_0_6 = sqrt(T_0_6(1:3,4)'*T_0_6(1:3,4)); */
    /*     %% 到这里相当于pdf的z6轴向前平移了71mm */
    /* 末端相当于沿着6系x方向前移了71，6系x方向是朝里的 */
    T12_tmp[0] = theta1;
    T12_tmp[4] = -a_tmp;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[9] = -1.0;
    T12_tmp[13] = -d[0];
    T12_tmp[2] = a_tmp;
    T12_tmp[6] = theta1;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[11] = 0.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      n_idx_2 = T0P[i];
      shldr_sing_cond = T0P[i + 4];
      scale = T0P[i + 8];
      b_a = T0P[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        T01_tmp[i + i2] =
            ((n_idx_2 * T12_tmp[i2] + shldr_sing_cond * T12_tmp[i2 + 1]) +
             scale * T12_tmp[i2 + 2]) +
            b_a * T12_tmp[i2 + 3];
      }
    }
    T12_tmp[0] = t;
    T12_tmp[4] = b_a_tmp;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = a[0];
    T12_tmp[2] = b_a_tmp;
    T12_tmp[6] = -t;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[9] = 1.0;
    T12_tmp[11] = 0.0;
    T12_tmp[13] = 0.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      n_idx_2 = T01_tmp[i];
      shldr_sing_cond = T01_tmp[i + 4];
      scale = T01_tmp[i + 8];
      b_a = T01_tmp[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        T0P[i + i2] =
            ((n_idx_2 * T12_tmp[i2] + shldr_sing_cond * T12_tmp[i2 + 1]) +
             scale * T12_tmp[i2 + 2]) +
            b_a * T12_tmp[i2 + 3];
      }
    }
    T12_tmp[0] = R_end_tmp;
    T12_tmp[4] = -theta4;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = -a[1];
    T12_tmp[1] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[9] = -1.0;
    T12_tmp[13] = -d5;
    T12_tmp[2] = theta4;
    T12_tmp[6] = R_end_tmp;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[11] = 0.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      n_idx_2 = T0P[i];
      shldr_sing_cond = T0P[i + 4];
      scale = T0P[i + 8];
      b_a = T0P[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        T01_tmp[i + i2] =
            ((n_idx_2 * T12_tmp[i2] + shldr_sing_cond * T12_tmp[i2 + 1]) +
             scale * T12_tmp[i2 + 2]) +
            b_a * T12_tmp[i2 + 3];
      }
    }
    T12_tmp[0] = T07_tmp;
    b_a_tmp = -sin(absxk);
    T12_tmp[4] = b_a_tmp;
    T12_tmp[8] = 0.0;
    T12_tmp[12] = 0.0;
    T12_tmp[2] = b_a_tmp;
    T12_tmp[6] = -T07_tmp;
    T12_tmp[10] = 0.0;
    T12_tmp[14] = 0.0;
    T12_tmp[1] = 0.0;
    T12_tmp[3] = 0.0;
    T12_tmp[5] = 0.0;
    T12_tmp[7] = 0.0;
    T12_tmp[9] = 1.0;
    T12_tmp[11] = 0.0;
    T12_tmp[13] = 0.0;
    T12_tmp[15] = 1.0;
    for (i = 0; i < 4; i++) {
      n_idx_2 = T01_tmp[i];
      shldr_sing_cond = T01_tmp[i + 4];
      scale = T01_tmp[i + 8];
      b_a = T01_tmp[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        T0P[i + i2] =
            ((n_idx_2 * T12_tmp[i2] + shldr_sing_cond * T12_tmp[i2 + 1]) +
             scale * T12_tmp[i2 + 2]) +
            b_a * T12_tmp[i2 + 3];
      }
      n_idx_2 = T0P[i];
      shldr_sing_cond = T0P[i + 4];
      scale = T0P[i + 8];
      b_a = T0P[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 << 2;
        T07[i + i2] =
            ((n_idx_2 * (double)b[i2] + shldr_sing_cond * (double)b[i2 + 1]) +
             scale * (double)b[i2 + 2]) +
            b_a * (double)b[i2 + 3];
      }
    }
    /* 后撤得到P系 */
    scale = 3.3121686421112381E-170;
    absxk = fabs(T07[12]);
    if (absxk > 3.3121686421112381E-170) {
      theta1 = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      theta1 = t * t;
    }
    absxk = fabs(T07[13]);
    if (absxk > scale) {
      t = scale / absxk;
      theta1 = theta1 * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta1 += t * t;
    }
    absxk = fabs(T07[14]);
    if (absxk > scale) {
      t = scale / absxk;
      theta1 = theta1 * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta1 += t * t;
    }
    theta1 = scale * sqrt(theta1);
    scale = sqrt(d[1] * d[1] + b_L_EW_tmp);
    if ((scale + b_L_BE_tmp) - theta1 > 1.0E-6) {
      scale = acos(((scale * scale + an_DEB_tmp) - theta1 * theta1) /
                   (2.0 * scale * b_L_BE_tmp));
      theta1 = rt_atan2d_snf(d[1], a[1]);
      shldr_sing_cond = rt_atan2d_snf(d[0], a[0]);
      if (LOrR == 1) {
        theta3 = -(((6.2831853071795862 - theta1) - scale) - shldr_sing_cond);
      } else {
        theta3 = (theta1 + shldr_sing_cond) - scale;
      }
      /* 以上求得 theta3 */
      for (i = 0; i < 16; i++) {
        T0P[i] = iv[i];
      }
      for (i = 0; i < 3; i++) {
        n_idx_2 = T07[i];
        R47[3 * i] = n_idx_2;
        T12_tmp_tmp = i << 2;
        T0P[T12_tmp_tmp] = n_idx_2;
        n_idx_2 = T07[i + 4];
        R47[3 * i + 1] = n_idx_2;
        T0P[T12_tmp_tmp + 1] = n_idx_2;
        n_idx_2 = T07[i + 8];
        R47[3 * i + 2] = n_idx_2;
        T0P[T12_tmp_tmp + 2] = n_idx_2;
      }
      for (i = 0; i < 9; i++) {
        R47[i] = -R47[i];
      }
      n_idx_2 = T07[12];
      shldr_sing_cond = T07[13];
      scale = T07[14];
      for (i = 0; i < 3; i++) {
        T0P[i + 12] = (R47[i] * n_idx_2 + R47[i + 3] * shldr_sing_cond) +
                      R47[i + 6] * scale;
      }
      for (i = 0; i < 4; i++) {
        n_idx_2 = T0P[i];
        shldr_sing_cond = T0P[i + 4];
        scale = T0P[i + 8];
        b_a = T0P[i + 12];
        for (i1 = 0; i1 < 4; i1++) {
          i2 = i1 << 2;
          T07[i + i2] = ((n_idx_2 * (double)iv1[i2] +
                          shldr_sing_cond * (double)iv1[i2 + 1]) +
                         scale * (double)iv1[i2 + 2]) +
                        b_a * (double)iv1[i2 + 3];
        }
      }
      absxk = sin(theta3);
      t = cos(theta3);
      shldr_sing_cond = a[0] * t;
      theta1 = T07[14] / ((shldr_sing_cond - a[1]) + d[0] * absxk);
      n_idx_2 = fabs(theta1);
      if ((n_idx_2 >= -1.0) && (n_idx_2 <= 1.0)) {
        if (FOrB == 1) {
          theta2 = asin(theta1);
          /*  -pi/2~pi/2 */
        } else {
          theta2 = 3.1415926535897931 - asin(theta1);
          /*  pi/2~3pi/2 */
        }
        /*  以上求得 theta2 */
        scale = cos(theta2);
        theta1 =
            rt_atan2d_snf(T07[13], T07[12]) +
            rt_atan2d_snf((a[0] * absxk - d[0] * t) - d[1],
                          scale * ((-d[0] * absxk + a[1]) - shldr_sing_cond));
        R_end_tmp = sin(theta1);
        T07_tmp = cos(theta1);
        T01_tmp[0] = T07_tmp;
        T01_tmp[4] = -R_end_tmp;
        T01_tmp[8] = 0.0;
        T01_tmp[12] = 0.0;
        T01_tmp[1] = R_end_tmp;
        T01_tmp[5] = T07_tmp;
        T01_tmp[9] = 0.0;
        T01_tmp[13] = 0.0;
        T12_tmp[0] = scale;
        b_a_tmp = -sin(theta2);
        T12_tmp[4] = b_a_tmp;
        T12_tmp[8] = 0.0;
        T12_tmp[12] = 0.0;
        T12_tmp[1] = 0.0;
        T12_tmp[5] = 0.0;
        T12_tmp[9] = 1.0;
        T12_tmp[13] = d[1];
        T12_tmp[2] = b_a_tmp;
        T12_tmp[6] = -scale;
        T12_tmp[10] = 0.0;
        T12_tmp[14] = 0.0;
        T01_tmp[2] = 0.0;
        T01_tmp[3] = 0.0;
        T12_tmp[3] = 0.0;
        T01_tmp[6] = 0.0;
        T01_tmp[7] = 0.0;
        T12_tmp[7] = 0.0;
        T01_tmp[10] = 1.0;
        T01_tmp[11] = 0.0;
        T12_tmp[11] = 0.0;
        T01_tmp[14] = 0.0;
        T01_tmp[15] = 1.0;
        T12_tmp[15] = 1.0;
        for (i = 0; i < 4; i++) {
          n_idx_2 = T01_tmp[i];
          shldr_sing_cond = T01_tmp[i + 4];
          scale = T01_tmp[i + 8];
          b_a = T01_tmp[i + 12];
          for (i1 = 0; i1 < 4; i1++) {
            i2 = i1 << 2;
            b_T0P[i + i2] =
                ((n_idx_2 * T12_tmp[i2] + shldr_sing_cond * T12_tmp[i2 + 1]) +
                 scale * T12_tmp[i2 + 2]) +
                b_a * T12_tmp[i2 + 3];
          }
        }
        T12_tmp[0] = t;
        T12_tmp[4] = -absxk;
        T12_tmp[8] = 0.0;
        T12_tmp[12] = a[1];
        T12_tmp[2] = absxk;
        T12_tmp[6] = t;
        T12_tmp[10] = 0.0;
        T12_tmp[14] = 0.0;
        for (i = 0; i < 4; i++) {
          T12_tmp_tmp = i << 2;
          i1 = iv2[i];
          T12_tmp[T12_tmp_tmp + 1] = i1;
          i2 = b_iv1[i];
          T12_tmp[T12_tmp_tmp + 3] = i2;
          n_idx_2 = T12_tmp[T12_tmp_tmp];
          shldr_sing_cond = T12_tmp[T12_tmp_tmp + 2];
          for (T12_tmp_tmp = 0; T12_tmp_tmp < 4; T12_tmp_tmp++) {
            T01_tmp[i + (T12_tmp_tmp << 2)] =
                ((b_T0P[T12_tmp_tmp] * n_idx_2 +
                  b_T0P[T12_tmp_tmp + 4] * (double)i1) +
                 b_T0P[T12_tmp_tmp + 8] * shldr_sing_cond) +
                b_T0P[T12_tmp_tmp + 12] * (double)i2;
          }
        }
        for (i = 0; i < 3; i++) {
          n_idx_2 = T01_tmp[i];
          shldr_sing_cond = T01_tmp[i + 4];
          scale = T01_tmp[i + 8];
          for (i1 = 0; i1 < 3; i1++) {
            i2 = i1 << 2;
            R47[i + 3 * i1] =
                (n_idx_2 * T07[i2] + shldr_sing_cond * T07[i2 + 1]) +
                scale * T07[i2 + 2];
          }
        }
        /* 以上求得 theta5 */
        R_end_tmp = acos(-R47[7]);
        if (fabs(R_end_tmp - 3.1415926535897931) < 1.0E-6) {
          theta4 = cur_theta[2];
          /* 注意，这里就要考虑正反对应了.加起来正好等于7 */
          absxk = rt_atan2d_snf(-R47[3], -R47[5]) - cur_theta[2];
        } else {
          theta4_tmp = sin(R_end_tmp - 3.1415926535897931);
          theta4 = rt_atan2d_snf(R47[8] / theta4_tmp, R47[6] / theta4_tmp);
          absxk = rt_atan2d_snf(-R47[4] / theta4_tmp, R47[1] / theta4_tmp);
        }
        if (lt_or_rt == 0) {
          theta[0] = absxk + 3.1415926535897931;
          theta[1] = R_end_tmp - 3.1415926535897931;
          theta[2] = -theta4;
          theta[3] = -theta3;
          theta[4] = theta2;
          theta[5] = -theta1;
          theta[6] = n_idx_0;
          /*  左臂关节1翻-pi直接是让关节2的值翻过来，与原来方向对应 */
        } else {
          theta[0] = absxk - 3.1415926535897931;
          theta[1] = R_end_tmp - 3.1415926535897931;
          theta[2] = -theta4;
          theta[3] = -theta3;
          theta[4] = theta2;
          theta[5] = -theta1;
          theta[6] = n_idx_0;
          /*  右臂关节相对于左臂加上2*pi直接是为了避免负角度，从而超过设定的限位值
           */
        }
        /*  theta = [theta6 theta5 -theta4 -theta3 theta2 -theta1 theta7]; */
        /*  倒置臂if 的 end */
        guard1 = true;
      } else {
        *ik_state = 29;
        /*  这边是6Dof逆解z方向上的肘部奇异 */
        for (i = 0; i < 7; i++) {
          theta[i] = cur_theta[i];
        }
      }
    } else {
      /* 注意，这里就要考虑正反对应了.加起来正好等于7 */
      *ik_state = 28;
      /*  这边是6Dof逆解时肘部奇异 */
      /* 肘奇异 */
      for (i = 0; i < 7; i++) {
        theta[i] = cur_theta[i];
      }
      /*  倒置臂if 的 end */
      guard1 = true;
    }
  } else {
    *ik_state = 25;
    /*  代表肘关节伸直，遇到肘奇异 */
    for (i = 0; i < 7; i++) {
      theta[i] = cur_theta[i];
    }
    guard1 = true;
  }
  if (guard1) {
    /*  大if分支的end */
    theta[5] = -theta[5];
    theta[5] += 1.5707963267948966;
    /* 把6轴从朝下90度翻转为朝着正前方 */
    if (theta[0] > 3.1415926535897931) {
      theta[0] -= 6.2831853071795862;
    } else if (theta[0] < -3.1415926535897931) {
      theta[0] += 6.2831853071795862;
    }
    if (theta[3] > 3.1415926535897931) {
      /* 这个不确定能不能约束住4轴不翻转 */
      theta[3] = 3.1415926535897931;
    } else if (theta[3] < -3.1415926535897931) {
      theta[3] = -3.1415926535897931;
    }
    if (theta[6] > 1.5707963267948966) {
      theta[6] = 1.5707963267948966;
    } else if (theta[6] < -1.5707963267948966) {
      theta[6] = -1.5707963267948966;
    }
    if (lt_or_rt == 0) {
      /*  0-left arm   1-right arm */
      for (i = 0; i < 7; i++) {
        lowLim[i] = (short)(10 * i + 2000);
        upLim[i] = (short)(10 * i + 2001);
      }
    } else {
      for (i = 0; i < 7; i++) {
        lowLim[i] = (short)(10 * i + 2100);
        upLim[i] = (short)(10 * i + 2101);
      }
    }
    for (T12_tmp_tmp = 0; T12_tmp_tmp < 7; T12_tmp_tmp++) {
      i = T12_tmp_tmp << 1;
      n_idx_2 = theta[T12_tmp_tmp];
      if (n_idx_2 < jntLim[i]) {
        *ik_state = lowLim[T12_tmp_tmp];
      } else if (n_idx_2 > jntLim[i + 1]) {
        *ik_state = upLim[T12_tmp_tmp];
      }
    }
  }
}

/*
 * File trailer for ik_7dof_ofst.c
 *
 * [EOF]
 */
