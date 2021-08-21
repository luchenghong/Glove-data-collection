#ifndef FAST_KALMAN_FILTER_H
#define FAST_KALMAN_FILTER_H

#include <stdint.h>
#include "matrix.h"
#include "quat.h"
#include "sensor.h"


#define ACC_VARIANCE	2E-6
#define GYRO_VARIANCE	1E-6
#define MAG_VARIANCE	3E-4

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
	euler_f64 euler;
	double mN, mD;
	quat_f64 q_k_1;
	quat_f64 q_k_;
	quat_f64 q_k;
	double omegaX[16];
	double Sigma_k[12];
	double Sigma_gyro[9]; //variance of gyroscope
	double P_k_1[16];
	double P_k_[16];
	double P_k[16];
	double Q_k[16];
	quat_f64 z_k;
	double R_k[16];
	double W_a[16];
	double W_m[16];
	double Epsilon_acc_mag[36];
	double J[24];
	double K_k[16]; 

    //auxiliary variables
	double I4x4[16];
	double Temp3x4I[12];
	double Temp4x1I[4];
	double Temp4x1II[4];
	double Temp4x3I[12];	
	double Temp4x4I[16];	
	double Temp4x4II[16];
	double Temp4x4III[16];
	double Temp4x6I[24];
	double Temp6x4I[24];

	mat_instance_f64 matrix_q_k_1;
	mat_instance_f64 matrix_q_k_;
	mat_instance_f64 matrix_q_k;
	mat_instance_f64 matrix_omegaX;
	mat_instance_f64 matrix_Sigma_k;
	mat_instance_f64 matrix_Sigma_gyro;
	mat_instance_f64 matrix_P_k_1;
	mat_instance_f64 matrix_P_k_;
	mat_instance_f64 matrix_P_k;
	mat_instance_f64 matrix_Q_k;
	mat_instance_f64 matrix_z_k;
	mat_instance_f64 matrix_R_k;
	mat_instance_f64 matrix_W_a;
	mat_instance_f64 matrix_W_m;
	mat_instance_f64 matrix_Epsilon_acc_mag;
	mat_instance_f64 matrix_J;
	mat_instance_f64 matrix_K_k;
	
	mat_instance_f64 matrix_I4x4;
	mat_instance_f64 matrix_Temp3x4I;
	mat_instance_f64 matrix_Temp4x1I;
	mat_instance_f64 matrix_Temp4x1II;
	mat_instance_f64 matrix_Temp4x3I;
	mat_instance_f64 matrix_Temp4x4I;
	mat_instance_f64 matrix_Temp4x4II;
	mat_instance_f64 matrix_Temp4x4III;
	mat_instance_f64 matrix_Temp4x6I;
	mat_instance_f64 matrix_Temp6x4I;

}fast_kalman_filter;

extern void fast_kalman_filter_init(fast_kalman_filter* pFilter);
extern void fast_kalman_filter_update(fast_kalman_filter* pFilter, sensor_data* pSensor);

#ifdef __cplusplus
}
#endif

#endif
