//Author: Zeyang Dai
//Date: 2019-07-24
//Email: zeyang-d@outlook.com
//Decription: Novel MARG-Sensor Orientation Estimation Algorithm Using Fast Kalman Filter, experiments show that Magnetic disturbance has no #            effect on pitch and roll, but affect yaw

#include "fast_kalman_filter.h"
#include <math.h>
#include "sensor.h"

void fast_kalman_filter_init(fast_kalman_filter* pFilter)
{
	int i;
	
	mat_init_f64(&(pFilter->matrix_q_k_1),4,1,(double*)&(pFilter->q_k_1));
	mat_init_f64(&(pFilter->matrix_q_k_),4,1,(double*)&(pFilter->q_k_));
	mat_init_f64(&(pFilter->matrix_q_k),4,1,(double*)&(pFilter->q_k));
	mat_init_f64(&(pFilter->matrix_omegaX),4,4,pFilter->omegaX);
	mat_init_f64(&(pFilter->matrix_Sigma_k),4,3,pFilter->Sigma_k);
	mat_init_f64(&(pFilter->matrix_Sigma_gyro),3,3,pFilter->Sigma_gyro);
	mat_init_f64(&(pFilter->matrix_P_k_1),4,4,pFilter->P_k_1);
	mat_init_f64(&(pFilter->matrix_P_k_),4,4,pFilter->P_k_);
	mat_init_f64(&(pFilter->matrix_P_k),4,4,pFilter->P_k);
	mat_init_f64(&(pFilter->matrix_Q_k),4,4,pFilter->Q_k);
	mat_init_f64(&(pFilter->matrix_z_k),4,1,(double*)&(pFilter->z_k));
	mat_init_f64(&(pFilter->matrix_R_k),4,4,pFilter->R_k);
	mat_init_f64(&(pFilter->matrix_W_a),4,4,pFilter->W_a);
	mat_init_f64(&(pFilter->matrix_W_m),4,4,pFilter->W_m);
	mat_init_f64(&(pFilter->matrix_Epsilon_acc_mag),6,6,pFilter->Epsilon_acc_mag);
	mat_init_f64(&(pFilter->matrix_J),4,6,pFilter->J);
	mat_init_f64(&(pFilter->matrix_K_k),4,4,pFilter->K_k);

	mat_init_f64(&(pFilter->matrix_I4x4),4,4,pFilter->I4x4);	
	mat_init_f64(&(pFilter->matrix_Temp3x4I),3,4,pFilter->Temp3x4I);
	mat_init_f64(&(pFilter->matrix_Temp4x1I),4,1,pFilter->Temp4x1I);
	mat_init_f64(&(pFilter->matrix_Temp4x1II),4,1,pFilter->Temp4x1II);
	mat_init_f64(&(pFilter->matrix_Temp4x3I),4,3,pFilter->Temp4x3I);
	mat_init_f64(&(pFilter->matrix_Temp4x4I),4,4,pFilter->Temp4x4I);
	mat_init_f64(&(pFilter->matrix_Temp4x4II),4,4,pFilter->Temp4x4II);
	mat_init_f64(&(pFilter->matrix_Temp4x4III),4,4,pFilter->Temp4x4III);
	mat_init_f64(&(pFilter->matrix_Temp4x6I),4,6,pFilter->Temp4x6I);
	mat_init_f64(&(pFilter->matrix_Temp6x4I),6,4,pFilter->Temp6x4I);
	
	mat_identity_f64(&(pFilter->matrix_I4x4));
	mat_zeros_f64(&(pFilter->matrix_q_k_1));
	pFilter->q_k_1.w = 1.0;
	mat_scale_f64(&(pFilter->matrix_I4x4), 0.001, &(pFilter->matrix_P_k_1));
	mat_zeros_f64(&(pFilter->matrix_Sigma_k));
	mat_zeros_f64(&(pFilter->matrix_Sigma_gyro));
	mat_zeros_f64(&(pFilter->matrix_Epsilon_acc_mag));
	
	for(i=X; i<=Z; i++)
		pFilter->Sigma_gyro[i*3+i] = GYRO_VARIANCE;

	for(i=0; i<=2; i++)
		pFilter->Epsilon_acc_mag[i*6+i] = ACC_VARIANCE;
	for(i=3; i<=5; i++)
		pFilter->Epsilon_acc_mag[i*6+i] = MAG_VARIANCE;
	
}

void fast_kalman_filter_update(fast_kalman_filter* pFilter, sensor_data* pSensor)
{
	double acc_norm, mag_norm;
	int i, j;

	acc_norm = sqrt(pSensor->fAccel[X]*pSensor->fAccel[X] + pSensor->fAccel[Y]*pSensor->fAccel[Y] + pSensor->fAccel[Z]*pSensor->fAccel[Z]);
	mag_norm = sqrt(pSensor->fMag[X]*pSensor->fMag[X] + pSensor->fMag[Y]*pSensor->fMag[Y] + pSensor->fMag[Z]*pSensor->fMag[Z]);
	for(i=X; i<=Z; i++)
	{
		pSensor->fAccel[i] /= acc_norm;
		pSensor->fMag[i] /= mag_norm;
	}	

	//quaternion from gyroscope: prediction, process model
	i=0;
	pFilter->omegaX[i++] = 0.0;
	pFilter->omegaX[i++] = -pSensor->fGyro[X];
	pFilter->omegaX[i++] = -pSensor->fGyro[Y];
	pFilter->omegaX[i++] = -pSensor->fGyro[Z];
	pFilter->omegaX[i++] = pSensor->fGyro[X];
	pFilter->omegaX[i++] = 0.0;
	pFilter->omegaX[i++] = pSensor->fGyro[Z];
	pFilter->omegaX[i++] = -pSensor->fGyro[Y];
	pFilter->omegaX[i++] = pSensor->fGyro[Y];;
	pFilter->omegaX[i++] = -pSensor->fGyro[Z];
	pFilter->omegaX[i++] = 0.0;
	pFilter->omegaX[i++] = pSensor->fGyro[X];
	pFilter->omegaX[i++] = pSensor->fGyro[Z];
	pFilter->omegaX[i++] = pSensor->fGyro[Y];
	pFilter->omegaX[i++] = -pSensor->fGyro[X];
	pFilter->omegaX[i++] = 0.0;
	mat_scale_f64(&(pFilter->matrix_omegaX), pSensor->deltaT/2.0, &(pFilter->matrix_Temp4x4I));
	mat_add_f64(&(pFilter->matrix_I4x4), &(pFilter->matrix_Temp4x4I), &(pFilter->matrix_Temp4x4II));
	mat_mult_f64(&(pFilter->matrix_Temp4x4II), &(pFilter->matrix_q_k_1), &(pFilter->matrix_q_k_));
	i=0;
	pFilter->Sigma_k[i++] = pFilter->q_k_1.x;
	pFilter->Sigma_k[i++] = pFilter->q_k_1.y;
	pFilter->Sigma_k[i++] = pFilter->q_k_1.z;
	pFilter->Sigma_k[i++] = -pFilter->q_k_1.w;
	pFilter->Sigma_k[i++] = -pFilter->q_k_1.z;
	pFilter->Sigma_k[i++] = -pFilter->q_k_1.y;
	pFilter->Sigma_k[i++] = pFilter->q_k_1.y;
	pFilter->Sigma_k[i++] = -pFilter->q_k_1.w;
	pFilter->Sigma_k[i++] = -pFilter->q_k_1.x;
	pFilter->Sigma_k[i++] = -pFilter->q_k_1.y;
	pFilter->Sigma_k[i++] = pFilter->q_k_1.x;
	pFilter->Sigma_k[i++] = -pFilter->q_k_1.w;
	mat_mult_f64(&(pFilter->matrix_Sigma_k), &(pFilter->matrix_Sigma_gyro), &(pFilter->matrix_Temp4x3I));
	mat_trans_f64(&(pFilter->matrix_Sigma_k), &(pFilter->matrix_Temp3x4I));
	mat_mult_f64(&(pFilter->matrix_Temp4x3I), &(pFilter->matrix_Temp3x4I), &(pFilter->matrix_Temp4x4I));
	mat_scale_f64(&(pFilter->matrix_Temp4x4I), pSensor->deltaT*pSensor->deltaT/4.0, &(pFilter->matrix_Q_k));
	mat_scale_f64(&(pFilter->matrix_omegaX), pSensor->deltaT/2.0, &(pFilter->matrix_Temp4x4I));
	mat_add_f64(&(pFilter->matrix_I4x4), &(pFilter->matrix_Temp4x4I), &(pFilter->matrix_Temp4x4II));
	mat_mult_f64(&(pFilter->matrix_Temp4x4II), &(pFilter->matrix_P_k_1), &(pFilter->matrix_Temp4x4III));
	mat_trans_f64(&(pFilter->matrix_Temp4x4II), &(pFilter->matrix_Temp4x4I));
	mat_mult_f64(&(pFilter->matrix_Temp4x4III), &(pFilter->matrix_Temp4x4I), &(pFilter->matrix_Temp4x4II));
	mat_add_f64(&(pFilter->matrix_Temp4x4II), &(pFilter->matrix_Q_k), &(pFilter->matrix_P_k_));

	//quaternion from accelerator and magnetometer: measurement model
	pFilter->mD = pSensor->fAccel[X]*pSensor->fMag[X] + pSensor->fAccel[Y]*pSensor->fMag[Y] + pSensor->fAccel[Z]*pSensor->fMag[Z];
	pFilter->mN = sqrt(1.0 - pFilter->mD*pFilter->mD);
	i=0;
	pFilter->W_a[i++] = pSensor->fAccel[Z];
	pFilter->W_a[i++] = pSensor->fAccel[Y];
	pFilter->W_a[i++] = -pSensor->fAccel[X];
	pFilter->W_a[i++] = 0.0;
	pFilter->W_a[i++] = pSensor->fAccel[Y];
	pFilter->W_a[i++] = -pSensor->fAccel[Z];
	pFilter->W_a[i++] = 0.0;
	pFilter->W_a[i++] = pSensor->fAccel[X];
	pFilter->W_a[i++] = -pSensor->fAccel[X];
	pFilter->W_a[i++] = 0.0;
	pFilter->W_a[i++] = -pSensor->fAccel[Z];
	pFilter->W_a[i++] = pSensor->fAccel[Y];
	pFilter->W_a[i++] = 0.0;
	pFilter->W_a[i++] = pSensor->fAccel[X];
	pFilter->W_a[i++] = pSensor->fAccel[Y];
	pFilter->W_a[i++] = pSensor->fAccel[Z];
	i=0;
	pFilter->W_m[i++] = pFilter->mN*pSensor->fMag[X] + pFilter->mD*pSensor->fMag[Z];
	pFilter->W_m[i++] = pFilter->mD*pSensor->fMag[Y];
	pFilter->W_m[i++] = pFilter->mN*pSensor->fMag[Z] - pFilter->mD*pSensor->fMag[X];
	pFilter->W_m[i++] = -pFilter->mN*pSensor->fMag[Y];
	pFilter->W_m[i++] = pFilter->mD*pSensor->fMag[Y];
    pFilter->W_m[i++] = pFilter->mN*pSensor->fMag[X] - pFilter->mD*pSensor->fMag[Z];
	pFilter->W_m[i++] = pFilter->mN*pSensor->fMag[Y];
	pFilter->W_m[i++] = pFilter->mN*pSensor->fMag[Z] + pFilter->mD*pSensor->fMag[X];
	pFilter->W_m[i++] = pFilter->mN*pSensor->fMag[Z] - pFilter->mD*pSensor->fMag[X];
	pFilter->W_m[i++] = pFilter->mN*pSensor->fMag[Y];
	pFilter->W_m[i++] = -pFilter->mN*pSensor->fMag[X] - pFilter->mD*pSensor->fMag[Z];
	pFilter->W_m[i++] = pFilter->mD*pSensor->fMag[Y];
	pFilter->W_m[i++] = -pFilter->mN*pSensor->fMag[Y];
	pFilter->W_m[i++] = pFilter->mN*pSensor->fMag[Z] + pFilter->mD*pSensor->fMag[X];	
	pFilter->W_m[i++] = pFilter->mD*pSensor->fMag[Y];
	pFilter->W_m[i++] = -pFilter->mN*pSensor->fMag[X] + pFilter->mD*pSensor->fMag[Z];
	mat_add_f64(&(pFilter->matrix_W_m), &(pFilter->matrix_I4x4), &(pFilter->matrix_Temp4x4I));
	mat_add_f64(&(pFilter->matrix_W_a), &(pFilter->matrix_I4x4), &(pFilter->matrix_Temp4x4II));
	mat_mult_f64(&(pFilter->matrix_Temp4x4I),&(pFilter->matrix_Temp4x4II), &(pFilter->matrix_Temp4x4III));
	mat_mult_f64(&(pFilter->matrix_Temp4x4III),&(pFilter->matrix_q_k_1), &(pFilter->matrix_Temp4x1I));
	mat_scale_f64(&(pFilter->matrix_Temp4x1I), 1.0/4.0, &(pFilter->matrix_z_k));
	quat_normlise_f64(&(pFilter->z_k));
	i=0;
	pFilter->J[i++] = (-pFilter->q_k_1.y-pFilter->mN*(pSensor->fMag[Z]*pFilter->q_k_1.w+pSensor->fMag[Y]*pFilter->q_k_1.x-pSensor->fMag[X]*pFilter->q_k_1.y)+pFilter->mD*(pSensor->fMag[X]*pFilter->q_k_1.w+pSensor->fMag[Z]*pFilter->q_k_1.y-pSensor->fMag[Y]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->q_k_1.x + pFilter->mN*pSensor->fMag[X]*pFilter->q_k_1.x+pFilter->mN*pSensor->fMag[Y]*pFilter->q_k_1.y+pFilter->mN*pSensor->fMag[Z]*pFilter->q_k_1.z+pFilter->mD*(pSensor->fMag[Y]*pFilter->q_k_1.w-pSensor->fMag[Z]*pFilter->q_k_1.x+pSensor->fMag[X]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->q_k_1.w+pFilter->mN*pSensor->fMag[X]*pFilter->q_k_1.w+pFilter->mD*pSensor->fMag[Z]*pFilter->q_k_1.w+pFilter->mD*pSensor->fMag[Y]*pFilter->q_k_1.x-pFilter->mD*pSensor->fMag[X]*pFilter->q_k_1.y+pFilter->mN*pSensor->fMag[Z]*pFilter->q_k_1.y-pFilter->mN*pSensor->fMag[Y]*pFilter->q_k_1.z)/4.0;
	pFilter->J[i++] = ((pSensor->fAccel[X]*pFilter->mD+pFilter->mN+pSensor->fAccel[Z]*pFilter->mN)*pFilter->q_k_1.w+pSensor->fAccel[Y]*pFilter->mN*pFilter->q_k_1.x+(-((1+pSensor->fAccel[Z])*pFilter->mD)+pSensor->fAccel[X]*pFilter->mN)*pFilter->q_k_1.y)/4.0;
	pFilter->J[i++] = (pSensor->fAccel[Y]*pFilter->mD*pFilter->q_k_1.w+(pFilter->mD+pSensor->fAccel[Z]*pFilter->mD-pSensor->fAccel[X]*pFilter->mN)*pFilter->q_k_1.x+pSensor->fAccel[Y]*pFilter->mN*pFilter->q_k_1.y-(pSensor->fAccel[X]*pFilter->mD+pFilter->mN+pSensor->fAccel[Z]*pFilter->mN)*pFilter->q_k_1.z)/4.0;
	pFilter->J[i++] = (pFilter->mD*(pFilter->q_k_1.w+pSensor->fAccel[Z]*pFilter->q_k_1.w-pSensor->fAccel[Y]*pFilter->q_k_1.x+pSensor->fAccel[X]*pFilter->q_k_1.y)+pFilter->mN*(-(pSensor->fAccel[X]*pFilter->q_k_1.w)+pFilter->q_k_1.y+pSensor->fAccel[Z]*pFilter->q_k_1.y+pSensor->fAccel[Y]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->q_k_1.z - pFilter->mN*(pSensor->fMag[Y]*pFilter->q_k_1.w-pSensor->fMag[Z]*pFilter->q_k_1.x+pSensor->fMag[X]*pFilter->q_k_1.z)+pFilter->mD*(pSensor->fMag[X]*pFilter->q_k_1.x+pSensor->fMag[Y]*pFilter->q_k_1.y+pSensor->fMag[Z]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->q_k_1.w+pFilter->mN*pSensor->fMag[X]*pFilter->q_k_1.w+pFilter->mD*pSensor->fMag[Z]*pFilter->q_k_1.w+pFilter->mD*pSensor->fMag[Y]*pFilter->q_k_1.x-pFilter->mD*pSensor->fMag[X]*pFilter->q_k_1.y+pFilter->mN*pSensor->fMag[Z]*pFilter->q_k_1.y-pFilter->mN*pSensor->fMag[Y]*pFilter->q_k_1.z)/4.0;
	pFilter->J[i++] = (-((1+pFilter->mN*pSensor->fMag[X])*pFilter->q_k_1.x)-pFilter->mD*(pSensor->fMag[Y]*pFilter->q_k_1.w-pSensor->fMag[Z]*pFilter->q_k_1.x+pSensor->fMag[X]*pFilter->q_k_1.z)-pFilter->mN*(pSensor->fMag[Y]*pFilter->q_k_1.y+pSensor->fMag[Z]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pSensor->fAccel[Y]*(pFilter->mN*pFilter->q_k_1.w-pFilter->mD*pFilter->q_k_1.y)-(-1+pSensor->fAccel[Z])*(pFilter->mN*pFilter->q_k_1.x+pFilter->mD*pFilter->q_k_1.z)+pSensor->fAccel[X]*(pFilter->mD*pFilter->q_k_1.x-pFilter->mN*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->mD*(pFilter->q_k_1.w-pSensor->fAccel[Z]*pFilter->q_k_1.w+pSensor->fAccel[Y]*pFilter->q_k_1.x+pSensor->fAccel[X]*pFilter->q_k_1.y)-pFilter->mN*(pSensor->fAccel[X]*pFilter->q_k_1.w+(-1+pSensor->fAccel[Z])*pFilter->q_k_1.y+pSensor->fAccel[Y]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pSensor->fAccel[Y]*(pFilter->mD*pFilter->q_k_1.w+pFilter->mN*pFilter->q_k_1.y)+pFilter->mD*((-1+pSensor->fAccel[Z])*pFilter->q_k_1.x+pSensor->fAccel[X]*pFilter->q_k_1.z)+pFilter->mN*(pSensor->fAccel[X]*pFilter->q_k_1.x+pFilter->q_k_1.z-pSensor->fAccel[Z]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (-((1+pFilter->mN*pSensor->fMag[X]+pFilter->mD*pSensor->fMag[Z])*pFilter->q_k_1.w)-pFilter->mD*pSensor->fMag[Y]*pFilter->q_k_1.x+pFilter->mD*pSensor->fMag[X]*pFilter->q_k_1.y-pFilter->mN*pSensor->fMag[Z]*pFilter->q_k_1.y+pFilter->mN*pSensor->fMag[Y]*pFilter->q_k_1.z)/4.0;
	pFilter->J[i++] = (pFilter->q_k_1.z-pFilter->mN*(pSensor->fMag[Y]*pFilter->q_k_1.w-pSensor->fMag[Z]*pFilter->q_k_1.x+pSensor->fMag[X]*pFilter->q_k_1.z)+pFilter->mD*(pSensor->fMag[X]*pFilter->q_k_1.x+pSensor->fMag[Y]*pFilter->q_k_1.y+pSensor->fMag[Z]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (-pFilter->q_k_1.y-pFilter->mN*(pSensor->fMag[Z]*pFilter->q_k_1.w+pSensor->fMag[Y]*pFilter->q_k_1.x-pSensor->fMag[X]*pFilter->q_k_1.y)+pFilter->mD*(pSensor->fMag[X]*pFilter->q_k_1.w+pSensor->fMag[Z]*pFilter->q_k_1.y-pSensor->fMag[Y]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->mD*((-1+pSensor->fAccel[Z])*pFilter->q_k_1.w+pSensor->fAccel[Y]*pFilter->q_k_1.x+pSensor->fAccel[X]*pFilter->q_k_1.y)-pFilter->mN*(pSensor->fAccel[X]*pFilter->q_k_1.w+pFilter->q_k_1.y-pSensor->fAccel[Z]*pFilter->q_k_1.y+pSensor->fAccel[Y]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pSensor->fAccel[Y]*(-(pFilter->mN*pFilter->q_k_1.w)+pFilter->mD*pFilter->q_k_1.y)-(-1+pSensor->fAccel[Z])*(pFilter->mN*pFilter->q_k_1.x+pFilter->mD*pFilter->q_k_1.z)+pSensor->fAccel[X]*(-(pFilter->mD*pFilter->q_k_1.x)+pFilter->mN*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->mN*(pFilter->q_k_1.w-pSensor->fAccel[Z]*pFilter->q_k_1.w+pSensor->fAccel[Y]*pFilter->q_k_1.x)-pSensor->fAccel[X]*(pFilter->mD*pFilter->q_k_1.w+pFilter->mN*pFilter->q_k_1.y)+pFilter->mD*((-1+pSensor->fAccel[Z])*pFilter->q_k_1.y+pSensor->fAccel[Y]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->q_k_1.x+pFilter->mN*pSensor->fMag[X]*pFilter->q_k_1.x+pFilter->mN*pSensor->fMag[Y]*pFilter->q_k_1.y+pFilter->mN*pSensor->fMag[Z]*pFilter->q_k_1.z+pFilter->mD*(pSensor->fMag[Y]*pFilter->q_k_1.w-pSensor->fMag[Z]*pFilter->q_k_1.x+pSensor->fMag[X]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->q_k_1.y+pFilter->mN*(pSensor->fMag[Z]*pFilter->q_k_1.w+pSensor->fMag[Y]*pFilter->q_k_1.x-pSensor->fMag[X]*pFilter->q_k_1.y)-pFilter->mD*(pSensor->fMag[X]*pFilter->q_k_1.w+pSensor->fMag[Z]*pFilter->q_k_1.y-pSensor->fMag[Y]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pFilter->q_k_1.z-pFilter->mN*(pSensor->fMag[Y]*pFilter->q_k_1.w-pSensor->fMag[Z]*pFilter->q_k_1.x+pSensor->fMag[X]*pFilter->q_k_1.z)+pFilter->mD*(pSensor->fMag[X]*pFilter->q_k_1.x+pSensor->fMag[Y]*pFilter->q_k_1.y+pSensor->fMag[Z]*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (-(pSensor->fAccel[Y]*(pFilter->mD*pFilter->q_k_1.w+pFilter->mN*pFilter->q_k_1.y))+pSensor->fAccel[X]*(pFilter->mN*pFilter->q_k_1.x+pFilter->mD*pFilter->q_k_1.z)+(1+pSensor->fAccel[Z])*(pFilter->mD*pFilter->q_k_1.x-pFilter->mN*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = ((1+pSensor->fAccel[Z])*(-(pFilter->mN*pFilter->q_k_1.w)+pFilter->mD*pFilter->q_k_1.y)+pSensor->fAccel[X]*(pFilter->mD*pFilter->q_k_1.w+pFilter->mN*pFilter->q_k_1.y)+pSensor->fAccel[Y]*(pFilter->mN*pFilter->q_k_1.x+pFilter->mD*pFilter->q_k_1.z))/4.0;
	pFilter->J[i++] = (pSensor->fAccel[Y]*(pFilter->mN*pFilter->q_k_1.w-pFilter->mD*pFilter->q_k_1.y)+(1+pSensor->fAccel[Z])*(pFilter->mN*pFilter->q_k_1.x+pFilter->mD*pFilter->q_k_1.z)+pSensor->fAccel[X]*(-(pFilter->mD*pFilter->q_k_1.x+pFilter->mN*pFilter->q_k_1.z)))/4.0;
	mat_mult_f64(&(pFilter->matrix_J), &(pFilter->matrix_Epsilon_acc_mag), &(pFilter->matrix_Temp4x6I));
	mat_trans_f64(&(pFilter->matrix_J), &(pFilter->matrix_Temp6x4I));
	mat_mult_f64(&(pFilter->matrix_Temp4x6I), &(pFilter->matrix_Temp6x4I), &(pFilter->matrix_R_k));

	//correct steps
	//calculate kalman gain
	mat_add_f64(&(pFilter->matrix_P_k_),&(pFilter->matrix_R_k),&(pFilter->matrix_Temp4x4I));
	mat_inverse_f64(&(pFilter->matrix_Temp4x4I), &(pFilter->matrix_Temp4x4II));
	mat_mult_f64(&(pFilter->matrix_P_k_), &(pFilter->matrix_Temp4x4II), &(pFilter->matrix_K_k));
	//calculate q_k
	mat_sub_f64(&(pFilter->matrix_z_k),&(pFilter->matrix_q_k_),&(pFilter->matrix_Temp4x1I));
	mat_mult_f64(&(pFilter->matrix_K_k), &(pFilter->matrix_Temp4x1I), &(pFilter->matrix_Temp4x1II));
	mat_add_f64(&(pFilter->matrix_q_k_),&(pFilter->matrix_Temp4x1II),&(pFilter->matrix_q_k));
	quat_normlise_f64(&(pFilter->q_k));
	//calculate P_k
	mat_sub_f64(&(pFilter->matrix_I4x4),&(pFilter->matrix_K_k),&(pFilter->matrix_Temp4x4I));
	mat_mult_f64(&(pFilter->matrix_Temp4x4I), &(pFilter->matrix_P_k_), &(pFilter->matrix_P_k));

	//settings for next turn
	mat_duplicate_f64(&(pFilter->matrix_P_k),&(pFilter->matrix_P_k_1));
	mat_duplicate_f64(&(pFilter->matrix_q_k),&(pFilter->matrix_q_k_1));
}
