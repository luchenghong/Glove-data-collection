#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include "matrix.h"

#ifdef __cplusplus
extern "C" {
#endif

#define X 0
#define Y 1
#define Z 2

typedef struct{
	double  deltaT;
	uint8_t rawBuffer[22];
	int16_t iAccel[3];
	int16_t iGyro[3];
	int16_t iMag[3];
	int16_t	iTemp;
	double   fAccel[3];
	double   fGyro[3];
	double   fMag[3];
	double   U[9];
	double   C[3];
	double   fMagc[3];

	mat_instance_f64 matrix_fAccel;
	mat_instance_f64 matrix_fGyro;
	mat_instance_f64 matrix_fMag;
	mat_instance_f64 matrix_U;
	mat_instance_f64 matrix_C;
	mat_instance_f64 matrix_fMagc;
	double   fTemp;
}sensor_data;

extern void sensor_object_init(sensor_data* pSensor);

#ifdef __cplusplus
}
#endif

#endif
