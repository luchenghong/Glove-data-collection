#include "sensor.h"

void sensor_object_init(sensor_data* pSensor)
{
	mat_init_f64(&(pSensor->matrix_fAccel), 3, 1, pSensor->fAccel);
	mat_init_f64(&(pSensor->matrix_fGyro), 3, 1, pSensor->fGyro);
	mat_init_f64(&(pSensor->matrix_fMag), 3, 1, pSensor->fMag);
	mat_init_f64(&(pSensor->matrix_U), 3, 3, pSensor->U);
	mat_init_f64(&(pSensor->matrix_C), 3, 1, pSensor->C);
	mat_init_f64(&(pSensor->matrix_fMagc), 3, 1, pSensor->fMagc);
}

