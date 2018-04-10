
#include "MPU6050_dmp_analysis.h"

void Compare_data(MPU6050_data_t *data_old, MPU6050_data_t *data_new) {
	if(data_new->Accelerometer_X * data_old->Accelerometer_X <0)  // so sanh trai dau 
		{
			//check goc 
		}
}