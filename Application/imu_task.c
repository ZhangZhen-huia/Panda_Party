#include "cmsis_os.h"
#include "gpio.h"
#include "freertos.h"
#include "bsp_imu.h"

extern imu_t              imu;
char buf[300];
int count;

//void imu_task(void const * argument)
//{
//	while(1)
//	{
//	mpu_get_data();
//	imu_ahrs_update();
//	imu_attitude_update(); 
//	osDelay(5);
//	}
//}

