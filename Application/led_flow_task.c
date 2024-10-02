#include "led_flow_task.h"
#include "tim.h"
#include "main.h"
#include "gpio.h"
#include "freertos.h"
#include "cmsis_os.h"

osThreadId led_TASKHandle;

void led_Task(void *pvParameters){
	uint32_t a=0;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	while(1)
	{
		for(a=0;a<1000;a+=2)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,a);
				osDelay(2);
			}
				for(a=1000;a>0;a-=2)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,a);
				osDelay(2);
			};
	}
	vTaskDelay(25);
}
