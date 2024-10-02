#include "main.h"
#include "freertos.h"
#include "my_task.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "chassis_task.h"

osThreadId usart_TASKHandle;

void usart_Task(void *pvParameters){
	//×¦×Ó³õÊ¼»¯
	chassis_move.Paw_flag = 5;
	while(HAL_UART_Transmit(&huart7,&chassis_move.Paw_flag,1,100)!=HAL_OK);
	HAL_UART_Transmit_IT(&huart7,&chassis_move.Paw_flag,1);
	HAL_UART_Receive_IT(&huart8,uwb_receive_data,8);
}


