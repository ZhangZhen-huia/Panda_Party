#include "main.h"
#include "freertos.h"
#include "my_task.h"
#include "task.h"
#include "cmsis_os.h"
#include "chassis_task.h"

extern osThreadId Start_TASKHandle;

/*呼吸灯任务*/
/*-摘要-*/ #define LED
	/*-优先-*/ #define LED_TASK_PRIO 1
	/*-堆栈-*/ #define LED_STK_SIZE 100
/*-声明-*/extern void led_Task(void *pvParameters);
extern osThreadId led_TASKHandle;

/*遥控器检测任务*/
/*-摘要-*/ #define DETECT
	/*-优先-*/ #define DETECT_TASK_PRIO 5
	/*-堆栈-*/ #define DETECT_STK_SIZE 100
/*-声明-*/extern void detect_Task(void *pvParameters);
extern osThreadId detect_TASKHandle;

/*底盘任务*/
/*-摘要-*/ #define CHASSIS
	/*-优先-*/ #define CHASSIS_TASK_PRIO 2
	/*-堆栈-*/ #define CHASSIS_STK_SIZE 1000
/*-声明-*/extern void chassis_Task(void *pvParameters);
extern osThreadId chassis_TASKHandle;

/*抬升任务*/
/*-摘要-*/ #define LIFT
	/*-优先-*/ #define LIFT_TASK_PRIO 2
	/*-堆栈-*/ #define LIFT_STK_SIZE 300
/*-声明-*/extern void lift_Task(void *pvParameters);
extern osThreadId lift_TASKHandle;

/*遥控器任务*/
/*-摘要-*/ #define REMOTE
	/*-优先-*/ #define REMOTE_TASK_PRIO 4
	/*-堆栈-*/ #define REMOTE_STK_SIZE 200
/*-声明-*/extern void remote_Task(void *pvParameters);
extern osThreadId remote_TASKHandle;

/*换点任务*/
/*-摘要-*/ #define WAYPOINT
	/*-优先-*/ #define WAYPOINT_TASK_PRIO 3
	/*-堆栈-*/ #define WAYPOINT_STK_SIZE 100
/*-声明-*/extern void waypoint_Task(void *pvParameters);
extern osThreadId waypoint_TASKHandle;


void All_Init(void)
{
	canfilter_init_start();
	UART_IDLE_init();
	//爪子初始化
	chassis_move.Paw_flag = 5;
	while(HAL_UART_Transmit(&huart7,&chassis_move.Paw_flag,1,100)!=HAL_OK);
	HAL_UART_Transmit_IT(&huart7,&chassis_move.Paw_flag,1);
	HAL_UART_Receive_IT(&huart8,uwb_receive_data,8);
	
}
                                                                        
void START_Task(void const * argument)
{
	All_Init();
	taskENTER_CRITICAL(); //进入临界区
	//创建任务
	mTaskCreate(DETECT,detect);
	mTaskCreate(CHASSIS,chassis);
	mTaskCreate(LIFT,lift);
	mTaskCreate(REMOTE,remote);
	mTaskCreate(WAYPOINT,waypoint);
	mTaskCreate(LED,led);
	//完事把自己删了
	vTaskDelete(Start_TASKHandle);
	taskEXIT_CRITICAL(); //进入临界区
	
}


