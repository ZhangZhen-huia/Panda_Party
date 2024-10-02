#include "main.h"
#include "freertos.h"
#include "my_task.h"
#include "task.h"
#include "cmsis_os.h"
#include "chassis_task.h"

extern osThreadId Start_TASKHandle;

/*����������*/
/*-ժҪ-*/ #define LED
	/*-����-*/ #define LED_TASK_PRIO 1
	/*-��ջ-*/ #define LED_STK_SIZE 100
/*-����-*/extern void led_Task(void *pvParameters);
extern osThreadId led_TASKHandle;

/*ң�����������*/
/*-ժҪ-*/ #define DETECT
	/*-����-*/ #define DETECT_TASK_PRIO 5
	/*-��ջ-*/ #define DETECT_STK_SIZE 100
/*-����-*/extern void detect_Task(void *pvParameters);
extern osThreadId detect_TASKHandle;

/*��������*/
/*-ժҪ-*/ #define CHASSIS
	/*-����-*/ #define CHASSIS_TASK_PRIO 2
	/*-��ջ-*/ #define CHASSIS_STK_SIZE 1000
/*-����-*/extern void chassis_Task(void *pvParameters);
extern osThreadId chassis_TASKHandle;

/*̧������*/
/*-ժҪ-*/ #define LIFT
	/*-����-*/ #define LIFT_TASK_PRIO 2
	/*-��ջ-*/ #define LIFT_STK_SIZE 300
/*-����-*/extern void lift_Task(void *pvParameters);
extern osThreadId lift_TASKHandle;

/*ң��������*/
/*-ժҪ-*/ #define REMOTE
	/*-����-*/ #define REMOTE_TASK_PRIO 4
	/*-��ջ-*/ #define REMOTE_STK_SIZE 200
/*-����-*/extern void remote_Task(void *pvParameters);
extern osThreadId remote_TASKHandle;

/*��������*/
/*-ժҪ-*/ #define WAYPOINT
	/*-����-*/ #define WAYPOINT_TASK_PRIO 3
	/*-��ջ-*/ #define WAYPOINT_STK_SIZE 100
/*-����-*/extern void waypoint_Task(void *pvParameters);
extern osThreadId waypoint_TASKHandle;


void All_Init(void)
{
	canfilter_init_start();
	UART_IDLE_init();
	//צ�ӳ�ʼ��
	chassis_move.Paw_flag = 5;
	while(HAL_UART_Transmit(&huart7,&chassis_move.Paw_flag,1,100)!=HAL_OK);
	HAL_UART_Transmit_IT(&huart7,&chassis_move.Paw_flag,1);
	HAL_UART_Receive_IT(&huart8,uwb_receive_data,8);
	
}
                                                                        
void START_Task(void const * argument)
{
	All_Init();
	taskENTER_CRITICAL(); //�����ٽ���
	//��������
	mTaskCreate(DETECT,detect);
	mTaskCreate(CHASSIS,chassis);
	mTaskCreate(LIFT,lift);
	mTaskCreate(REMOTE,remote);
	mTaskCreate(WAYPOINT,waypoint);
	mTaskCreate(LED,led);
	//���°��Լ�ɾ��
	vTaskDelete(Start_TASKHandle);
	taskEXIT_CRITICAL(); //�����ٽ���
	
}


