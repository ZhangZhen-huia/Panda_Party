#pragma once 

/*�Զ������񴴽����������ڴ������񣬲���c��capitalȫ��д��l��lowercase����ĸ��д*/
//	 	mTaskCreate(START,Start);
#define mTaskCreate(c,l)    xTaskCreate((TaskFunction_t )l##_Task,		\
										(const char*    )#l,		\
										(uint16_t       )c##_STK_SIZE,	\
										(void*          )NULL,			\
										(UBaseType_t    )c##_TASK_PRIO,	\
										(TaskHandle_t*  )&l##_TASKHandle)




										