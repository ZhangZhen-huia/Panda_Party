#pragma once 

/*自定义任务创建函数，用于创建任务，参数c：capital全大写；l：lowercase首字母大写*/
//	 	mTaskCreate(START,Start);
#define mTaskCreate(c,l)    xTaskCreate((TaskFunction_t )l##_Task,		\
										(const char*    )#l,		\
										(uint16_t       )c##_STK_SIZE,	\
										(void*          )NULL,			\
										(UBaseType_t    )c##_TASK_PRIO,	\
										(TaskHandle_t*  )&l##_TASKHandle)




										