#include "chassis_task.h"
#include "detect_task.h"
#include "Waypoint_task.h"
/*'''

　　┏┓　　　┏┓+ +
　┏┛┻━━━┛┻┓ + +
　┃　　　　　　　┃ 　
　┃　　　━　　　┃ ++ + + +
 ████━████ ┃+
　┃　　　　　　　┃ +
　┃　　　┻　　　┃
　┃　　　　　　　┃ + +
　┗━┓　　　┏━┛
　　　┃　　　┃　　　　　　　　　　　
　　　┃　　　┃ + + + +
　　　┃　　　┃
　　　┃　　　┃ +  神兽保佑
　　　┃　　　┃    代码无bug　　
　　　┃　　　┃　　+　　　　　　　　　
　　　┃　 　　┗━━━┓ + +
　　　┃ 　　　　　　　┣┓
　　　┃ 　　　　　　　┏┛
　　　┗┓┓┏━┳┓┏┛ + + + +
　　　　┃┫┫　┃┫┫
　　　　┗┻┛　┗┻┛+ + + +
'''*/

Waypoints_t Waypoint[50];
void Sensor_Get(void);
osThreadId waypoint_TASKHandle;

void waypoint_Task(void *pvParameters){
	Waypoint_Init();
	while(1)
	{
		Sensor_Get();
		vTaskDelay(5);
	}
}

void Waypoint_Init(void)
{
	Waypoint_Write(0,-120.0f , 140.0f); //取竹子
	/*左->右 地标*/ 
	Waypoint_Write(1,-130.0f, 255.0f);  //放first
	
	Waypoint_Write(2,-103.0f, 205.0f);  // 转点
	
	/*右->左 竹子*/
	/*第一个*/
	Waypoint_Write(3,-105.0f , 250.0f); // 放second
	
	/*回弹*/
	Waypoint_Write(4,-100.0f , 185.0f); //后退
	
	/*第二个*/
	Waypoint_Write(5,-60.0f , 205.0f);  //取竹子
	
										//后退
	Waypoint_Write(6,-71.0f , 255.0f); //放first

	Waypoint_Write(7,-45.0f, 200.0f);  // 转点	
	
	Waypoint_Write(8,-45.0f, 250.0f);  // 放second
	
	//重试之后的夹取的竹子
	
	Waypoint_Write(9,-120.0f , 167.0f);//取竹子

	Waypoint_Write(10,-70.0f , 200.0f);//取竹子
	
	Waypoint_Write(11,-90.0f , 145.0f); // 转点

	Waypoint_Write(12,-89.0f , 35.0f);//放first

	Waypoint_Write(13,-142.0f , 140.0f);// 转点

	Waypoint_Write(14,-139.0f , 35.0f);// 放second

	Waypoint_Write(15,159.0f , 140.0f);//转点

	Waypoint_Write(16,156.0f , 20.0f);// 放first

	Waypoint_Write(17,93.0f , 130.0f);//转点
	
	Waypoint_Write(18,90.0f , 20.0f);// 放second
	
///////////////////////////////////////////////////////////////
	
//	Waypoint_Write(19,103, 118.0f);// 档它

	
	
//	/*最左边的竹子对应的蓝色区域的竹子 逆时针*/
//	Waypoint_Write(6,-135.0f , 95.0f);
//	Waypoint_Write(7,-120.0f , 95.0f);
//	Waypoint_Write(8,-105.0f , 95.0f);
//	Waypoint_Write(9,-95.0f , 95.0f);
//	Waypoint_Write(10,-75.0f  , 95.0f);
//	Waypoint_Write(11,-60.0f  , 95.0f);
//	Waypoint_Write(12,-45.0f  , 95.0f);
//	Waypoint_Write(13,-30.0f  , 95.0f);
//	Waypoint_Write(14,-15.0f  , 95.0f);
//	Waypoint_Write(15,0.0f  , 95.0f);
//	Waypoint_Write(16, 15.0f  , 95.0f);
//	Waypoint_Write(17, 30.0f  , 95.0f);
//	Waypoint_Write(18, 45.0f  , 95.0f);
//	Waypoint_Write(19, 60.0f  , 95.0f);
//	Waypoint_Write(20, 75.0f  , 95.0f);
//	Waypoint_Write(21, 90.0f  , 95.0f);
//	Waypoint_Write(22,105.0f , 95.0f);
//	Waypoint_Write(23,120.0f , 95.0f);
//	Waypoint_Write(24,135.0f , 95.0f);
//	Waypoint_Write(25,150.0f , 95.0f);
//	Waypoint_Write(26,165.0f , 95.0f);
//	Waypoint_Write(27,180.0f , 95.0f);
//	Waypoint_Write(28,-170.0f , 95.0f);
//	Waypoint_Write(29,-150.0f , 95.0f);
}

void Waypoint_Write(uint8_t number,float Plo_Angle,float Plo_Length)
{
	Waypoint[number].Plo_Angle		= Plo_Angle;
	Waypoint[number].Plo_Length 	= Plo_Length;
}


inline uint8_t GD_key_3_get(void)
{
	if(HAL_GPIO_ReadPin(GD3_GPIO_Port,GD3_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t GD_key_2_get(void)
{
	if(HAL_GPIO_ReadPin(GD2_GPIO_Port,GD2_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t GD_key_1_get(void)
{
	if(HAL_GPIO_ReadPin(GD1_GPIO_Port,GD1_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t touch_key_1_get(void)
{
	if(HAL_GPIO_ReadPin(TOUCH1_GPIO_Port,TOUCH1_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t touch_key_2_get(void)
{
	if(HAL_GPIO_ReadPin(TOUCH2_GPIO_Port,TOUCH2_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t touch_key_3_get(void)
{
	if(HAL_GPIO_ReadPin(TOUCH3_GPIO_Port,TOUCH3_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}

void Sensor_Get(void)
{
	chassis_move.GD_flag[0]=GD_key_1_get();
	chassis_move.GD_flag[1]=GD_key_2_get();
	chassis_move.GD_flag[2]=GD_key_3_get();
	chassis_move.touch_flag[0]=touch_key_1_get();
	chassis_move.touch_flag[1]=touch_key_2_get();
	chassis_move.touch_flag[2]=touch_key_3_get();
}
