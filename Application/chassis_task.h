#pragma once 

#include "main.h"
#include "Can_recive.h"
#include "cmsis_os.h"
#include "usart.h"
#include "remote_task.h"
#include "math.h"
#include "filter.h"
#include "Paw_Task.h"
#include "bsp_imu.h"
#include "lift_task.h"
#include "detect_task.h"
#include "math.h"
#include "filter.h"
#include "Waypoint_task.h"
#include "PAW_TASK.h"
#include "base_usart.h"
#include "main.h"
#include "tim.h"



typedef enum
{
	Chassis_Automatic=0,
	Chassis_Remote,
}CHASSIS_MODE; //底盘模式

typedef enum
{
	TOP = 0,
	ALL_DIRECTION=1,
	HALT = 2,
}Auto_mode;

typedef enum
{
	begin = 0,
	reset=9,
}my_mode;

typedef enum
{
	CHASSIS_ZERO_FORCE = 0, //无力模式
	CHASSIS_RC_GYROSCOPE = 1, //遥控器 小陀螺模式
	CHASSIS_RC_FOLLOW_GIMBAL = 2, //遥控器 跟随云台模式
	CHASSIS_PC_CONTROL = 3, //PC 键鼠模式
		
}REMOTE_MODE; //遥控器模式


/*************底盘状态是否到达设定状态枚举****************/
typedef enum
{
    YES,      //位置接近
    NO,     //位置未接近

} chassis_statu_e;


typedef struct
{
  const Revice_Motro *chassis_motor_measure;//电机数据反馈值
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef __packed struct
{
    int16_t x;
    int16_t y;
		uint16_t yaw;
}uwb_info_t;
typedef struct
{
	 uwb_info_t uwb_data;
	const RC_ctrl_t *chassis_RC;               					//底盘使用的遥控器指针, the point to remote control
  chassis_motor_t motor_chassis[8];          					//chassis motor data.底盘电机数据
	
  pid_type_def chassis_3508_angle_pid[4];             //motor speed PID.底盘电机速度pid
  pid_type_def chassis_3508_speed_pid[4];             //motor speed PID.底盘电机速度pid
  pid_type_def chassis_6020_angle_pid[4];             //底盘6020角度pid
  pid_type_def chassis_6020_speed_pid[4];							//底盘6020速度pid
	
  fp32 vx;                         									  //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          							  	//底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          							  	//底盘旋转角速度，逆时针为正 单位 rad/s

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vz;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_3508;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
	
  ramp_function_source_t chassis_ramp;  			 				//斜波
	
  REMOTE_MODE 	my_remote_mode;													//遥控器模式状态机 
	CHASSIS_MODE  my_chassis_mode;												//底盘控制状态机
	Auto_mode			my_auto_mode;														//自动模式控制状态机
	
	
  fp32 vx_set;                      									//底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                      									//底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      									//底盘设定旋转角速度，逆时针为正 单位 rad/s
	
  int8_t drct;																				//决定驱动电机正反转
	
  uint8_t Remote_init_mode;														//遥控器初始化变量
	uint8_t Auto_init_mode;															//自动初始化变量
	
  uint8_t angle_ready;        												//3508等待6020转动到指定角度标志位
  
  fp32 AGV_wheel_Angle[4];       											 //6020最终计算出的角度
  fp32 wheel_speed[4]; 																//3508最终计算出的速度
	
  fp32 vx_set_channel, vy_set_channel,vz_set_channel;    //设定转速范围： +-8911
  int32_t vx_channel,vy_channel,vz_channel;         		 //接收遥控器数据
	
	fp32 feed_back_X;           														//底盘X反馈       单位  cm  
  fp32 feed_back_Y;          														//底盘Y反馈       单位  cm 
  fp32 feed_back_A;          													  //底盘朝向A反馈   单位  度 （-180 — 180）

  fp32 target_X;          														 //最终底盘X目标位置   单位  cm
  fp32 target_Y;          														 //最终底盘Y目标位置   单位  cm
  int16_t target_A;          														 //最终底盘A目标朝向   单位  度 （-180 — 180）

	fp32 derta_X,derta_Y,derta_ecd,derta_length;								//目标x y坐标 ecd差值 距离差值
	fp32 current_distence;														//当前走过的距离 单位：cm
	int8_t position_judge_flag;
	
	uint8_t Paw_flag;
	circle_number_lift_flag lift_circle_number;
	uint8_t GD_flag[3];
	uint8_t touch_flag[3];
	int8_t num_over[10];
	int8_t lift_over;
	int8_t step[5];
	
	my_mode begin_reset;
} chassis_move_t;


extern chassis_move_t chassis_move;//底盘运动数据

extern osThreadId LED_FLOW_TASKHandle;

/*******************************一节低通滤波参数************************/
#define CHASSIS_CONTROL_TIME 0.012f   //x和y本次信任参数
#define CHASSIS_CONTROL_TIME_Z 0.005f  //z本次信任参数
#define CHASSIS_CONTROL_TIME_3508 0.1f   //3508本次信任参数

//信任上一次参数占比
#define CHASSIS_ACCEL_X_NUM 0.98f
#define CHASSIS_ACCEL_Y_NUM 0.98f
#define CHASSIS_ACCEL_Z_NUM 0.98f
#define CHASSIS_ACCEL_3508_NUM 0.90f

/*******************************轮组数据*******************************/
#define R       MOTOR_DISTANCE_TO_CENTER
#define PI       3.1415926f

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191

//小陀螺半径 m 
#define MOTOR_DISTANCE_TO_CENTER 0.235619445f

//小陀螺周长
#define SMALL_TOP_CIRCUMFERENCE	 	MOTOR_DISTANCE_TO_CENTER*2*3.1415926f			//1.480440609656214f

//轮子半径  m
#define WHEEL_HALF_SIZE 	0.0375f

//轮子周长	m
#define WHEEL_CIRCUMFERENCE				0.235619445f          //WHEEL_HALF_SIZE*2*3.1415926f  	

//前进最大速度  1.84175866175m/s   --8911
//									0						0

//减速比19，rpm: 圈/min
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例	
#define CHASSIS_VX_RC_SEN            19.0f/60.0f*WHEEL_CIRCUMFERENCE

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE / SMALL_TOP_CIRCUMFERENCE/2.0f/PI			


/**************************************ECD初始化************************/
//小陀螺 
#define GIM_TOP_ECD1				7068.0f
#define GIM_TOP_ECD2				2916.0f
#define GIM_TOP_ECD3				4285.0f
#define GIM_TOP_ECD4				1311.0f

//y轴
#define GIM_Y_ECD_1				8116.0f
#define GIM_Y_ECD_2				5960.0f // 6024
#define GIM_Y_ECD_3				1382.0f // 7515
#define GIM_Y_ECD_4				390.0f	//1682

//自动
#define GIM_AUTO_ECD_1				4021.0f
#define GIM_AUTO_ECD_2				1865.0f
#define GIM_AUTO_ECD_3				5478.0f
#define GIM_AUTO_ECD_4				4538.0f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define ACCELERATION 20.0  // 设定一个加速度值，单位为从cm/s^2


/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_Task(void *pvParameters);



