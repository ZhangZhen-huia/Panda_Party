#include "chassis_task.h"
#include "pid.h"
int num=0;
int8_t slow_down;
uint8_t stop_flag;
int8_t over;
void Angle_Pid_Trace(chassis_move_t *chassis_move_control_loop,float angle);
#include "bsp_imu.h"
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

/*---------------------------------------------------------------------------------------------------------*/
/*																				|	控制模式	|																											 */
/*					遥控器左边	上：遥控器模式											右边：上：原地小陀螺																 */
/*										中：底盘无力模式													中：全向移动																	 */
/*										下：自动模式															下：急停 or 无力															 */
/*---------------------------------------------------------------------------------------------------------*/

chassis_move_t chassis_move;//底盘运动数据



/*--------------------------------------------------遥控器模式-----------------------------------------------*/
// 遥控器模式所有初始化
static void remote_control_chassis_init(chassis_move_t* chassis_move_init);

//遥控器控制模式，获取8个电机的速度并且进行PID计算
static void chassis_remote_control_loop(chassis_move_t *chassis_move_control_loop);

//舵轮6020角度计算,运用atn2函数得到  PI 到 PI 的角度
void Remote_chassis_AGV_wheel_angle(chassis_move_t *chassis_move_control_loop);

//舵轮3508电机速度分解
static void Remote_chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop);

//等待6020角度转到位在转3508
void angle_judge(chassis_move_t *chassis_move_control_loop);

//底盘和遥控器模式选择函数
void Chassis_mode_change(chassis_move_t *chassis_move_rc_to_mode);

//遥控器数值获取加死区限制
void chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector);

//pid计算
void Remote_pid_caculate(chassis_move_t *chassis_move_control_loop);

/*-----------------------------------------------------自动------------------------------------------------------*/
// 自动模式所有初始化
static void automatic_control_chassis_init(chassis_move_t* chassis_move_init);

//自动模式，获取8个电机的速度并且进行PID计算
static void chassis_automatic_control_loop(chassis_move_t *chassis_move_control_loop);

//自动模式下舵轮6020解算
static void Auto_chassis_AGV_wheel_angle(chassis_move_t *chassis_move_control_loop);

//自动模式下舵轮3508解算
static void Auto_chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop);

//PID计算
static void Auto_pid_caculate(chassis_move_t *chassis_move_control_loop);

//计算当前位置Y坐标与目标位置的Y坐标的差值
float Chassis_Me_To_Target_Y_Distance(chassis_move_t *chassis_move_data);

//计算当前位置X坐标与目标位置的X坐标的差值
float Chassis_Me_To_Target_X_Distance(chassis_move_t *chassis_move_data);

//计算当前位置与目标位置的差值
float Chassis_Me_To_Target_Distance(chassis_move_t *chassis_move_data);

//计算当前位置角度与目标位置的角度的差值
int16_t Chassis_Me_To_Target_Angle(chassis_move_t *chassis_move_data);

//极坐标转直角坐标
void Polar_To_Stright(chassis_move_t *chassis_move_data,float plo_angle,float plo_length);

void limit_speed(chassis_move_t *chassis_move_control_loop);

/*-------------------------------------------------------公共-----------------------------------------------------------*/
//判断优弧劣弧-->只转3508，不转6020，把6020角度控制在0-90度内，可以更好的转换运动方向
void Speed_Toggle(chassis_move_t *chassis_move_control_Speed);

//清除PID
static void PID_CLEAR_ALL(chassis_move_t *chassis_move_data);

//将角度范围控制在 0 - 8191
float Angle_Limit (float angle ,float max);

//将电机转子转向内侧时 修正方向
fp32 Find_min_Angle(int16_t angle1,fp32 angle2);

void position_judge(chassis_move_t *chassis_move_control_loop);

void num_0_task(chassis_move_t *chassis_move_data);



/*----------------------------------------------------------------------TASK---------------------------------------------------------------------*/

void chassis_task(void const * argument)
{
	
	chassis_move.Auto_init_mode=1;
	chassis_move.Remote_init_mode=1;
	
	while(1) 
	{
		//底盘控制模式选择
			Chassis_mode_change(&chassis_move);
		
		//模式判断在前，失能底盘在后
		if(toe_is_error(DBUS_TOE))
		{
			CAN_cmd_chassis(0,0,0,0);
			CAN_cmd_gimbal(0,0,0,0);
		}
		else
		{
			angle_judge(&chassis_move);
			CAN_cmd_gimbal(chassis_move.chassis_6020_speed_pid[0].out, chassis_move.chassis_6020_speed_pid[1].out,
														chassis_move.chassis_6020_speed_pid[2].out, chassis_move.chassis_6020_speed_pid[3].out);		
			if(chassis_move.angle_ready)
			{
				CAN_cmd_chassis(chassis_move.chassis_3508_speed_pid[0].out, chassis_move.chassis_3508_speed_pid[1].out,
												chassis_move.chassis_3508_speed_pid[2].out, chassis_move.chassis_3508_speed_pid[3].out);
			}
		}
		osDelay(1);//控制频率为1khz，与can接收中断频率一致
	}
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/



/*****************************************************模式选择函数******************************************/
int num_init=0;
//自动模式和遥控模式选择
void Chassis_mode_change(chassis_move_t *chassis_move_rc_to_mode)
{
	//左边
	switch(chassis_move_rc_to_mode->chassis_RC->rc.s[1])
	{
		//上拨
		case 1:	PID_CLEAR_ALL(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->my_chassis_mode = Chassis_Remote;
						osThreadSuspend(LED_FLOW_TASKHandle);												
						chassis_move_rc_to_mode->Auto_init_mode = 1;
						if(chassis_move_rc_to_mode->Remote_init_mode)
						remote_control_chassis_init(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->Remote_init_mode = 0;											
						chassis_remote_control_loop(chassis_move_rc_to_mode);			
						
						//右边							
						switch(chassis_move_rc_to_mode->chassis_RC->rc.s[0])
						{
							case 1:
								chassis_move_rc_to_mode->my_remote_mode=CHASSIS_RC_GYROSCOPE;break;//小陀螺 上拨1
							case 2:
								chassis_move_rc_to_mode->my_remote_mode=CHASSIS_ZERO_FORCE;break;//下面，无力不动 
							case 3:
								chassis_move_rc_to_mode->my_remote_mode=CHASSIS_RC_FOLLOW_GIMBAL;break;//中间 全向平移
						}
		break;
						
		//下面		
		case 2:	PID_CLEAR_ALL(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->my_chassis_mode = Chassis_Automatic;													
						osThreadResume(LED_FLOW_TASKHandle);																							
						chassis_move_rc_to_mode->Remote_init_mode = 1;
						if(chassis_move_rc_to_mode->Auto_init_mode)     
							automatic_control_chassis_init(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->Auto_init_mode = 0;		
//						if(chassis_move_rc_to_mode->chassis_RC->rc.s[0]==3)
//						{
//							chassis_move_rc_to_mode->begin_reset=begin;
//						}
//						else if(chassis_move_rc_to_mode->chassis_RC->rc.s[0]==2)		// AFTER RESET
//						{
//							chassis_move_rc_to_mode->begin_reset=reset;
//						}
						chassis_automatic_control_loop(chassis_move_rc_to_mode);	
						
		break;
		
		//中间				
		case 3:			
			PID_CLEAR_ALL(chassis_move_rc_to_mode);
			CAN_cmd_chassis(0,0,0,0);
			CAN_cmd_gimbal(0,0,0,0);
			break;
	}
}
/*-----------------------------------------------------------------------------------------------------------------------*/
/**************************************************自动模式相关函数********************************************************/
/*-----------------------------------------------------------------------------------------------------------------------*/

//初始化函数
static void automatic_control_chassis_init(chassis_move_t* chassis_move_init)
{
	uint8_t i=0;
	static float SPEED_3508[3]={AUTO_3508_SPEED_PID_KP,AUTO_3508_SPEED_PID_KI,AUTO_3508_SPEED_PID_KD};//PID参数设置
	static float ANGLE_3508[3]={AUTO_3508_ANGLE_PID_KP,AUTO_3508_ANGLE_PID_KI,AUTO_3508_ANGLE_PID_KD};//PID参数设置
	static float ANGLE_6020[3]={M6020_MOTOR_ANGLE_PID_KP_AUTO,M6020_MOTOR_ANGLE_PID_KI_AUTO,M6020_MOTOR_ANGLE_PID_KD_AUTO};//PID参数设置
	static float SPEED_6020[3]={M6020_MOTOR_SPEED_PID_KP_AUTO,M6020_MOTOR_SPEED_PID_KI_AUTO,M6020_MOTOR_SPEED_PID_KD_AUTO};//PID参数设置
	const static fp32 chassis_3508_order_filter[1] = {CHASSIS_ACCEL_3508_NUM};

	//chassis_move_init->drct=1;
	chassis_move_init->angle_ready=0;
	chassis_move_init->my_auto_mode = ALL_DIRECTION;
	/**初始化3508双环PID 并获取电机数据**/
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);//获取底盘3508的数据，接收电机的反馈结构体
		//位置环
		PID_init(&chassis_move_init->chassis_3508_angle_pid[i],PID_POSITION,ANGLE_3508,AUTO_3508_ANGLE_PID_MAX_OUT,AUTO_3508_ANGLE_PID_MAX_IOUT);//初始化底盘PID
		//速度环
		PID_init(&chassis_move_init->chassis_3508_speed_pid[i],PID_POSITION,SPEED_3508,AUTO_3508_SPEED_PID_MAX_OUT,AUTO_3508_SPEED_PID_MAX_IOUT);//初始化底盘PID
		chassis_move_init->wheel_speed[i]=0.0f;
	}
	//初始化6020速度和角度PID并获取电机数据
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i+4].chassis_motor_measure = get_gimbal_motor_measure_point(i);//获取航向电机的数据，接收电机的反馈结构体
		PID_init(&chassis_move_init->chassis_6020_speed_pid[i],PID_POSITION,SPEED_6020,M6020_MOTOR_SPEED_PID_MAX_OUT_AUTO,M6020_MOTOR_SPEED_PID_MAX_IOUT_AUTO);//初始化速度PID
		PID_init(&chassis_move_init->chassis_6020_angle_pid[i],PID_POSITION,ANGLE_6020,M6020_MOTOR_ANGLE_PID_MAX_OUT_AUTO,M6020_MOTOR_ANGLE_PID_MAX_IOUT_AUTO);//初始化角度PID
		chassis_move_init->AGV_wheel_Angle[i]=0.0f;
	}
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_3508, CHASSIS_CONTROL_TIME_3508, chassis_3508_order_filter);
}

int position_judge_flag=0;

//自动模式下舵轮3508解算
static void Auto_chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop)
{
	chassis_move_control_loop->derta_length = Chassis_Me_To_Target_Distance(chassis_move_control_loop);
			for(int i=0;i<4;i++)
		{
			chassis_move_control_loop->wheel_speed[i] = chassis_move_control_loop->drct*chassis_move_control_loop->derta_length;

		}
}

extern int16_t final_yaw;
//自动模式下舵轮6020解算
static void Auto_chassis_AGV_wheel_angle(chassis_move_t *chassis_move_control_loop)
{
	Polar_To_Stright(chassis_move_control_loop,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);
	//6020角度
	chassis_move_control_loop->derta_ecd = chassis_move_control_loop->target_A*22.75f;

			chassis_move_control_loop->AGV_wheel_Angle[0] = Angle_Limit(chassis_move_control_loop->derta_ecd+GIM_AUTO_ECD_1+final_yaw*22.75,ECD_RANGE);
			chassis_move_control_loop->AGV_wheel_Angle[1] = Angle_Limit(chassis_move_control_loop->derta_ecd+GIM_AUTO_ECD_2+final_yaw*22.75,ECD_RANGE);
			chassis_move_control_loop->AGV_wheel_Angle[2] = Angle_Limit(chassis_move_control_loop->derta_ecd+GIM_AUTO_ECD_3+final_yaw*22.75,ECD_RANGE);
			chassis_move_control_loop->AGV_wheel_Angle[3] = Angle_Limit(chassis_move_control_loop->derta_ecd+GIM_AUTO_ECD_4+final_yaw*22.75,ECD_RANGE);

	Speed_Toggle(chassis_move_control_loop);
}

//自动模式pid计算
static void Auto_pid_caculate(chassis_move_t *chassis_move_control_loop)
{
	uint8_t i=0;

	//计算6020双环
	for(i=0;i<4;i++)
	{
			//角度环
			PID_Calc_Angle(&chassis_move_control_loop->chassis_6020_angle_pid[i],
				chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->ecd,chassis_move_control_loop->AGV_wheel_Angle[i]);
		if(chassis_move_control_loop->step[0])
	{
		chassis_move_control_loop->chassis_6020_angle_pid[i].out=0;
	}
			//速度环
			PID_calc(&chassis_move_control_loop->chassis_6020_speed_pid[i],
				chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->rpm,
				chassis_move_control_loop->chassis_6020_angle_pid[i].out);
	//计算3508双环
			limit_speed(chassis_move_control_loop);
			//位置环
		PID_calc(&chassis_move_control_loop->chassis_3508_angle_pid[i],chassis_move_control_loop->wheel_speed[i],0);
			//速度环
		
		if(slow_down)
		{
			chassis_move_control_loop->chassis_3508_angle_pid[i].out=-chassis_move_control_loop->drct*666;
		}
		if(stop_flag)
		{
			chassis_move_control_loop->chassis_3508_angle_pid[i].out=0;
		}

		PID_calc(&chassis_move_control_loop->chassis_3508_speed_pid[i],
				chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->rpm,
				chassis_move_control_loop->chassis_3508_angle_pid[i].out);
}
}

/*-----------------------------------------------------------------------------------------------------------------------*/
/***************************************************遥控器模式相关函数*****************************************************/
/*-----------------------------------------------------------------------------------------------------------------------*/

//初始化
static void remote_control_chassis_init(chassis_move_t* chassis_move_init)
{
	uint8_t i=0;
	
	static float PID_SPEED_3508[3]={M3508_MOTOR_SPEED_PID_KP,M3508_MOTOR_SPEED_PID_KI,M3508_MOTOR_SPEED_PID_KD};//PID参数设置
	static float PID_ANGLE_6020[3]={M6020_MOTOR_ANGLE_PID_KP,M6020_MOTOR_ANGLE_PID_KI,M6020_MOTOR_ANGLE_PID_KD};//PID参数设置
	static float PID_SPEED_6020[3]={M6020_MOTOR_SPEED_PID_KP,M6020_MOTOR_SPEED_PID_KI,M6020_MOTOR_SPEED_PID_KD};//PID参数设置
	
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static fp32 chassis_z_order_filter[1] = {CHASSIS_ACCEL_Z_NUM};
	
	chassis_move_init->drct=1;
	chassis_move_init->angle_ready=0;
	
	/**初始化3508速度PID 并获取电机数据**/
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);//获取底盘3508的数据，接收电机的反馈结构体		
		PID_init(&chassis_move_init->chassis_3508_speed_pid[i],PID_POSITION,PID_SPEED_3508,M3508_MOTOR_SPEED_PID_MAX_OUT,M3508_MOTOR_SPEED_PID_MAX_IOUT);//初始化底盘PID
		chassis_move_init->wheel_speed[i]=0.0f;
	}
	
	//初始化6020速度和角度PID并获取电机数据
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i+4].chassis_motor_measure = get_gimbal_motor_measure_point(i);//获取航向电机的数据，接收电机的反馈结构体
		PID_init(&chassis_move_init->chassis_6020_speed_pid[i],PID_POSITION,PID_SPEED_6020,M6020_MOTOR_SPEED_PID_MAX_OUT,M6020_MOTOR_SPEED_PID_MAX_IOUT);//初始化速度PID
		PID_init(&chassis_move_init->chassis_6020_angle_pid[i],PID_POSITION,PID_ANGLE_6020,M6020_MOTOR_ANGLE_PID_MAX_OUT,M6020_MOTOR_ANGLE_PID_MAX_IOUT);//初始化角度PID
		chassis_move_init->AGV_wheel_Angle[i]=0.0f;
	}
	    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vz, CHASSIS_CONTROL_TIME_Z, chassis_z_order_filter);
}

//遥控器模式下，取8个电机的速度并且进行PID计算
static void chassis_remote_control_loop(chassis_move_t *chassis_move_control_loop)
{
	//遥控器数值获取加死区限制和平均滤波
	chassis_rc_to_control_vector(chassis_move_control_loop); //获取遥控器值

	//舵轮6020角度
	Remote_chassis_AGV_wheel_angle(chassis_move_control_loop); //先计算角度再计算3508速度 因为优劣弧判断

	//舵轮3508运动分解
	Remote_chassis_AGV_wheel_speed(chassis_move_control_loop);

	//pid计算
	Remote_pid_caculate(chassis_move_control_loop);

	//等待6020角度转到位在转3508
	angle_judge(chassis_move_control_loop);
}


//舵轮3508电机速度分解
static void Remote_chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop)
{
	switch(chassis_move_control_loop->my_remote_mode)
	{
		case CHASSIS_RC_GYROSCOPE:
		chassis_move_control_loop->wheel_speed[0] = chassis_move_control_loop->drct*fabs(chassis_move_control_loop->vz_set_channel);// 0-8911				0 - 1.244  圈/s
		chassis_move_control_loop->wheel_speed[1] = chassis_move_control_loop->drct*fabs(chassis_move_control_loop->vz_set_channel);
		chassis_move_control_loop->wheel_speed[2] = chassis_move_control_loop->drct*fabs(chassis_move_control_loop->vz_set_channel);
		chassis_move_control_loop->wheel_speed[3] = chassis_move_control_loop->drct*fabs(chassis_move_control_loop->vz_set_channel);
		chassis_move_control_loop->wz_set = chassis_move_control_loop->vz_set_channel/CHASSIS_WZ_RC_SEN ;//最大->1.24圈/s,转换为rad/s
		break;
		case CHASSIS_RC_FOLLOW_GIMBAL:
		chassis_move_control_loop->vz_set_channel=0;
		chassis_move_control_loop->wheel_speed[0] = chassis_move_control_loop->drct*sqrt(pow(chassis_move_control_loop->vx_set_channel+chassis_move_control_loop->vz_set_channel*R*cos(-45.0),2)+pow((chassis_move_control_loop->vy_set_channel+chassis_move_control_loop->vz_set_channel*R*sin(-45.0)),2));
		chassis_move_control_loop->wheel_speed[1] = chassis_move_control_loop->drct*sqrt(pow(chassis_move_control_loop->vx_set_channel+chassis_move_control_loop->vz_set_channel*R*cos(45.0),2)+pow((chassis_move_control_loop->vy_set_channel+chassis_move_control_loop->vz_set_channel*R*sin(45.0)),2));
		chassis_move_control_loop->wheel_speed[2] = chassis_move_control_loop->drct*sqrt(pow(chassis_move_control_loop->vx_set_channel+chassis_move_control_loop->vz_set_channel*R*cos(135.0),2)+pow((chassis_move_control_loop->vy_set_channel+chassis_move_control_loop->vz_set_channel*R*sin(135.0)),2));
		chassis_move_control_loop->wheel_speed[3] = chassis_move_control_loop->drct*sqrt(pow(chassis_move_control_loop->vx_set_channel+chassis_move_control_loop->vz_set_channel*R*cos(-135.0),2)+pow((chassis_move_control_loop->vy_set_channel+chassis_move_control_loop->vz_set_channel*R*sin(-135.0)),2));
		
		
		chassis_move_control_loop->vx = chassis_move_control_loop->vx_set_channel/CHASSIS_VX_RC_SEN;
		chassis_move_control_loop->vy = chassis_move_control_loop->vy_set_channel/CHASSIS_VY_RC_SEN;
		
		break;
		case CHASSIS_ZERO_FORCE:
		chassis_move_control_loop->wheel_speed[0]=0;
		chassis_move_control_loop->wheel_speed[1]=0;
		chassis_move_control_loop->wheel_speed[2]=0;
		chassis_move_control_loop->wheel_speed[3]=0;
		PID_CLEAR_ALL(chassis_move_control_loop);
		break;
	}
}

//舵轮6020角度计算,运用atn2函数得到  PI 到 PI 的角度
void Remote_chassis_AGV_wheel_angle(chassis_move_t *chassis_move_control_loop)
{	
	static fp64 atan_angle[4];	
	static fp32 AGV_wheel_Angle_last[4];
	switch(chassis_move_control_loop->my_remote_mode)
	{
		case CHASSIS_RC_GYROSCOPE://上拨
			chassis_move_control_loop->AGV_wheel_Angle[0]=	GIM_TOP_ECD1;
			chassis_move_control_loop->AGV_wheel_Angle[1]=	GIM_TOP_ECD2;
			chassis_move_control_loop->AGV_wheel_Angle[2]=	GIM_TOP_ECD3;
			chassis_move_control_loop->AGV_wheel_Angle[3]=	GIM_TOP_ECD4;
			if(chassis_move_control_loop->vz_set_channel<-10)
				chassis_move_control_loop->drct=-1;
			else
				chassis_move_control_loop->drct=1;
			break;
		case CHASSIS_RC_FOLLOW_GIMBAL://中间
			chassis_move_control_loop->vz_set_channel = 0;
			//由于atan2算出来的结果是弧度，需转换成角度 计算公式为 弧度 * 180.f / PI 最终得到角度值 
			atan_angle[0]=atan2(chassis_move_control_loop->vy_set_channel + chassis_move_control_loop->vz_set_channel*R*sin(-45.0) ,chassis_move_control_loop->vx_set_channel + chassis_move_control_loop->vz_set_channel*R*cos(-45.0))/PI*180.0;
			atan_angle[1]=atan2(chassis_move_control_loop->vy_set_channel + chassis_move_control_loop->vz_set_channel*R*sin(45.0),chassis_move_control_loop->vx_set_channel + chassis_move_control_loop->vz_set_channel*R*cos(45.0))/PI*180.0;
			atan_angle[2]=atan2(chassis_move_control_loop->vy_set_channel + chassis_move_control_loop->vz_set_channel*R*sin(135.0) ,chassis_move_control_loop->vx_set_channel + chassis_move_control_loop->vz_set_channel*R*cos(135.0))/PI*180.0;
			atan_angle[3]=atan2(chassis_move_control_loop->vy_set_channel + chassis_move_control_loop->vz_set_channel*R*sin(-135.0),chassis_move_control_loop->vx_set_channel + chassis_move_control_loop->vz_set_channel*R*cos(-135.0))/PI*180.0;
			// 将一圈360°转换成编码值的一圈0-8191 -> 角度 * 8191 / 360 最终转换为需要转动的角度对应的编码值，再加上偏置角度,最终得到目标编码值
			chassis_move_control_loop->AGV_wheel_Angle[0]=	Angle_Limit(GIM_Y_ECD_1 + (fp32)(atan_angle[0] * 22.75f),ECD_RANGE);
			chassis_move_control_loop->AGV_wheel_Angle[1]=	Angle_Limit(GIM_Y_ECD_2 + (fp32)(atan_angle[1] * 22.75f),ECD_RANGE);
			chassis_move_control_loop->AGV_wheel_Angle[2]=	Angle_Limit(GIM_Y_ECD_3 + (fp32)(atan_angle[2] * 22.75f),ECD_RANGE);
			chassis_move_control_loop->AGV_wheel_Angle[3]=	Angle_Limit(GIM_Y_ECD_4 + (fp32)(atan_angle[3] * 22.75f),ECD_RANGE);
			
			//优弧 劣弧 驱动电机转向判断
			Speed_Toggle(chassis_move_control_loop);
		
			if(chassis_move_control_loop->vx_channel == 0 && chassis_move_control_loop->vy_channel == 0 && chassis_move_control_loop->vz_channel == 0)//摇杆回中时，保持6020角度
			{
				for(int i=0;i<4;i++)//memcpy狗都不用
				chassis_move_control_loop->AGV_wheel_Angle[i] = AGV_wheel_Angle_last[i];
			}
			else
			{
				for(int i=0;i<4;i++)
				{
					AGV_wheel_Angle_last[i]=chassis_move_control_loop->AGV_wheel_Angle[i];
				}
			}
			for(int i=0;i<4;i++)//减少手抖误差 和回中误差
			{
				if(fabs(chassis_move_control_loop->AGV_wheel_Angle[i]-AGV_wheel_Angle_last[i])<22)//小于1度维持原值
				{
					chassis_move_control_loop->AGV_wheel_Angle[i]=AGV_wheel_Angle_last[i];
				}
			}				
			break;
		case CHASSIS_ZERO_FORCE:
				chassis_move_control_loop->vx_set_channel=0;chassis_move_control_loop->vy_set_channel=0;chassis_move_control_loop->vz_set_channel=0;
				break;
		}
}

//遥控器数值获取加死区限制
void chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector)
{
//   static int32_t vx_last_channel,vy_last_channel,vz_last_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[3], chassis_move_rc_to_vector->vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[2], chassis_move_rc_to_vector->vy_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[0], chassis_move_rc_to_vector->vz_channel, CHASSIS_RC_DEADLINE);
	
	
	///************************遗弃*****************/
//	//vx_channel范围：+-660
//	for(i=0;i<100;i++)
//	{
//    vx_last_channel -= vx_channel; 
//    vy_last_channel -= vy_channel;
//	vz_last_channel += vz_channel;
//	}
//	//最大660*100=66000
//	//把66000/8911=7.407
//	//把66000 扩展到0-8911
//	vx_set_channel=vx_last_channel/7.407;
//	vy_set_channel=vy_last_channel/7.407;
//	vz_set_channel=vz_last_channel/7.407;
	/*************************************************/
	
	//8911/660=13.5015 将速度扩大到额定转速
	chassis_move_rc_to_vector->vx_set_channel = -chassis_move_rc_to_vector->vx_channel*13.5015;
	chassis_move_rc_to_vector->vy_set_channel = -chassis_move_rc_to_vector->vy_channel*13.5015;
	chassis_move_rc_to_vector->vz_set_channel =  chassis_move_rc_to_vector->vz_channel*13.5015;
	
	//一阶低通滤波代替斜波作为底盘速度输入
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, chassis_move_rc_to_vector->vx_set_channel);
	chassis_move_rc_to_vector->vx_set_channel = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, chassis_move_rc_to_vector->vy_set_channel);
	chassis_move_rc_to_vector->vy_set_channel = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
	
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vz, chassis_move_rc_to_vector->vz_set_channel);
	chassis_move_rc_to_vector->vz_set_channel = chassis_move_rc_to_vector->chassis_cmd_slow_set_vz.out;

	chassis_move_rc_to_vector->vx_set = chassis_move_rc_to_vector->vx_set_channel/19*WHEEL_CIRCUMFERENCE;
	chassis_move_rc_to_vector->vy_set = chassis_move_rc_to_vector->vy_set_channel/19*WHEEL_CIRCUMFERENCE;
	chassis_move_rc_to_vector->wz_set = chassis_move_rc_to_vector->vy_set_channel/19*WHEEL_CIRCUMFERENCE/MOTOR_DISTANCE_TO_CENTER;
	
//	vx_last_channel=0;vy_last_channel=0;vz_last_channel=0; //对vx vy vz滤波后并调用进行清零处理
}

//pid计算
void Remote_pid_caculate(chassis_move_t *chassis_move_control_loop)
{
	for (int i = 0; i < 4; i++)
		{
        PID_calc(&chassis_move_control_loop->chassis_3508_speed_pid[i], 
					chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->rpm,chassis_move_control_loop->wheel_speed[i]);
		}
					//计算6020角度串速度环，速度环做内环，角度环做外环
		for(int i=0;i<4;i++)
		{
				//角度环
				PID_Calc_Ecd(&chassis_move_control_loop->chassis_6020_angle_pid[i],
					chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->ecd,chassis_move_control_loop->AGV_wheel_Angle[i],8191.0f);
				//速度环
				PID_calc(&chassis_move_control_loop->chassis_6020_speed_pid[i],
					chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->rpm,
					chassis_move_control_loop->chassis_6020_angle_pid[i].out);
		}
}

/*-----------------------------------------------------------------------------------------------公共函数---------------------------------------------------------------------------------------------------*/

//将电机转子转向内侧时 修正方向
fp32 Find_min_Angle(int16_t angle1,fp32 angle2)
{
	fp32 err;
    err = (fp32)angle1 - angle2;
    if(fabs(err) > 4096)
    {
        err = 8192 - fabs(err);
    }
    return err;
}


//等待6020角度转到位在转3508
void angle_judge(chassis_move_t *chassis_move_control_loop)
{
	if(fabs(chassis_move_control_loop->motor_chassis[4].chassis_motor_measure->ecd - chassis_move_control_loop->AGV_wheel_Angle[0]) < 1000)//当角度偏差为50时使能3508转动
		chassis_move_control_loop->angle_ready=1;
	else
		chassis_move_control_loop->angle_ready=0;
}


//判断优弧劣弧-->只转3508，不转6020，把6020角度控制在0-90度内，可以更好的转换运动方向
void Speed_Toggle(chassis_move_t *chassis_move_control_Speed)
{
		if(fabs(Find_min_Angle(chassis_move_control_Speed->motor_chassis[4].chassis_motor_measure->ecd,chassis_move_control_Speed->AGV_wheel_Angle[0]))>2048)
	{
		for(int i=0;i<4;i++)
		{
			chassis_move_control_Speed->AGV_wheel_Angle[i] += 4096;		
			chassis_move_control_Speed->AGV_wheel_Angle[i]=Angle_Limit(chassis_move_control_Speed->AGV_wheel_Angle[i],8192);
		}
			chassis_move_control_Speed->drct = -1;
	}
	else
			chassis_move_control_Speed->drct=1;
}


//清除PID
static void PID_CLEAR_ALL(chassis_move_t *chassis_move_data)
{
	for(uint8_t i=0;i<4;i++)
	{
		PID_clear(&chassis_move_data->chassis_6020_angle_pid[i]);
		PID_clear(&chassis_move_data->chassis_6020_speed_pid[i]);
		PID_clear(&chassis_move_data->chassis_3508_speed_pid[i]);
	}
}


//初始化uwb 的x和y数据 x：363  y：145   a板yaw为0度
//起始世界坐标为（0，-238） 
/** 														-----机器人跑点思路-----																	**/
/**	 	以地图原点为极坐标原点，由于场上竹子和地标位置已知，所以相当于各个点位的极角已知  			**/
/**	 			点位的极径已知，机器人的初始坐标在y轴的负方向238cm处，即初始极径为238cm  					**/
/**				重要的是把极坐标转换为直角坐标之中，这样得出x，y轴速度以及6020的转向角度					**/
/*******计算当前位置Y坐标与目标位置的Y坐标的差值******/

float Chassis_Me_To_Target_Y_Distance(chassis_move_t *chassis_move_data)
{
	return (chassis_move_data->target_Y-chassis_move_data->uwb_data.y);
}

/*******计算当前位置X坐标与目标位置的X坐标的差值******/
float Chassis_Me_To_Target_X_Distance(chassis_move_t *chassis_move_data)
{
	return (chassis_move_data->target_X-chassis_move_data->uwb_data.x);
}

//yaw轴初始化为零  零逆时针360度
fp32 yaw_error;
int yaw_calibrate_flag;
int16_t yaw_num;
extern int16_t final_yaw;
//计算所有信号以及yaw轴偏航角度
/*******计算当前位置角度坐标与目标位置的角度坐标的差值******/
int16_t Chassis_Me_To_Target_Angle(chassis_move_t *chassis_move_data)
{
	if(num==0)
	{
		yaw_num=56;return final_yaw-yaw_num;
	}
	if(num==16 || num==18)
	{
		return final_yaw+chassis_move_data->target_A+90;
	}
	if(num==9)
	{
		yaw_num=78;return final_yaw-yaw_num+90;
	}
	else
	{
	return final_yaw+chassis_move_data->target_A;
	}
}

/*********计算当前位置与目标位置的差值*************/
float Chassis_Me_To_Target_Distance(chassis_move_t *chassis_move_data)
{
	return sqrt(pow(Chassis_Me_To_Target_X_Distance(chassis_move_data),2)+pow(Chassis_Me_To_Target_Y_Distance(chassis_move_data),2));
}

/****************极坐标转直角坐标*****************/
/******* 车初始坐标 (238,-90°)----->(0,-238) ****/

void Polar_To_Stright(chassis_move_t *chassis_move_data,float plo_angle,float plo_length)
{
	int8_t left_right=0;  //相对左右标志位
	chassis_move_data->target_X =  (plo_length * cosf(plo_angle /180.f * PI));//X   
	chassis_move_data->target_Y =  (plo_length * sinf(plo_angle /180.f * PI));//Y  
	chassis_move_data->derta_X = Chassis_Me_To_Target_X_Distance(chassis_move_data);
	chassis_move_data->derta_Y = Chassis_Me_To_Target_Y_Distance(chassis_move_data);

	if(chassis_move_data->derta_Y > 0 && chassis_move_data->derta_X < 0) //相对车为原点 第二象限
	{
		left_right=-1; //左侧为负
		chassis_move_data->derta_Y = fabs(chassis_move_data->derta_Y);
		chassis_move_data->derta_X = fabs(chassis_move_data->derta_X);
		chassis_move_data->target_A =  atan2(chassis_move_data->derta_Y ,chassis_move_data->derta_X)/PI*180;//相对于车的角度
		chassis_move_data->target_A=left_right*(90.0f-chassis_move_data->target_A);
	}		
	else if(chassis_move_data->derta_Y > 0 && chassis_move_data->derta_X > 0)//第一象限
	{
		left_right=1; //右侧为正
		chassis_move_data->derta_Y = fabs(chassis_move_data->derta_Y);
		chassis_move_data->derta_X = fabs(chassis_move_data->derta_X);
		chassis_move_data->target_A =  atan2(chassis_move_data->derta_Y ,chassis_move_data->derta_X)/PI*180;//相对于车的角度
		chassis_move_data->target_A=left_right*(90.0f-chassis_move_data->target_A);
	}
	else if(chassis_move_data->derta_Y < 0 && chassis_move_data->derta_X < 0)//第三象限
	{
		left_right=-1; //左侧为负
		chassis_move_data->derta_Y = fabs(chassis_move_data->derta_Y);
		chassis_move_data->derta_X = fabs(chassis_move_data->derta_X);
		chassis_move_data->target_A =  atan2(chassis_move_data->derta_Y ,chassis_move_data->derta_X)/PI*180;//相对于车的角度
		chassis_move_data->target_A=left_right*(90.0f+chassis_move_data->target_A);
	}
	else if(chassis_move_data->derta_Y < 0 && chassis_move_data->derta_X > 0)//第四象限
	{
		left_right=1; //右侧为正
		chassis_move_data->derta_Y = fabs(chassis_move_data->derta_Y);
		chassis_move_data->derta_X = fabs(chassis_move_data->derta_X);
		chassis_move_data->target_A =  atan2(chassis_move_data->derta_Y ,chassis_move_data->derta_X)/PI*180;//相对于车的角度
		chassis_move_data->target_A=left_right*(90.0f+chassis_move_data->target_A);
	}
}

void stop_3508(chassis_move_t *chassis_move_control_loop)
{
	//急刹需要控制角度环输出为零，也就是让转速趋近于零，根据当前rpm控制而不是直接基于电流值为零
		chassis_move.wheel_speed[0]=0;
		chassis_move.wheel_speed[1]=0;
		chassis_move.wheel_speed[2]=0;
		chassis_move.wheel_speed[3]=0;
}

void Top_6020_enforce(chassis_move_t *chassis_move_control_loop)
{
		chassis_move_control_loop->AGV_wheel_Angle[0]=	GIM_TOP_ECD1;
		chassis_move_control_loop->AGV_wheel_Angle[1]=	GIM_TOP_ECD2;
		chassis_move_control_loop->AGV_wheel_Angle[2]=	GIM_TOP_ECD3;
		chassis_move_control_loop->AGV_wheel_Angle[3]=	GIM_TOP_ECD4;
}

void speed_3508_enforce(chassis_move_t *chassis_move_control_loop,float speed)
{
		chassis_move_control_loop->wheel_speed[0]=speed;
		chassis_move_control_loop->wheel_speed[1]=speed;
		chassis_move_control_loop->wheel_speed[2]=speed;
		chassis_move_control_loop->wheel_speed[3]=speed;
}

//top急停
void top_stop(chassis_move_t *chassis_move_control_loop)
{
	stop_3508(chassis_move_control_loop);
	Top_6020_enforce(chassis_move_control_loop);
}

void yaw_calibrate(chassis_move_t *chassis_move_control_loop)
{
	//计算yaw轴偏差
	yaw_error=Chassis_Me_To_Target_Angle(chassis_move_control_loop);
	if(yaw_error>180 && yaw_error<360)
	{
		yaw_error=-(360-yaw_error);
	}
	if(yaw_error<-180 && yaw_error>-360)
	{
		yaw_error=180+(yaw_error+180);
	}	

	//给定yaw正负3度的偏差
	if(fabs(yaw_error)>5)
	{		//6020转到小陀螺
			yaw_calibrate_flag=0;
			Top_6020_enforce(chassis_move_control_loop);
		if(num==5)
		{
			speed_3508_enforce(chassis_move_control_loop,-fabs(yaw_error)/yaw_error*150);
		}
		else
		{
			speed_3508_enforce(chassis_move_control_loop,-fabs(yaw_error)/yaw_error*100);
		}
	}
	else
	{
		yaw_calibrate_flag=1;
	}
				//pid计算
				Auto_pid_caculate(chassis_move_control_loop);
}

void position_judge(chassis_move_t *chassis_move_control_loop)
{
	chassis_move_control_loop->derta_length = Chassis_Me_To_Target_Distance(chassis_move_control_loop);
	if(num==0)
	{
		if(chassis_move_control_loop->derta_length < 20)position_judge_flag=1;
	}
		
	if(chassis_move_control_loop->derta_length < 45)
	{
		position_judge_flag=1;
	}
	else
	{
		//到达后进行夹取
		position_judge_flag=0;
	}
}

int16_t  last_ecd_3508;
int64_t  ecd_sum_3508=0;

void Control_3508(int16_t k)
{

			/*过零检测*/
		static int16_t ecd_error;
    ecd_error = chassis_move.motor_chassis[0].chassis_motor_measure->ecd -last_ecd_3508;
    ecd_error = ecd_error >  4095 ?   ecd_error - 8191 : ecd_error;
    ecd_error = ecd_error < -4095 ?   ecd_error + 8191 : ecd_error;
    ecd_sum_3508 -= ecd_error;
    last_ecd_3508 = chassis_move.motor_chassis[0].chassis_motor_measure->ecd;

		for(uint8_t i=0;i<4;i++)
	{
		PID_calc(&chassis_move.chassis_3508_angle_pid[i],-ecd_sum_3508,k*8191);//位置PID计算
		PID_calc(&chassis_move.chassis_3508_speed_pid[i],chassis_move.motor_chassis[i].chassis_motor_measure->rpm,chassis_move.chassis_3508_angle_pid[i].out);//速度PID计算
	}
	
	if(fabs(-ecd_sum_3508 - k*8191)<10000)
	{
		num++;
		chassis_move.lift_circle_number=ground;
	}

}

void Control_6020(int angle)
{

	angle = angle*22.75;
	chassis_move.AGV_wheel_Angle[0] = Angle_Limit(angle+GIM_AUTO_ECD_1,ECD_RANGE);
	chassis_move.AGV_wheel_Angle[1] = Angle_Limit(angle+GIM_AUTO_ECD_2,ECD_RANGE);
	chassis_move.AGV_wheel_Angle[2] = Angle_Limit(angle+GIM_AUTO_ECD_3,ECD_RANGE);
	chassis_move.AGV_wheel_Angle[3] = Angle_Limit(angle+GIM_AUTO_ECD_4,ECD_RANGE);

		for(uint8_t i=0;i<4;i++)
	{
			//角度环
			PID_Calc_Angle(&chassis_move.chassis_6020_angle_pid[i],
				chassis_move.motor_chassis[i+4].chassis_motor_measure->ecd,chassis_move.AGV_wheel_Angle[i]);
			//速度环
			PID_calc(&chassis_move.chassis_6020_speed_pid[i],
				chassis_move.motor_chassis[i+4].chassis_motor_measure->rpm,
				chassis_move.chassis_6020_angle_pid[i].out);
	}

}

void clear_step(chassis_move_t *chassis_move_data)
{
	for(int i=0;i<5;i++)
	{
		chassis_move_data->step[i]=0;
	}
}

#define Paw_1_catch 1
#define Paw_1_loose 2
#define Paw_2_catch 3
#define Paw_2_loose 4
#define Paw_3_catch 5
#define Paw_3_loose 6

#define average_min_speed 130 
#define change_min_speed 150
#define yaw_calibrate_speed 130
#define middle_get_speed 60
#define track_speed 180

//时间时间
uint64_t current_time;
uint64_t last_time;
uint8_t time_save;

void speed_limit(chassis_move_t *chassis_move_control_loop,int min_speed)
{
	for(int i=0;i<4;i++)
	{
			if(chassis_move_control_loop->wheel_speed[i]<0 && chassis_move_control_loop->wheel_speed[i]>-min_speed)
			{
				 chassis_move_control_loop->wheel_speed[i]=-min_speed;
			}
			if(chassis_move_control_loop->wheel_speed[i]>0 && chassis_move_control_loop->wheel_speed[i]<min_speed)
			{
				chassis_move_control_loop->wheel_speed[i]=min_speed;
			}
		}
}


//限制最低速度
void limit_speed(chassis_move_t *chassis_move_control_loop)
{
	if(!stop_flag)
	{
			if(num==5)speed_limit(chassis_move_control_loop,change_min_speed);
			if(num==0 || num==10 || num==9)speed_limit(chassis_move_control_loop,yaw_calibrate_speed);
			if(over)speed_limit(chassis_move_control_loop,track_speed);
			else speed_limit(chassis_move_control_loop,average_min_speed);
	for(int i=0;i<4;i++)
	{
			if(num==16 || num==18 || num ==15 || num==17)
			{
				if(chassis_move_control_loop->wheel_speed[i]<-70)
			{
				 chassis_move_control_loop->wheel_speed[i]=-70;
			}
			if(chassis_move_control_loop->wheel_speed[i]>70)
			{
				chassis_move_control_loop->wheel_speed[i]=70;
			}
			}
		}
	}
}

void put_first_task(chassis_move_t *chassis_move_data,int Paw_catch,int Paw_loose,int GD)
{
	//校准完成之后
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	current_time= HAL_GetTick();
	position_judge(chassis_move_data);
	if(chassis_move_data->GD_flag[GD] && !chassis_move_data->step[0])
	{
		if(position_judge_flag)
		{
		slow_down=1;
		}
	if(!time_save)
		{
		time_save=1;
		last_time=HAL_GetTick();
		}if(current_time-last_time>1500){chassis_move_data->lift_circle_number=put_bamboo;chassis_move_data->step[0]=1;stop_3508(chassis_move_data);stop_flag=1;}
	}
	if(chassis_move_data->step[0])
	{
		stop_3508(chassis_move_data);
		if(chassis_move_data->lift_over)
		{
		if(current_time-last_time>2000)chassis_move_data->Paw_flag=Paw_loose;
		if(current_time-last_time>2500){chassis_move_data->lift_circle_number=second;chassis_move_data->step[1]=1;stop_3508(chassis_move_data);}
		if(chassis_move_data->lift_over && chassis_move_data->step[1])
		{
			if(current_time-last_time>3000){chassis_move_data->Paw_flag=Paw_catch;chassis_move_data->step[2]=1;stop_3508(chassis_move_data);}
		}
		if(current_time-last_time>3700 && chassis_move_data->step[2])
		{
			chassis_move_data->lift_circle_number=ceiling;
			if(chassis_move_data->lift_over)
			{
				chassis_move_data->step[3]=1;
				if(current_time-last_time>4400)
				{
				num++; //完成
				clear_step(chassis_move_data);  //清除所有标志位
				yaw_calibrate_flag=0;
				time_save=0;
				stop_flag=0;
				slow_down=0;
				}
			}
		}
	}
	}
}

//放置第二个竹子之后
void put_second_task(chassis_move_t *chassis_move_data,int Paw_loose,int GD)
{
	//校准完成之后
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	current_time= HAL_GetTick();
	position_judge(chassis_move_data);
	if(chassis_move_data->GD_flag[GD] && !chassis_move_data->step[0])
	{
		if(position_judge_flag)
		{
		slow_down=1;
		}
		if(!time_save)
		{
		time_save=1;
		last_time=HAL_GetTick();
		}
	if(current_time-last_time>1500){stop_3508(chassis_move_data);chassis_move_data->lift_circle_number=put_bamboo;chassis_move_data->step[0]=1;stop_flag=1;}
	}
	
	if(chassis_move_data->step[0])
	{
		stop_3508(chassis_move_data);
		if(chassis_move_data->lift_over)
		{
		if(current_time-last_time>2200)chassis_move_data->Paw_flag=Paw_loose; chassis_move_data->step[1]=1;stop_3508(chassis_move_data);
		}
	}
	
	if(current_time-last_time>3000 && chassis_move_data->step[1])
	{
		num++; //完成
		clear_step(chassis_move_data);  //清除所有标志位
		yaw_calibrate_flag=0;
		time_save=0;
		stop_flag=0;
		slow_down=0;

	}
}

void num_0_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	
	else{
	//校准完成之后
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	position_judge(chassis_move_data);
		if(position_judge_flag)
		{
			stop_3508(chassis_move_data);
			stop_flag=1;
		}
	if(chassis_move.GD_flag[0] && !time_save){time_save=1;last_time = HAL_GetTick();}
	current_time= HAL_GetTick();
	if(chassis_move.GD_flag[0])
	{
	if(current_time-last_time > 1200){chassis_move_data->Paw_flag=Paw_2_catch;}
}
	if(chassis_move_data->Paw_flag==Paw_2_catch)
	{
	if(current_time-last_time > 1700){chassis_move_data->lift_circle_number=ceiling;chassis_move_data->step[0]=1;}
}
	if( chassis_move_data->step[0])
	{
		if(chassis_move_data->lift_over)
		{
				num++;
				clear_step(chassis_move_data);
				time_save=0;
				yaw_calibrate_flag=0;
				stop_flag=0;
		}
	}
}
}

void num_1_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	
	else{
	put_first_task(chassis_move_data,3,4,0);
}
}

void num_2_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	position_judge(chassis_move_data);
	if(position_judge_flag)
	{
		num++;
		stop_3508(chassis_move_data);
	}
}

void num_3_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	else{
	put_second_task(chassis_move_data,4,0);
}
}

void forward_task(chassis_move_t *chassis_move_data)
{
	Control_6020(180);
	Control_3508(-25);
	Auto_pid_caculate(chassis_move_data);
}

int8_t num_5_over;
void num_5_task(chassis_move_t *chassis_move_data)
{	

	chassis_move_data->lift_circle_number=ground;
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	else{
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	current_time=HAL_GetTick();
			position_judge(chassis_move_data);
		if(position_judge_flag)
		{
			stop_3508(chassis_move_data);
			stop_flag=1;
		}
	if(chassis_move_data->GD_flag[0])
	{
		if(!time_save)
		{
		time_save=1;
		last_time=HAL_GetTick();
		}
	}
	if(chassis_move_data->GD_flag[0] && !chassis_move_data->step[0])
	{
		if(current_time-last_time>700){
			chassis_move_data->Paw_flag=Paw_2_catch;
			chassis_move_data->step[0]=1;
		}
	}
	if(chassis_move_data->step[0] && current_time-last_time>1100)
	{
			chassis_move_data->lift_circle_number=ceiling;
			clear_step(chassis_move_data);  //清除所有标志位
			yaw_calibrate_flag=0;
			time_save=0;
			stop_flag=0;
			num_5_over=1;
		num++;
			
	}

}
}

void num_9_task(chassis_move_t *chassis_move_data)
{
	num=9;
	chassis_move_data->lift_circle_number=ground;
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	
	else{
	//校准完成之后
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	position_judge(chassis_move_data);
		if(position_judge_flag)
		{
			stop_3508(chassis_move_data);
			stop_flag=1;
		}
	if(chassis_move.GD_flag[1] && !time_save){time_save=1;last_time = HAL_GetTick();}
	current_time= HAL_GetTick();
	if(chassis_move.GD_flag[1])
	{
	if(current_time-last_time > 1000){chassis_move_data->Paw_flag=Paw_1_catch;}
}
	if(chassis_move_data->Paw_flag==Paw_1_catch)
	{
	if(current_time-last_time > 1500){chassis_move_data->step[0]=1;}
}
	if(chassis_move_data->step[0])
	{
		if(chassis_move_data->lift_over)
		{
				num++;
				clear_step(chassis_move_data);
				time_save=0;
				yaw_calibrate_flag=0;
				stop_flag=0;
		}
	}
}
}

void num_10_task(chassis_move_t *chassis_move_control_loop)
{
	chassis_move_control_loop->lift_circle_number=ground;
	Polar_To_Stright(chassis_move_control_loop,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次 校准完成后再次开始
	yaw_calibrate(chassis_move_control_loop);
	else{
		Auto_chassis_AGV_wheel_speed(chassis_move_control_loop);
		Auto_chassis_AGV_wheel_angle(chassis_move_control_loop);
		current_time=HAL_GetTick();
		if(chassis_move.GD_flag[0] && !time_save){time_save=1;last_time = HAL_GetTick();}
	current_time= HAL_GetTick();
	if(chassis_move.GD_flag[0])
	{
	if(current_time-last_time > 700){chassis_move_control_loop->Paw_flag=Paw_2_catch;chassis_move_control_loop->step[0]=1;}
}
		if(current_time-last_time > 1200&&chassis_move_control_loop->step[0]){
		chassis_move_control_loop->lift_circle_number=ceiling;
		num++; //完成
		clear_step(chassis_move_control_loop);  //清除所有标志位
		yaw_calibrate_flag=0;
		time_save=0;
		stop_flag=0;
	}
}
}

void num_16_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	{yaw_calibrate(chassis_move_data); chassis_move_data->lift_circle_number=ceiling;}
	else{
		put_first_task(chassis_move_data,1,2,1);
	}
}

void num_18_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	else
	{
		put_second_task(chassis_move_data,2,1);
}
}

int track_angle_start=106;

void track(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,track_angle_start,120);  //targetA  
	position_judge(chassis_move_data);
	current_time=HAL_GetTick();
	if(position_judge_flag)
	{
		if(!time_save)
		{
		time_save=1;
		last_time=HAL_GetTick();
		}
		if(current_time-last_time<800)
		{
			stop_3508(chassis_move_data);
			stop_flag=1;
		}
		if(current_time-last_time>800)
		{
			stop_flag=0;
			Top_6020_enforce(chassis_move_data);
		}
		if(current_time-last_time>6000)
		{
			track_angle_start-=40;
			if(track_angle_start<-180){track_angle_start=179;}
			time_save=0;
		}
	}
	else
	{
		Auto_chassis_AGV_wheel_speed(chassis_move_data);
		Auto_chassis_AGV_wheel_angle(chassis_move_data);
	}
}

//自动模式下，取8个电机的速度并且进行PID
static void chassis_automatic_control_loop(chassis_move_t *chassis_move_control_loop)
{
	if(!over)
	{
	switch(chassis_move_control_loop->chassis_RC->rc.s[0])
	{
		case 3 :
		switch(num)
	{
		case 0:num_0_task(chassis_move_control_loop);break;  //取地上竹子
		case 1:num_1_task(chassis_move_control_loop);break;	 //放first
		case 2:num_2_task(chassis_move_control_loop);break;  //转点
		case 3:num_3_task(chassis_move_control_loop);break;  //放second
		case 4:num_2_task(chassis_move_control_loop);break;  //前进
		case 5:num_5_task(chassis_move_control_loop);break;  //取竹子
		case 6:num_1_task(chassis_move_control_loop);break;  //放first
		case 7:num_2_task(chassis_move_control_loop);break;  //转点
		case 8:num_3_task(chassis_move_control_loop);break;  //放second
	}
	break;
		case 2:
			switch(num)
	{		
		case 0:num_9_task(chassis_move_control_loop);break;  //重置后取地上竹子
		case 9:num_9_task(chassis_move_control_loop);break;  //重置后取地上竹子
		case 10:num_10_task(chassis_move_control_loop);break; //取竹子
		case 11:num_2_task(chassis_move_control_loop);break;  //转点
		case 12:num_1_task(chassis_move_control_loop);break; //放first
		case 13:num_2_task(chassis_move_control_loop);break;  //转点
		case 14:num_3_task(chassis_move_control_loop);break;  //放second
		case 15:chassis_move_control_loop->lift_circle_number=put_bamboo;num_2_task(chassis_move_control_loop);break;  //转点
		case 16:num_16_task(chassis_move_control_loop);break; //放first
		case 17:chassis_move_control_loop->lift_circle_number=ceiling;num_2_task(chassis_move_control_loop);break; //转点
		case 18:num_18_task(chassis_move_control_loop);break; //放second
	}
	break;
	}
}
	if(over)
	{
		track(chassis_move_control_loop);
	}
	Auto_pid_caculate(chassis_move_control_loop);
}



