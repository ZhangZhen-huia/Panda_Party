#pragma once 

#include "Can_recive.h"
#include "pid.h"

typedef enum 
{
	ground = 0,
	put_bamboo = 12,
	ceiling = 37,
	second = 22,
}circle_number_lift_flag;

void lift_motor_init(void);


