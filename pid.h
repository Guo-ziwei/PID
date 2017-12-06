#ifndef __PID_H
#define __PID_H

#define KA_INIT_VALUE  0x01
#define KB_INIT_VALUE  0x01
#define KC_INIT_VALUE  0x01

extern unsigned char control_flag;
struct PID 	// 定义闭环控制数据结构体
{
	unsigned int Reference;		// 参考值
	unsigned int FeedBack;		// 反馈值
	
	// 下边两行为PID算法实现时需要加的两个变量
	int ek_1; 		// 前一次误差, Reference_1 - PreFeedBack_1
	int ek_2;	    // 前两次误差, Reference_2 - PreFeedBack_2
	
	unsigned char Ka;		// Ka = Kp
	float  Kb;		// Kb = ( T / Ti )
	float  Kc;		// Kc =  ( Td / T )
	
	unsigned int ControlValue;		//电机控制输出值
		
} speedPID;


void PIDInit ( struct PID *p )
{	
	p->Reference = 0;		
	p->FeedBack = 0;		
	
	// 下边两行为PID算法实现时需要加的两个变量的初始化	
	p->ek_1 = 0;
	p->ek_2 = 0;
	
	p->Ka = 1;
	p->Kb = 1;
	p->Kc = 1;
	
	p->ControlValue = 0L;
}

void PID_reload ( struct PID *p, unsigned int ref, unsigned char a, float b, float c )
{
    p->Reference = ref;
	p->Ka = a;
	p->Kb = b;
	p->Kc = c;
	//p->ControlValue= 0;
}

void PID_feedback ( struct PID *p, unsigned int fb )
{
 	p->FeedBack = fb;
}

unsigned int speed_PID_Calc( struct PID *p ) //输出pwm脉宽在0到551之间，551=0x0227
{
 	if(control_flag == 0)
	{
 		 int error;
		 int wave_width;
		 error = p->Reference - p->FeedBack;
		 wave_width = p->Ka*(error - p->ek_1) + p->Kb*error + p->Kc*(error - 2*p->ek_1 + p->ek_2);
		 p->ek_2 = p->ek_1;
		 p->ek_1 = error;
		 p->ControlValue =  p->ControlValue + wave_width;
		 if(p->ControlValue > 0x0226)
		    p->ControlValue = 0x0226;
		 return (p->ControlValue);
	}
	else
        return 	( p->Reference );  // 开环返回值
}


#endif
