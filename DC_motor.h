#ifndef __DCmotor_H
#define __DCmotor_H

#include <iom64v.h>
#include <macros.h>
#include "initial.h"
#include "pid.h"

unsigned char DC_State = STOP;//电机运转状态
unsigned char motor_dir = FORWARD; //方向指令标志

unsigned int speed = 0;
unsigned long TargetPosition = 0L;
unsigned long distance = 0L;//编码器脉冲数累计值，表示当前位移
unsigned char dir_sign = 0; //编码器检测的脉冲方向，0为正向，1为反向

extern void data_update( unsigned int PWM_width );

void delay_ms(int ms)
{
  int i,j;
  for ( i=0; i<ms; i++ )
    for ( j=0; j<1579; j++ );
}

void DC_forward(void)
{
 PORTB |= 0B00010000;
}

void DC_Backward(void) 
{
   PORTB &= 0B11101111;
}

void DC_Stop(void) 
{
   PORTB &= 0B11101111;
   OCR1A = 0x0000;
   DC_State = STOP;
}

void timer0_init(void)
{
 TCCR0 = 0x00; //stop
 ASSR  = 0x00; //set async mode
 TCNT0 = 0x27; //set count
 OCR0  = 0xD7;
 TCCR0 = 0x07; //start timer
}

#pragma interrupt_handler timer0_ovf_isr:17
void timer0_ovf_isr(void)
{
 unsigned int PWM_width = 0; 
 TCNT0 = 0x27; //reload counter value
 speed = TCNT3;
 TCNT3 = 0; 
 PID_feedback ( &speedPID, speed );  // 更新速度反馈 
 PWM_width = speed_PID_Calc( &speedPID );

 if (DC_State == RUN )  //判断电机状态，并按相应状态运行电机
 {
  if (dir_sign==0) distance += speed;
  else if (dir_sign==1) distance += speed;

   if(motor_dir == FORWARD) 
   {    
       DC_forward();
	   OCR1A = 0x0226 - PWM_width;
   }
   else if ( motor_dir == BACKWARD ) 
   {    
	   DC_Backward();  
	   OCR1A = PWM_width;
   }
 }
 else DC_Stop();   
 if(distance >= TargetPosition) DC_Stop();
 data_update(PWM_width);
// 
}


//TIMER1 initialize - prescale:1
// WGM: 14) PWM fast, TOP=ICRn
// desired value: 20KHz
// actual value: 20.034KHz (0.2%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0x00; //setup
 TCNT1L = 0x00;
 OCR1AH = 0x02;
 OCR1AL = 0x27;
 OCR1BH = 0x02;
 OCR1BL = 0x27;
 OCR1CH = 0x02;
 OCR1CL = 0x27;
 ICR1H  = 0x02;
 ICR1L  = 0x27;
 TCCR1A = 0x82;//82
 TCCR1B = 0x19; //start Timer
}

#pragma interrupt_handler timer1_ovf_isr:15
void timer1_ovf_isr(void)
{
 //TIMER1 has overflowed

 TCNT1H = 0x00; //reload counter high value
 TCNT1L = 0x00; //reload counter low value
}

void timer3_init(void)
{
 TCCR3B = 0x00; //stop
 TCNT3H = 0x00 /*INVALID SETTING*/; //setup
 TCNT3L = 0x00 /*INVALID SETTING*/;
 OCR3AH = 0x00 /*INVALID SETTING*/;
 OCR3AL = 0x00 /*INVALID SETTING*/;
 OCR3BH = 0x00 /*INVALID SETTING*/;
 OCR3BL = 0x00 /*INVALID SETTING*/;
 OCR3CH = 0x00 /*INVALID SETTING*/;
 OCR3CL = 0x00 /*INVALID SETTING*/;
 ICR3H  = 0x00 /*INVALID SETTING*/;
 ICR3L  = 0x00 /*INVALID SETTING*/;
 TCCR3A = 0x00;
 TCCR3B = 0x07; //start Timer
}


// 编码器正转检测
#pragma interrupt_handler int0_isr:2
void int0_isr(void)
{
 //external interupt on INT0
  LedZeroOff();
  dir_sign = 0;
}

// 编码器反转检测
#pragma interrupt_handler int1_isr:3
void int1_isr(void)
{
 //external interupt on INT1
  LedZeroOn();
  dir_sign = 1;
}

#pragma interrupt_handler int4_isr:6
void int4_isr(void)
{
 //external interupt on INT4
 delay_ms(50);
 
 if ( DC_State == RUN ) {
 	 if ( isButtonFWDown() ) 
	  {
	     dir_sign = FORWARD;
		 LedLimit1On();
		 LedLimit2Off();	     
	  }
	 else if ( isButtonBWDown() ) 
	  {
	     dir_sign = BACKWARD;
		 LedLimit2On();
		 LedLimit1Off();
	  }
  }
}

// 电机开始/停止指令，零点检测
#pragma interrupt_handler int5_isr:7
void int5_isr(void)
{
 //external interupt on INT5
 delay_ms(50);
 if ( isButtonSTOPDown() ) {
     if ( DC_State == RUN ) 
	    {
		  DC_State = STOP;
		  LedRunOff();
		  DC_Stop();
		}
	 else if ( DC_State == STOP )
	    {
		  DC_State = RUN;
		  LedRunOn();
		}
	}
 else if ( isButtonZERODown() )
 {
 }	
 
}


#endif
