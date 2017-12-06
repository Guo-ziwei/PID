#ifndef __UART_H
#define __UART_H

#include <iom64v.h>
#include <macros.h>
#include "initial.h"


unsigned char send_flag = 0;           // 开始发送标志
unsigned char receive_flag = 0;        // 接收完成标志

unsigned char SendData[9] = {0};
unsigned char ReceivedData[13] = {0};
float ka,kb,kc;
unsigned char control_flag = 1;
extern unsigned char motor_dir;
extern unsigned int speed;
extern unsigned long distance;
extern unsigned long TargetPosition;

extern struct PID speedPID;
extern void PIDInit ( struct PID *p );
extern void PID_reload ( struct PID *p, unsigned int ref, unsigned char  a, float b, float c );


void uart1_init(void)
{
 UCSR1B = 0x00; //disable while setting baud rate
 UCSR1A = 0x00;
 UCSR1C = 0x06;
 UBRR1L = 0x47; //set baud rate lo
 UBRR1H = 0x00; //set baud rate hi 9600
 UCSR1B = 0x98;
}

// 说明：接收控制指令，进行数据处理
void receive_control_command(void)
{
 unsigned int ref;
 //unsigned char ka,kb,kc;
 extern unsigned char DC_State;
 
 motor_dir = ReceivedData[1];
 control_flag = ReceivedData[11];
 
 
 if(motor_dir == 0)
 {
    LedLimit1On();
	LedLimit2Off();	
 }
 else if ( motor_dir == 1 ) 
 {
   LedLimit2On();
   LedLimit1Off();
 }
 
 	ref = (ReceivedData[2]<<8) + ReceivedData[3];	  //给定速度大小
	if  (ref > 0x05 ) 
   	{ DC_State = RUN;
	  LedRunOn();
	}
 	else 
	{ DC_State = STOP;
	  LedRunOff();
	}
	ref = ref;//*27/28;
	TargetPosition = (ReceivedData[4]<<8) + ReceivedData[5];  //目标位置
	TargetPosition = TargetPosition*50;
    distance=0;  //位移清0
	
	ka = ReceivedData[6];			//参数Ka = Kp
	kb = ReceivedData[7] + (float)(ReceivedData[8]/10);			//参数Kb =  T / Ti 
	kc = ReceivedData[9] + (float)(ReceivedData[10]/10);			//参数Kc = ( Td / T )
	PID_reload ( &speedPID, ref, ka, kb, kc );  // 更新PID参数
	receive_flag=0;  //清接收完成标志
 }
 
#pragma interrupt_handler uart1_rx_isr:31
void uart1_rx_isr(void)
{
 //uart has received a character in UDR
 static unsigned char rece_num=0;
 unsigned char data;
 
 data=UDR1;

 if(( 0xAA == data )&&( 0 == rece_num ) || rece_num > 0 )
     {
	 	 ReceivedData[ rece_num ] = data;		
     	 rece_num++;
     }  

 if ( 13 == rece_num )//
      {
      	 receive_control_command();
	     rece_num = 0; 		// 
	     receive_flag = 1;	// 
       }
}

void data_update( unsigned int PWM_width )
{
 CLI();
 
 SendData[0] = 0xaa;
 
 if ( motor_dir == 0 )      SendData[1] = 0;
 else if ( motor_dir == 1 ) SendData[1] = 1;
 
 SendData[2] = PWM_width>>8; //pwm脉宽高位
 SendData[3] = PWM_width;//pwm脉宽低位
 
 SendData[4] = speed>>8;
 SendData[5] = speed;
 
 //distance = distance/500;
 SendData[6] = distance>>8;
 SendData[7] = distance; 

 SendData[8] = 0x55;
 
 send_flag = 1;
 
 SEI();
}

void send_control_result(void)
{
 	unsigned char i;
	for( i=0; i<=8; i++)
  	{
   		while (!(UCSR1A&(1<<UDRE1)));		// 等待发送寄存器为空
   		UDR1=SendData[ i ];				// 为空时，发送下一字节
  	} 	
  	send_flag = 0;				// 清开始发送标志
}

#endif
