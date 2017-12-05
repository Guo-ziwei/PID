#include <iom64v.h>
#include <macros.h>
#include "DCmotor.h"
#include "initial.h"
#include "uart.h"
#include "pid.h"

unsigned int time = 0;

void port_init(void)
{
 PORTA = 0x3F;
 DDRA  = 0x3F;
 PORTB = 0x00;
 DDRB  = 0xF0;
 PORTC = 0x00; //m103 output only
 DDRC  = 0x00;
 PORTD = 0x0C;
 DDRD  = 0x0C;
 PORTE = 0x00;
 DDRE  = 0x00;
 PORTF = 0x00;
 DDRF  = 0x00;
 PORTG = 0x00;
 DDRG  = 0x00;
}

void init_devices(void)
{
 //stop errant interrupts until set up
 CLI(); //disable all interrupts
 XDIV  = 0x00; //xtal divider
 XMCRA = 0x00; //external memory
 port_init();
 timer0_init();//产生20ms周期
 timer1_init();//产生PWM波
 timer3_init();//记录脉冲数量
 uart1_init();

 MCUCR = 0x00;
 EICRA = 0x0A; //extended ext ints
 EICRB = 0x0F; //extended ext ints
 EIMSK = 0x33; //外部中断屏蔽寄存器
 TIMSK = 0x05; //timer interrupt sources
 ETIMSK = 0x00; //extended timer interrupt sources
 SEI(); //re-enable interrupts
 //all peripherals are now initialized
}

void main(void)
{	
	init_devices();
	PIDInit( &speedPID );
	 while(1)
 {
 
 	if ( send_flag == 1 )
 	{
   		send_control_result();
 	}
 }
}
