#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "utils/uartstdio.h"
#define pi 3.14159265

uint32_t g_NumOfPulses_1,g_NumOfPulses_2,g_NumOfPulses_3,g_NumOfPulses_4,g_NumOfPulses_5,g_NumOfPulses_6;
uint32_t d_AngleRotating_1,d_AngleRotating_2,d_AngleRotating_3,d_AngleRotating_4,d_AngleRotating_5,d_AngleRotating_6;
uint32_t d_AngleTemp_1,d_AngleTemp_2;
uint32_t n_EventCounter,n_PhaseCounter,first_time;
float l2,l3,l4;
uint8_t d_AngleCalculated_1,d_AngleCalculated_2,d_AngleCalculated_3,d_AngleCalculated_4;
uint32_t Reserved_x_AxisCoordinates,Reserved_y_AxisCoordinates;
int d_ObjectWidth[4],d_ClassifiedType[4];
uint8_t n_Delay_i;
int b_ProcessPhase,Reserved_ObjectToRobotArmDistance,Reserved_ObjectToRobotArmTime,Reserved_Error;
static char Buff[100];
char d_UARTData[100],d_UARTDataToProcess[100];
int d_PreviousAngle_1,d_PreviousAngle_2,d_PreviousAngle_3,d_PreviousAngle_4,d_PreviousAngle_5,d_PreviousAngle_6;

void Timer_ISR_1(void)
{
    TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
    if(g_NumOfPulses_1 == SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_1/1800))
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,0);
        g_NumOfPulses_1 = SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_1/1800);
    }
    else if(g_NumOfPulses_1 == SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_1/1800))
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,GPIO_PIN_7);
        g_NumOfPulses_1 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_1/1800);
    }
    else
    {

    }
    TimerLoadSet(TIMER0_BASE,TIMER_A,g_NumOfPulses_1);


}

void Timer_ISR_2(void)
{
    TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
    if(g_NumOfPulses_2 ==SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_2/1800))
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0);
        g_NumOfPulses_2 = SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_2/1800);
    }
    else if(g_NumOfPulses_2 ==SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_2/1800))
    {
        GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);
        g_NumOfPulses_2 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_2/1800);
    }
    else
    {

    }
    TimerLoadSet(TIMER1_BASE,TIMER_A,g_NumOfPulses_2);

}

void Timer_ISR_3(void)
{
    TimerIntClear(TIMER2_BASE,TIMER_TIMA_TIMEOUT);
    if(g_NumOfPulses_3 ==SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_3/1800))
    {
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4,0);
        g_NumOfPulses_3 = SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_3/1800);
    }
    else if(g_NumOfPulses_3 ==SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_3/1800))
    {
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_PIN_4);
        g_NumOfPulses_3 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_3/1800);
    }
    else
    {

    }
    TimerLoadSet(TIMER2_BASE,TIMER_A,g_NumOfPulses_3);

}

void Timer_ISR_4(void)
{
    TimerIntClear(TIMER3_BASE,TIMER_TIMA_TIMEOUT);
    if(g_NumOfPulses_4 ==SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_4/1800))
    {
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,0);
        g_NumOfPulses_4 = SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_4/1800);
    }
    else if(g_NumOfPulses_4 ==SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_4/1800))
    {
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,GPIO_PIN_5);
        g_NumOfPulses_4 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_4/1800);
    }
    else
    {

    }
    TimerLoadSet(TIMER3_BASE,TIMER_A,g_NumOfPulses_4);

}
void Timer_ISR_5(void)
{
    TimerIntClear(TIMER4_BASE,TIMER_TIMA_TIMEOUT);
    if(g_NumOfPulses_5 ==SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_5/1800))
    {
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4,0);
        g_NumOfPulses_5 = SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_5/1800);
    }
    else if(g_NumOfPulses_5 ==SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_5/1800))
    {
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4,GPIO_PIN_4);
        g_NumOfPulses_5 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_5/1800);
    }
    else
    {

    }
    TimerLoadSet(TIMER4_BASE,TIMER_A,g_NumOfPulses_5);

}

void Timer_ISR_6(void)
{
    TimerIntClear(TIMER5_BASE,TIMER_TIMA_TIMEOUT);
    if(g_NumOfPulses_6 ==SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_6/1800))
    {
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1,0);
        g_NumOfPulses_6 = SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_6/1800);
    }
    else if(g_NumOfPulses_6 ==SysCtlClockGet()/1000000*(19500-19000*d_AngleRotating_6/1800))
    {
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1,GPIO_PIN_1);
        g_NumOfPulses_6 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_6/1800);
    }
    else
    {

    }
    TimerLoadSet(TIMER5_BASE,TIMER_A,g_NumOfPulses_6);

}

void Timer_ISR_7(void)
{
    TimerIntClear(WTIMER0_BASE,TIMER_TIMA_TIMEOUT);
	n_Delay_i = 1;
    TimerDisable(WTIMER0_BASE,TIMER_A);
}

void DelayFunction(int a)
{
	int n_Delay;
	n_Delay = 0;
	n_Delay_i = 0;
    TimerLoadSet(WTIMER0_BASE,TIMER_A,SysCtlClockGet()/1000*a-1);
    TimerEnable(WTIMER0_BASE,TIMER_A);
	while(n_Delay == 0)
	{
		n_Delay = n_Delay_i;
	}
}

void StartStep(void);
void PreGrabStep(void);
void GrabStep(void);
void MiddleStep(void);
void PreReleaseStep(void);
void ReleaseStep(void);


void Timer_ISR_8_Reserved(void)
{
    TimerIntClear(WTIMER2_BASE,TIMER_TIMA_TIMEOUT);
    if(n_PhaseCounter <= 3)
    {
        n_EventCounter = n_EventCounter + 1;
        if(d_ClassifiedType[n_PhaseCounter] != 0)
        {
			switch(n_EventCounter)
			{
				case 20:
					PreGrabStep();
					break;
				case 40:
					GrabStep();
					break;
				case 60:
					MiddleStep();
					break;
				case 80:
					PreReleaseStep();
					break;
				case 100:
					ReleaseStep();
					break;
				case 120:
					StartStep();
					n_EventCounter = 0;
					n_PhaseCounter++;
					break;
			}
        }
        else
        {
        	n_EventCounter = 0;
			n_PhaseCounter++;
        }
    }

}

void Timer_ISR_8(void)
{
    TimerIntClear(WTIMER2_BASE,TIMER_TIMA_TIMEOUT);
    n_EventCounter = 1;
    TimerDisable(WTIMER2_BASE,TIMER_A);

}

void ClassificationProcess(void)
{
	for(n_PhaseCounter = 0;n_PhaseCounter <= 3;n_PhaseCounter++)
	{
		if(d_ClassifiedType[n_PhaseCounter] != 0)
		{
			PreGrabStep();
			GrabStep();
			MiddleStep();
			PreReleaseStep();
			ReleaseStep();
			MiddleStep();
		}
		if(n_PhaseCounter==3 && (d_ClassifiedType[0] != 0 ||d_ClassifiedType[1] != 0 || d_ClassifiedType[2] != 0 || d_ClassifiedType[3] != 0  ))
		{
			StartStep();
		}
	}

	//if(d_ClassifiedType[0]+d_ClassifiedType[3]+d_ClassifiedType[2]+d_ClassifiedType[3]==0)
	//{
		DelayFunction(500);
	//}
    UARTCharPut(UART0_BASE,'E');
	n_EventCounter = 0;
}




void ConfigTimer_1(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC); // config timer
    TimerIntRegister(TIMER0_BASE,TIMER_A,&Timer_ISR_1);  // dang ki la co ngat timer, ham Timer_ISR la ham ngat
    IntEnable(INT_TIMER0A);  // cho phep ngat timer 0
    TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);  //


}

void ConfigTimer_2(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC);
    TimerIntRegister(TIMER1_BASE,TIMER_A,&Timer_ISR_2);  // dang ki la co ngat timer, ham Timer_ISR la ham ngat
    IntEnable(INT_TIMER1A);  // cho phep ngat timer 0
    TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);  //

   // TimerEnable(TIMER1_BASE,TIMER_A); //bat timer len
}
void ConfigTimer_3(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC); // config timer
    TimerIntRegister(TIMER2_BASE,TIMER_A,&Timer_ISR_3);  // dang ki la co ngat timer, ham Timer_ISR la ham ngat
    IntEnable(INT_TIMER2A);  // cho phep ngat timer 0
    TimerIntEnable(TIMER2_BASE,TIMER_TIMA_TIMEOUT);


}

void ConfigTimer_4(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerConfigure(TIMER3_BASE,TIMER_CFG_PERIODIC);
    TimerIntRegister(TIMER3_BASE,TIMER_A,&Timer_ISR_4);  // dang ki la co ngat timer, ham Timer_ISR la ham ngat
    IntEnable(INT_TIMER3A);
    TimerIntEnable(TIMER3_BASE,TIMER_TIMA_TIMEOUT);


}
 void ConfigTimer_5(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    TimerConfigure(TIMER4_BASE,TIMER_CFG_PERIODIC); // config timer
    TimerIntRegister(TIMER4_BASE,TIMER_A,&Timer_ISR_5);  // dang ki la co ngat timer, ham Timer_ISR la ham ngat
    IntEnable(INT_TIMER4A);  // cho phep ngat timer 0
    TimerIntEnable(TIMER4_BASE,TIMER_TIMA_TIMEOUT);  //


}
void ConfigTimer_6(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    TimerConfigure(TIMER5_BASE,TIMER_CFG_PERIODIC); // config timer
    TimerIntRegister(TIMER5_BASE,TIMER_A,&Timer_ISR_6);  // dang ki la co ngat timer, ham Timer_ISR la ham ngat
    IntEnable(INT_TIMER5A);  // cho phep ngat timer 0
    TimerIntEnable(TIMER5_BASE,TIMER_TIMA_TIMEOUT);  //


}

void ConfigTimer_7(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    TimerConfigure(WTIMER0_BASE,TIMER_CFG_PERIODIC); // config timer
    TimerIntRegister(WTIMER0_BASE,TIMER_A,&Timer_ISR_7);  // dang ki la co ngat timer, ham Timer_ISR la ham ngat
    IntEnable(INT_WTIMER0A);  // cho phep ngat timer 0
    TimerIntEnable(WTIMER0_BASE,TIMER_TIMA_TIMEOUT);  //
}

void ConfigTimer_8(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
    TimerConfigure(WTIMER2_BASE,TIMER_CFG_PERIODIC); // config timer
    TimerIntRegister(WTIMER2_BASE,TIMER_A,&Timer_ISR_8);  // dang ki la co ngat timer, ham Timer_ISR la ham ngat
    IntEnable(INT_WTIMER2A);  // cho phep ngat timer 0
    TimerIntEnable(WTIMER2_BASE,TIMER_TIMA_TIMEOUT);  //


}

void ConfigServoMotor_1(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,0);

}
void ConfigServoMotor_2(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0);

}
void ConfigServoMotor_3(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4,0);

}

void ConfigServoMotor_4(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,0);

}
void ConfigServoMotor_5(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4,0);

}



void ConfigServoMotor_6(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1,0);

}


void Input_ISR(void)
{
    GPIOIntClear(GPIO_PORTA_BASE,GPIOIntStatus(GPIO_PORTA_BASE,true));
}

void ConfigSwitch(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTA_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTA_BASE,&Input_ISR);
    GPIOIntEnable(GPIO_PORTA_BASE,GPIO_INT_PIN_4);
    IntEnable(INT_GPIOA);
}



void RotateServoMotor_1(uint32_t a)
{
    d_AngleRotating_1 = a;
    TimerDisable(TIMER0_BASE, TIMER_A);
    g_NumOfPulses_1 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_1/1800);
    TimerLoadSet(TIMER0_BASE,TIMER_A,g_NumOfPulses_1);
    TimerEnable(TIMER0_BASE,TIMER_A);

    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,GPIO_PIN_7);

}

void AdvancedRotateServoMotor_1(uint32_t a)
{
	int n_RotateStep_i,d_DeltaAngle_1;
	d_DeltaAngle_1 = a - d_PreviousAngle_1;
	if(d_DeltaAngle_1 >= 0)
	{
		if(d_DeltaAngle_1 >= 10)
		{
			for(n_RotateStep_i = 1; n_RotateStep_i <= d_DeltaAngle_1; n_RotateStep_i++)
			{
				RotateServoMotor_1(d_PreviousAngle_1 + n_RotateStep_i);
				if((1 <= n_RotateStep_i) && (n_RotateStep_i < d_DeltaAngle_1*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_1*15/100 <= n_RotateStep_i) && (n_RotateStep_i < d_DeltaAngle_1*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_1*30/100 <= n_RotateStep_i) && (n_RotateStep_i < d_DeltaAngle_1*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_1*70/100 <= n_RotateStep_i) && (n_RotateStep_i < d_DeltaAngle_1*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(n_RotateStep_i = 0; n_RotateStep_i <= d_DeltaAngle_1; n_RotateStep_i++)
			{
				RotateServoMotor_1(d_PreviousAngle_1 + n_RotateStep_i);
				DelayFunction(8);
			}
		}
	}
	else
	{
		if(d_DeltaAngle_1 <= -10)
		{
			for(n_RotateStep_i = -1; n_RotateStep_i >= d_DeltaAngle_1; n_RotateStep_i--)
			{
				RotateServoMotor_1(d_PreviousAngle_1 + n_RotateStep_i);
				if((-1 >= n_RotateStep_i) && (n_RotateStep_i > d_DeltaAngle_1*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_1*15/100 >= n_RotateStep_i) && (n_RotateStep_i > d_DeltaAngle_1*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_1*30/100 >= n_RotateStep_i) && (n_RotateStep_i > d_DeltaAngle_1*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_1*70/100 >= n_RotateStep_i) && (n_RotateStep_i > d_DeltaAngle_1*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(n_RotateStep_i = 0; n_RotateStep_i >= d_DeltaAngle_1; n_RotateStep_i--)
			{
				RotateServoMotor_1(d_PreviousAngle_1 + n_RotateStep_i);
				DelayFunction(8);
			}
		}
	}
	d_PreviousAngle_1 = a;
}


void RotateServoMotor_2(uint32_t a)
{
    d_AngleRotating_2 = a;
    TimerDisable(TIMER1_BASE, TIMER_A);
    g_NumOfPulses_2 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_2/1800);
    TimerLoadSet(TIMER1_BASE,TIMER_A,g_NumOfPulses_2);
    TimerEnable(TIMER1_BASE,TIMER_A);

    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);
}

void AdvancedRotateServoMotor_2(uint32_t a)
{
	int i,d_DeltaAngle_2;
	d_DeltaAngle_2 = a - d_PreviousAngle_2;
	if(d_DeltaAngle_2 >= 0)
	{
		if(d_DeltaAngle_2 >= 10)
		{
			for(i = 1; i <= d_DeltaAngle_2; i++)
			{
				RotateServoMotor_2(d_PreviousAngle_2+i);
				if((1 <= i) && (i < d_DeltaAngle_2*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_2*15/100 <= i) && (i < d_DeltaAngle_2*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_2*30/100 <= i) && (i < d_DeltaAngle_2*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_2*70/100 <= i) && (i < d_DeltaAngle_2*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i <= d_DeltaAngle_2; i++)
			{
				RotateServoMotor_2(d_PreviousAngle_2+i);
				DelayFunction(8);
			}
		}
	}
	else
	{
		if(d_DeltaAngle_2 <= -10)
		{
			for(i = -1; i >= d_DeltaAngle_2; i--)
			{
				RotateServoMotor_2(d_PreviousAngle_2+i);
				if((-1 >= i) && (i > d_DeltaAngle_2*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_2*15/100 >= i) && (i > d_DeltaAngle_2*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_2*30/100 >= i) && (i > d_DeltaAngle_2*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_2*70/100 >= i) && (i > d_DeltaAngle_2*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i >= d_DeltaAngle_2; i--)
			{
				RotateServoMotor_2(d_PreviousAngle_2+i);
				DelayFunction(8);
			}
		}
	}
	d_PreviousAngle_2 = a;
}

void RotateServoMotor_3(uint32_t a)
{
    d_AngleRotating_3 = a;
    TimerDisable(TIMER2_BASE, TIMER_A);
    g_NumOfPulses_3 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_3/1800);
    TimerLoadSet(TIMER2_BASE,TIMER_A,g_NumOfPulses_3);
    TimerEnable(TIMER2_BASE,TIMER_A);

    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_PIN_4);
}

void AdvancedRotateServoMotor_3(uint32_t a)
{
	int i,d_DeltaAngle_3;
	d_DeltaAngle_3 = a - d_PreviousAngle_3;
	if(d_DeltaAngle_3 >= 0)
	{
		if(d_DeltaAngle_3 >= 10)
		{
			for(i = 1; i <= d_DeltaAngle_3; i++)
			{
				RotateServoMotor_3(d_PreviousAngle_3+i);
				if((1 <= i) && (i < d_DeltaAngle_3*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_3*15/100 <= i) && (i < d_DeltaAngle_3*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_3*30/100 <= i) && (i < d_DeltaAngle_3*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_3*70/100 <= i) && (i < d_DeltaAngle_3*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i <= d_DeltaAngle_3; i++)
			{
				RotateServoMotor_3(d_PreviousAngle_3+i);
				DelayFunction(8);
			}
		}
	}
	else
	{
		if(d_DeltaAngle_3 <= -10)
		{
			for(i = -1; i >= d_DeltaAngle_3; i--)
			{
				RotateServoMotor_3(d_PreviousAngle_3+i);
				if((-1 >= i) && (i > d_DeltaAngle_3*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_3*15/100 >= i) && (i > d_DeltaAngle_3*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_3*30/100 >= i) && (i > d_DeltaAngle_3*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_3*70/100 >= i) && (i > d_DeltaAngle_3*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i >= d_DeltaAngle_3; i--)
			{
				RotateServoMotor_3(d_PreviousAngle_3+i);
				DelayFunction(8);
			}
		}
	}
	d_PreviousAngle_3 = a;
}

void RotateServoMotor_4(uint32_t a)
{
    d_AngleRotating_4 = a;
    TimerDisable(TIMER3_BASE, TIMER_A);
    g_NumOfPulses_4 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_4/1800);
    TimerLoadSet(TIMER3_BASE,TIMER_A,g_NumOfPulses_4);
    TimerEnable(TIMER3_BASE,TIMER_A);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,GPIO_PIN_5);
}

void AdvancedRotateServoMotor_4(uint32_t a)
{
	int i,d_DeltaAngle_4;
	d_DeltaAngle_4 = a - d_PreviousAngle_4;
	if(d_DeltaAngle_4 >= 0)
	{
		if(d_DeltaAngle_4 >= 10)
		{
			for(i = 1; i <= d_DeltaAngle_4; i++)
			{
				RotateServoMotor_4(d_PreviousAngle_4+i);
				if((1 <= i) && (i < d_DeltaAngle_4*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_4*15/100 <= i) && (i < d_DeltaAngle_4*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_4*30/100 <= i) && (i < d_DeltaAngle_4*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_4*70/100 <= i) && (i < d_DeltaAngle_4*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i <= d_DeltaAngle_4; i++)
			{
				RotateServoMotor_4(d_PreviousAngle_4+i);
				DelayFunction(8);
			}
		}
	}
	else
	{
		if(d_DeltaAngle_4 <= -10)
		{
			for(i = -1; i >= d_DeltaAngle_4; i--)
			{
				RotateServoMotor_4(d_PreviousAngle_4+i);
				if((-1 >= i) && (i > d_DeltaAngle_4*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_4*15/100 >= i) && (i > d_DeltaAngle_4*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_4*30/100 >= i) && (i > d_DeltaAngle_4*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_4*70/100 >= i) && (i > d_DeltaAngle_4*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i >= d_DeltaAngle_4; i--)
			{
				RotateServoMotor_4(d_PreviousAngle_4+i);
				DelayFunction(8);
			}
		}
	}
	d_PreviousAngle_4 = a;
}

void RotateServoMotor_5(uint32_t a)
{
    d_AngleRotating_5 = a;

    TimerDisable(TIMER4_BASE, TIMER_A);
    g_NumOfPulses_5 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_5/1800);
    TimerLoadSet(TIMER4_BASE,TIMER_A,g_NumOfPulses_5);
    TimerEnable(TIMER4_BASE,TIMER_A);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4,GPIO_PIN_4);
}

void AdvancedRotateServoMotor_5(uint32_t a)
{
	int i,d_DeltaAngle_5;
	d_DeltaAngle_5 = a - d_PreviousAngle_5;
	if(d_DeltaAngle_5 >= 0)
	{
		if(d_DeltaAngle_5 >= 10)
		{
			for(i = 1; i <= d_DeltaAngle_5; i++)
			{
				RotateServoMotor_5(d_PreviousAngle_5+i);
				if((1 <= i) && (i < d_DeltaAngle_5*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_5*15/100 <= i) && (i < d_DeltaAngle_5*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_5*30/100 <= i) && (i < d_DeltaAngle_5*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_5*70/100 <= i) && (i < d_DeltaAngle_5*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i <= d_DeltaAngle_5; i++)
			{
				RotateServoMotor_5(d_PreviousAngle_5+i);
				DelayFunction(8);
			}
		}
	}
	else
	{
		if(d_DeltaAngle_5 <= -10)
		{
			for(i = -1; i >= d_DeltaAngle_5; i--)
			{
				RotateServoMotor_5(d_PreviousAngle_5+i);
				if((-1 >= i) && (i > d_DeltaAngle_5*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_5*15/100 >= i) && (i > d_DeltaAngle_5*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_5*30/100 >= i) && (i > d_DeltaAngle_5*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_5*70/100 >= i) && (i > d_DeltaAngle_5*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i >= d_DeltaAngle_5; i--)
			{
				RotateServoMotor_5(d_PreviousAngle_5+i);
				DelayFunction(8);
			}
		}
	}
	d_PreviousAngle_5 = a;
}

void RotateServoMotor_6(uint32_t a)
{
    d_AngleRotating_6 = a;
    TimerDisable(TIMER5_BASE, TIMER_A);
    g_NumOfPulses_6 = SysCtlClockGet()/1000000*(500+19000*d_AngleRotating_6/1800);
    TimerLoadSet(TIMER5_BASE,TIMER_A,g_NumOfPulses_6);
    TimerEnable(TIMER5_BASE,TIMER_A);
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1,GPIO_PIN_1);
}

void AdvancedRotateServoMotor_6(uint32_t a)
{
	int i,d_DeltaAngle_6;
	d_DeltaAngle_6 = a - d_PreviousAngle_6;
	if(d_DeltaAngle_6 >= 0)
	{
		if(d_DeltaAngle_6 >= 10)
		{
			for(i = 1; i <= d_DeltaAngle_6; i++)
			{
				RotateServoMotor_6(d_PreviousAngle_6+i);
				if((1 <= i) && (i < d_DeltaAngle_6*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_6*15/100 <= i) && (i < d_DeltaAngle_6*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_6*30/100 <= i) && (i < d_DeltaAngle_6*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_6*70/100 <= i) && (i < d_DeltaAngle_6*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i <= d_DeltaAngle_6; i++)
			{
				RotateServoMotor_6(d_PreviousAngle_6+i);
				DelayFunction(8);
			}
		}
	}
	else
	{
		if(d_DeltaAngle_6 <= -10)
		{
			for(i = -1; i >= d_DeltaAngle_6; i--)
			{
				RotateServoMotor_6(d_PreviousAngle_6+i);
				if((-1 >= i) && (i > d_DeltaAngle_6*15/100))
				{
					DelayFunction(10);
				}
				else if((d_DeltaAngle_6*15/100 >= i) && (i > d_DeltaAngle_6*30/100))
				{
					DelayFunction(8);
				}
				else if((d_DeltaAngle_6*30/100 >= i) && (i > d_DeltaAngle_6*70/100))
				{
					DelayFunction(6);
				}
				else if((d_DeltaAngle_6*70/100 >= i) && (i > d_DeltaAngle_6*85/100))
				{
					DelayFunction(8);
				}
				else
				{
					DelayFunction(10);
				}
			}
		}
		else
		{
			for(i = 0; i >= d_DeltaAngle_6; i--)
			{
				RotateServoMotor_6(d_PreviousAngle_6+i);
				DelayFunction(8);
			}
		}
	}
	d_PreviousAngle_6 = a;
}

//ham ngat UART

void AdvancedRotateServoMotor_1_2_3(float x,float y,float z,char s_RotatingType)
 {
    float a2_1,a2_2,a3;
    if((x*x +y*y + z*z <= l2*l2 + l3*l3)||(z <= 0)||(x*x +y*y + z*z >= (l2+l3)*(l2+l3)))
    {
        Reserved_Error = 1;
    }
    else
    {
        Reserved_Error = 0;
    }
    if(Reserved_Error == 0)
    {
        if(x > 0)
        {
            d_AngleCalculated_1 = atan(y/x) * 180 / pi;
        }
        else if(x == 0)
        {
            d_AngleCalculated_1 = 90;
        }
        else
        {
            d_AngleCalculated_1 = 180 + atan(y/x) * 180 / pi;
        }
        a3 = acos((l2*l2 + l3*l3 - x*x -y*y - z*z)/(2*l2*l3)) * 180 / pi;
        a2_1 = acos((l2*l2 + x*x + y*y + z*z - l3*l3)/(2*sqrt(x*x + y*y + z*z)*l2)) * 180 / pi;
        a2_2 = asin(z/sqrt(x*x + y*y + z*z)) * 180 / pi;
        if(y >= 0)
        {
            d_AngleCalculated_2 = 180 - a2_1 - a2_2;
        }
        else
        {
            d_AngleCalculated_2 = a2_2 - a2_1;
        }
        d_AngleCalculated_3 = a3 - 90;
        //d_AngleCalculated_4 = a4;
        d_AngleTemp_1 = 180 - d_AngleCalculated_2;
		d_AngleTemp_2 = a3;

        switch(s_RotatingType)
        {
        	case 'f':
        		AdvancedRotateServoMotor_1(d_AngleCalculated_1);
				AdvancedRotateServoMotor_3(d_AngleCalculated_3);
				AdvancedRotateServoMotor_2(d_AngleCalculated_2);
				break;
        	case 'b':
        		AdvancedRotateServoMotor_2(d_AngleCalculated_2);
        		AdvancedRotateServoMotor_3(d_AngleCalculated_3);
				AdvancedRotateServoMotor_1(d_AngleCalculated_1);
				break;
        	default:
        		AdvancedRotateServoMotor_1(d_AngleCalculated_1);
        		AdvancedRotateServoMotor_2(d_AngleCalculated_2);
        		AdvancedRotateServoMotor_3(d_AngleCalculated_3);
				break;
        }

        //RotateServoMotor_4(d_AngleCalculated_4);
    }
    else
    {
    	AdvancedRotateServoMotor_1(90);
        AdvancedRotateServoMotor_2(90);
        AdvancedRotateServoMotor_3(90);
    }
 }

void AdvancedRotateServoMotor_1_2_3_4(float x2,float y2,float z2, int t_Angle,char s_RotatingType_2)
{
	float x1,y1,z1;
	x1 = x2*(1-(l4*cos(t_Angle*pi/180)/sqrt(x2*x2+y2*y2)));
	y1 = y2*(1-(l4*cos(t_Angle*pi/180)/sqrt(x2*x2+y2*y2)));
	z1 = l4*sin(t_Angle*pi/180) + z2;
	AdvancedRotateServoMotor_1_2_3(x1,y1,z1,s_RotatingType_2);
	d_AngleCalculated_4 = 360 - d_AngleTemp_1 - d_AngleTemp_2 - t_Angle;
	RotateServoMotor_4(d_AngleCalculated_4);
 }

void StartStep(void)
{
    AdvancedRotateServoMotor_2(80);
    AdvancedRotateServoMotor_4(120);
    AdvancedRotateServoMotor_1_2_3(0,-3,15,'f');
    RotateServoMotor_6(10);
}

void PreGrabStep(void)
{
    switch(n_PhaseCounter)
    {
		case 0:
			AdvancedRotateServoMotor_4(175);
			AdvancedRotateServoMotor_1_2_3_4(-6.58, 21.06, -7.56, 70,'f');
			AdvancedRotateServoMotor_4(175);
			break;
		case 1:
			AdvancedRotateServoMotor_4(175);
			AdvancedRotateServoMotor_1_2_3_4(3.04, 21.45, -7.56, 70,'f');
			AdvancedRotateServoMotor_4(175);
			break;
		case 2:
			AdvancedRotateServoMotor_4(150);
			AdvancedRotateServoMotor_1_2_3_4(-6.01, 14.65, -6.46, 80,'f');
			AdvancedRotateServoMotor_4(150);
			break;
		case 3:
			AdvancedRotateServoMotor_4(150);
			AdvancedRotateServoMotor_1_2_3_4(3.62, 15.3, -6.26, 80,'f');
			AdvancedRotateServoMotor_4(150);
			break;
    }
    DelayFunction(200);
}

void GrabStep(void)
{
  //  code do mo cua canh tay phu hop voi chieu rong qua d_UARTData
	int a = 55 - d_ObjectWidth[n_PhaseCounter] + 25;
	AdvancedRotateServoMotor_6(a);
	DelayFunction(200);

}


void MiddleStep()
{
//    switch(n_PhaseCounter)
//    {
//		case 0:
//			AdvancedRotateServoMotor_1_2_3(-5,16,10,'b');
//			//AdvancedRotateServoMotor_4(175);
//			break;
//		case 1:
//			AdvancedRotateServoMotor_1_2_3(2.3,16.2,10,'b');
//			//AdvancedRotateServoMotor_4(175);
//			break;
//		case 2:
//			AdvancedRotateServoMotor_1_2_3(-5.5,12,12.8,'b');
//			//AdvancedRotateServoMotor_4(50);
//			break;
//		case 3:
//			AdvancedRotateServoMotor_1_2_3(3,12.7,12.8,'b');
//			//AdvancedRotateServoMotor_4(50);
//			break;
//    }
	AdvancedRotateServoMotor_2(80);
	DelayFunction(200);
}


void PreReleaseStep(void)
{
    switch(d_ClassifiedType[n_PhaseCounter])
    {
		case 1:
			AdvancedRotateServoMotor_4(175);
			AdvancedRotateServoMotor_1_2_3_4(-18.2, 13.65, -2.42, 60,'f');
			AdvancedRotateServoMotor_4(175);
			break;
		case 2:
			AdvancedRotateServoMotor_4(175);
			AdvancedRotateServoMotor_1_2_3_4(-19.35, 6.45, -2.42, 60,'f');
			AdvancedRotateServoMotor_4(175);
			break;
		case 3:
			AdvancedRotateServoMotor_4(175);
			AdvancedRotateServoMotor_1_2_3_4(14.18, 15.76, 10.7, 60,'f');
			AdvancedRotateServoMotor_4(175);
			break;
		case 4:
			AdvancedRotateServoMotor_4(150);
			AdvancedRotateServoMotor_1_2_3_4(13.74, 6.87, 12.59, 70,'f');
			AdvancedRotateServoMotor_4(150);
			break;
    }
    DelayFunction(200);

}

void ReleaseStep(void)
{
// AdvancedRotateServoMotor_1_2_3(-10,0,11,'b');
	AdvancedRotateServoMotor_6(0);
    DelayFunction(200);
}

void UARTDataProcessing(char str[])
{
    char *p = strtok(str,".");
    int i=0;
    char str1[8][8];
    while(p != '\0')
    {
        strcpy(str1[i],p);
        i++;
        p = strtok('\0',".");
    }
    d_ObjectWidth[0] = atoi(str1[0]);
    d_ClassifiedType[0] = atoi(str1[1]);
    d_ObjectWidth[1] = atoi(str1[2]);
    d_ClassifiedType[1] = atoi(str1[3]);
    d_ObjectWidth[2] = atoi(str1[4]);
    d_ClassifiedType[2] = atoi(str1[5]);
    d_ObjectWidth[3] = atoi(str1[6]);
    d_ClassifiedType[3] = atoi(str1[7]);

}



void UARTGetBuffer(char *pBuff)
{

    static uint16_t i =0;
    char c;
    while(UARTCharsAvail(UART0_BASE))
    {
        c = UARTCharGet(UART0_BASE);
        *(pBuff+i) = c;
        d_UARTData[i]=c;
        i++;
    }
    if(c=='A')
    {
      d_UARTData[i-1]=0;
      i = 0;
      strcpy(d_UARTDataToProcess,d_UARTData);
      UARTDataProcessing(d_UARTDataToProcess);
    }
}
void UART_ISR(void)
{
    UARTIntClear(UART0_BASE,UARTIntStatus(UART0_BASE,true));
    UARTGetBuffer(&Buff[0]);
    TimerDisable(WTIMER2_BASE, TIMER_A);
    TimerLoadSet(WTIMER2_BASE,TIMER_A,SysCtlClockGet()*100/1000-1);
    TimerEnable(WTIMER2_BASE,TIMER_A);
    n_EventCounter = 0;

}


void ConfigUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0|GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE,UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk(UART0_BASE,SysCtlClockGet(),115200,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    UARTIntRegister(UART0_BASE,&UART_ISR);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_RT);
}



int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);   // 80MHZ
    ConfigUART();
    ConfigServoMotor_1();
    ConfigServoMotor_2();
    ConfigServoMotor_3();
    ConfigServoMotor_4();
    ConfigServoMotor_5();
    ConfigServoMotor_6();
    ConfigTimer_1();
    ConfigTimer_2();
    ConfigTimer_3();
    ConfigTimer_4();
    ConfigTimer_5();
    ConfigTimer_6();
    ConfigTimer_7();
    ConfigTimer_8();
    IntMasterEnable();
    l2 = 10.5;
    l3 = 9.8;
    l4 = 15.5;
    d_PreviousAngle_1 = 80;
    d_PreviousAngle_2 = 30;
    d_PreviousAngle_4 = 90;
    AdvancedRotateServoMotor_1_2_3(0,-3,15,'b');
    AdvancedRotateServoMotor_4(120);
    RotateServoMotor_5(105);
    RotateServoMotor_6(0);
    n_EventCounter = 0;
    first_time = 0;
    //StartStep();
    //DelayFunction(10);
    //PreGrabStep();
    //GrabStep();
    //MiddleStep();
    //PreReleaseStep();
    //ReleaseStep();
    //AdvancedRotateServoMotor_1_2_3(0,-5,17);
    //RotateServoMotor_1(0);
    //RotateServoMotor_2(0);
    //RotateServoMotor_3(0);
    //RotateServoMotor_6(90);
    //AdvancedRotateServoMotor_1(90);
    //AdvancedRotateServoMotor_1();
    //AdvancedRotateServoMotor_2(0);
    //AdvancedRotateServoMotor_3(0);
    //AdvancedRotateServoMotor_5(45);
    //AdvancedRotateServoMotor_6(70);
    //AdvancedRotateServoMotor_6(90);

    while(1)
    {
    	if(n_EventCounter == 1)
    	{
//    		if(first_time == 0)
//    		{
//    			DelayFunction(1900);
//    			first_time = 1;
//    		}
    		ClassificationProcess();
    	}
    }

}

