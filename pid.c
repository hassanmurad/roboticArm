#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "inc/hw_gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"

#define PWM_FREQUENCY 600
//Defineparameter
#define epsilon 0.01
#define dt 0.00008 //100mslooptime
#define MAX 4 //ForCurrent Saturation
#define MIN -4
#define Kp 500
#define Kd 0
#define Ki 0
#define ZERO_DUTY_CYCLE 1
#define encoderConversion 360/12600

uint32_t volatile status;
int16_t volatile channel1_;
int16_t volatile channel2_;
int16_t volatile channel1;
int16_t volatile channel2;
int16_t volatile overshot = 0;
int32_t enpo=0;
int32_t enpo_=0;
int32_t Denpo=0;
int32_t currentPosition;
double desiredPosition = 45;
double deadbandEpsilon = 5;
double volatile pidValue;
double volatile error;
double volatile derivative;
double volatile integral = 0;
double volatile pre_error = 0;
double volatile pulseWidth = 0;
int volatile settled = false;
int volatile inNegativePart = false;

int32_t volatile pwm_value;
int32_t counter1=0;

volatile uint32_t ui32Load2;
volatile uint32_t ui32Load;
volatile uint32_t ui32Load3;

double calculatePID(void);
void saturate(void);

void PortBPin6IntHandler(void){

	channel1=GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6);
	channel2=GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2);

	counter1++;

	if ((channel1_==0)&&(channel2_==0))
	{
		if ((channel1!=0)&&(channel2==0))
		{
			enpo++;
		}
		else if ((channel1==0)&&(channel2!=0))
			enpo--;
	}

	if ((channel1_!=0)&&(channel2_==0))
	{
		if ((channel1!=0)&&(channel2!=0))
		{
			enpo++;
		}
		else if ((channel1==0)&&(channel2==0))
					enpo--;
	}

	if ((channel1_!=0)&&(channel2_!=0))
	{
		if ((channel1==0)&&(channel2!=0))
		{
			enpo++;
		}
		else if ((channel1!=0)&&(channel2==0))
					enpo--;
	}

	if ((channel1_==0)&&(channel2_!=0))
		{
			if ((channel1==0)&&(channel2==0))
			{
				enpo++;
			}
			else if ((channel1!=0)&&(channel2!=0))
						enpo--;
		}

	channel1_=channel1;
	channel2_=channel2;

	GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2 |GPIO_PIN_6 );
}

void Timer1IntHandler(void)
{
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)){
    		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    }
    else{
    		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    }

    currentPosition = enpo*0.03157156; //Convert encoder value to degrees
    error = desiredPosition - currentPosition;
    pidValue = calculatePID();

    if (error > deadbandEpsilon || error < -1*deadbandEpsilon){
    		settled = false;
		if ((pidValue < 0) ) { //negative PID value, we need to come back
			inNegativePart = true;
			saturate();
			pwm_value = 0.001963483749023*pow(pidValue, 6) - 0.0696342924929922*pow(pidValue, 5) + 0.966972236521542*pow(pidValue, 4) - 6.42952954769134*pow(pidValue, 3) + 19.9319366961717*pow(pidValue, 2) - 20.3848745599389*pidValue;
			if (pwm_value > 2){
				pulseWidth = ((pwm_value-2) * 650); //ui32Load2 / 1000
			}
			else{
				pulseWidth = (pwm_value * 650);
			}
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ZERO_DUTY_CYCLE);
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, pulseWidth);
		}
		else { //positive PID value, we are heading to the target
			inNegativePart = false;
			saturate();
			pwm_value = 0.001963483749023*pow(pidValue, 6) - 0.0696342924929922*pow(pidValue, 5) + 0.966972236521542*pow(pidValue, 4) - 6.42952954769134*pow(pidValue, 3) + 19.9319366961717*pow(pidValue, 2) - 20.3848745599389*pidValue;
			if (pwm_value > 2){
				pulseWidth = ((pwm_value-2) * 650); //ui32Load2 / 1000
			}
			else{
				pulseWidth = (pwm_value * 650);
			}
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pulseWidth);
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ZERO_DUTY_CYCLE);
		}
    }
    else{
    		pulseWidth = 0;
    		settled = true;
		ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ZERO_DUTY_CYCLE);
		ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ZERO_DUTY_CYCLE);
    }
	pre_error = error;
	ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

void saturate(void){
	if (pidValue > 12){
		pidValue = 12;
	}
	else if (pidValue < -12){
		pidValue = -12;
	}
}

double calculatePID(void){
	static double pid = 0;
	//integral = integral + error*dt;
	integral = 0;
	derivative = (error - pre_error)/dt;
	pid = Kp*error + Ki*integral + Kd*derivative;
	return pid;
}

int main ()
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	//pwm first pin config section
	//M1PWM2
	//pin E4
	volatile uint32_t ui32PWMClock;
	volatile uint16_t ui8Adjust;
	ui8Adjust =100;
	volatile uint32_t ui32TempAvg;

	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
	ROM_GPIOPinConfigure(GPIO_PE4_M1PWM2);
	ui32PWMClock = SysCtlClockGet() / 64;

	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 1);
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
	//end of pwm first pin config section

	//pwm second pin config section
	//M1PWM3
	//pin E5
	volatile uint32_t ui32PWMClock2;
	volatile uint16_t ui8Adjust2;
	ui8Adjust2 = 100;
	volatile uint32_t ui32TempAvg2;

	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
	ROM_GPIOPinConfigure(GPIO_PE5_M1PWM3);
	ui32PWMClock2 = SysCtlClockGet() / 64;

	ui32Load2 = (ui32PWMClock2 / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load2);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load2 / 1000);
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	//end of pwm second pin config section


	//pwm third pin config section

	ui8Adjust2 = 100;

	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
	ROM_GPIOPinConfigure(GPIO_PD1_M1PWM1);
	ui32PWMClock2 = SysCtlClockGet() / 64;

	ui32Load2 = (ui32PWMClock2 / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load2);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8Adjust2 * ui32Load2 / 1000);
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	//end of pwm third pin config section

	//pwm fourth pin config section

	ui8Adjust2 = 100;

	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
	ROM_GPIOPinConfigure(GPIO_PC4_M0PWM6);
	ui32PWMClock2 = SysCtlClockGet() / 64;

	ui32Load2 = (ui32PWMClock2 / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Load2);
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, ui8Adjust2 * ui32Load2 / 1000);
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
	//end of pwm fourth pin config section

	// config of leds
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	//end of config of leds

	// config of external interrupts
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOIntRegister(GPIO_PORTB_BASE, PortBPin6IntHandler);
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_6 |  GPIO_PIN_2);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_2 );
	GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_BOTH_EDGES);
	GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2  |GPIO_PIN_6);
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2 |GPIO_PIN_6 );
	channel1_=GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6);
	channel2_=GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2);
	SysCtlDelay(SysCtlClockGet()  /10/ 3);
	IntMasterEnable();
	//end of config of external interrupts

	//config of timer interrupt

	ROM_FPULazyStackingEnable();

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

	ROM_IntMasterEnable();

	ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet() / 100);

	TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1IntHandler);

	ROM_IntEnable(INT_TIMER1A);
	ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	ROM_TimerEnable(TIMER1_BASE, TIMER_A);
	//end of config of time interrupt

	while (1){

		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
		SysCtlDelay(SysCtlClockGet()  /100/ 4);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
		SysCtlDelay(SysCtlClockGet()  /100/ 4);

	}

};
