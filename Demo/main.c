//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include "diag/Trace.h"
//#include "Timer.h"

#include "uart.h"
#include <stm32f10x.h>



// ----- Timing definitions -------------------------------------------------

//#define TICKS  (TIMER_FREQUENCY_HZ )

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 empty sample (trace via STDOUT).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the STDOUT output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

//#define myUSART USART1
#define myUSART 1
#define espUSART 2

#define WIFI_NET "MYNET"
#define WIFI_PWD "MYPWD"

uint8_t buf[QUEUE_SIZE];
uint8_t nbyte = 32; //size_t

uint8_t cmd[16];
uint8_t cmdi; //size_t

uint8_t cmd_feedback[32];

uint8_t out[32];
uint8_t tmp[5];

uint32_t ticks = 0;
GPIO_InitTypeDef GPIO_InitStructure ;
uint8_t getValueFromOnes(uint8_t start, uint8_t to);
void ADC_Calibrate();

uint8_t mode_conf_wifi = 0; // 1=get myUART commands
void initWifi();

// avoid newlib
int strlen(char *s);
int strcmp(const char* s1, const char* s2);
char *strcpy(char *dst, char *src);
char *strcat(char *dest, const char *src) ;
void get_dec_str (uint8_t* str, uint8_t len, uint32_t val);
void itoa(char *buf, int base, int d);

uint8_t ri;
        
int main(int argc, char* argv[])
{

	RCC_APB2PeriphClockCmd (  RCC_APB2Periph_GPIOA  | RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1 |  RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3 | RCC_APB1Periph_USART2 , ENABLE );

	// Configure the NVIC to enable interrupt channel TIM3_IRQn  with the
	// highest priority (0 for both sub-priority and preemption priority)
	// ONLY TIM1 can be split in TIM1_CC_IRQn and TIM1_UP_IRQn
	NVIC_InitTypeDef NVIC_InitStructure ;
	NVIC_InitStructure . NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure . NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure . NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure . NVIC_IRQChannelCmd = ENABLE ;
	NVIC_Init (& NVIC_InitStructure );

	// Configure the NVIC to enable interrupt channel ADC1_IRQn with the
	// highest priority (0 for both sub-priority and preemption priority)
	NVIC_InitStructure . NVIC_IRQChannel = ADC1_IRQn ;
	NVIC_InitStructure . NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure . NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure . NVIC_IRQChannelCmd = ENABLE ;
	NVIC_Init (& NVIC_InitStructure );

	// PWM input DH11 sensor
	GPIO_StructInit (& GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;  // TIM3_CH1: STM32VL PA6 , on olimex PC6 -> PC4 (really)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ; // OpenDrain!!
	GPIO_Init (GPIOA , & GPIO_InitStructure );
	// no remap needed	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	//debug test points (oscilloscope)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; // on olimex PC8-9 (9 is under 8) (really)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_Init (GPIOC , & GPIO_InitStructure );

	//ADC_IN6  (MQ2 sensor - 5V)
	GPIO_StructInit (& GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; //STM32VL PA1  // on olimex PA6 is under PA4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN ;
	GPIO_Init (GPIOA , & GPIO_InitStructure );

	// ADC init
	ADC_InitTypeDef	 ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent ;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE ; //DISABLE ;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO; //ADC_ExternalTrigConv_T3_TRGO ;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right ;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init (ADC1 , & ADC_InitStructure );

	// Configure ADC_IN1
	ADC_RegularChannelConfig (ADC1 , ADC_Channel_1 , 1, ADC_SampleTime_55Cycles5 ); //ADC_SampleTime_55Cycles5 );

	// ADC Trigger
	ADC_ITConfig (ADC1 , ADC_IT_EOC , ENABLE );
	ADC_ExternalTrigConvCmd (ADC1 , ENABLE );

	ADC_Cmd (ADC1 , ENABLE );
	//calibration
	ADC_Calibrate();

	// configure timer TIM3 for tick every 1ms
	// PWM frequency = 100 hz with 72,000,000 hz system clock
	// 72,000,000/720 = 100,000
	// 100,000/100 = 1000
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
	TIM_TimeBaseStructInit (& TIM_TimeBaseStructure );
	TIM_TimeBaseStructure.TIM_Prescaler = 240 -1; //on olimex 720
	TIM_TimeBaseStructure.TIM_Period = 100 -1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInit (TIM3 , & TIM_TimeBaseStructure );
	TIM_SelectOutputTrigger (TIM3 , TIM_TRGOSource_Update );

	/* TIM3 configuration: PWM Input mode ------------------------
	The external signal is connected to TIM3 CH1 pin
	The Rising edge is used as active edge,
	The TIM3 CCR1 is used to compute the frequency value
	The TIM3 CCR2 is used to compute the duty cycle value
	----------------------------------------------------------- */
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1); 	// Select the TIM3 Input Trigger: TIM_TS_TI2FP2
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset); // Select the slave Mode: Reset Mode
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);	// Enable the Master/Slave Mode

	
	// open DEBUG console
	uart_open(myUSART, 115200, 0);

	// welcome
	uart_write(myUSART, "\n\n\r############################\r\n", strlen("\n\n\r############################\r\n"));
	uart_write(myUSART, "Welcome to the MeteoStation!\r\n", strlen("Welcome to the MeteoStation!\r\n"));
	uart_write(myUSART, "############################\r\n", strlen("############################\r\n"));

        //WIP
        initWifi() ;
        
	GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);

	TIM_Cmd (TIM3 , ENABLE ); 	// Enable the tick timer
	TIM_ITConfig (TIM3 , TIM_IT_Update , ENABLE ); 	// Enable Timer Interrupt , enable timer

	// prompt
	uart_write(myUSART, "\r\n# ", strlen("\r\n# "));

        cmdi = 0;
	// Infinite loop
	while (1)
	{
		if(mode_conf_wifi) 
                {
		
                    // read
                    ri = uart_read(myUSART, buf, nbyte);

                    if(ri > 0) {

                            for(int j=0; j<ri; j++, cmdi++)
                                    cmd[cmdi] = buf[j];

                            // \r = 0x0A, \n = 0x0D
                            if(buf[ri-1] == '\r') {

                                    //echo it back!!
                                    ri++;
                                    buf[ri-1] = '\n';
                                    uart_write(myUSART, buf, ri);

                                    // replace \r or \n with string end
                                    cmd[cmdi-1] = '\0';

                                    cmd_feedback[0] = '\0';
                                    //TODO SAVE NET & PWD
                                    if(strcmp(cmd, "WIFI_NET") == 0) {
                                            strcpy(cmd_feedback,"Received WIFI_NET :) \n\r");
                                    }
                                    else if(strcmp(cmd, "WIFI_PWD") == 0) {
                                            strcpy(cmd_feedback,"Received WIFI_PWD :) \n\r");
                                            
                                            //TODO - run initWifi(); with saved NET and PWD
                                    }

                                    if(strlen(cmd) > 0)
                                            uart_write(myUSART, cmd_feedback, strlen(cmd_feedback));

                                    // clean cmd
                                    cmd[0] = '\0';
                                    cmdi = 0;

                                    // prompt
                                    uart_write(myUSART, "# ", strlen("# "));
                            }
                            else {
                                    //echo it back!!
                                    uart_write(myUSART, buf, ri);
                            }
                    
                    }
              }
	}
	uart_close(myUSART);
}


void initWifi() 
{
        // open esp8266 wifi uart	
	uart_open(espUSART, 115200, 0);
	//uint32_t delay=1000000;
	
	// AT
	uart_write(espUSART, "AT\n\r", strlen("AT\n\r"));
	//delay=4000000000; while(delay) delay--;
	/*ri = uart_read(espUSART, buf, nbyte);
	tmp[0] = '\0';
        itoa(tmp, 'd', ri);
	uart_write(myUSART, tmp, strlen(tmp));*/
	uart_write(myUSART, buf, strlen(buf));

	// fw version
	//   it will output the firmware revision number similar to: 0018000902-AI03.
	//   000902 means firmware version 0.9.2, 0018 is the version level of the AT command support.
	uart_write(espUSART, "AT+GMR\n\r", strlen("AT+GMR\n\r"));
	//delay=4000000000; while(delay) delay--;
	ri = uart_read(espUSART, buf, nbyte);
	uart_write(myUSART, buf, strlen(buf));

	// Access Point and STAtion
	uart_write(espUSART, "AT+CWMODE=3\n\r", strlen("AT+CWMODE=3\n\r"));
	ri = uart_read(espUSART, buf, nbyte);
	uart_write(myUSART, buf, strlen(buf));

	// List Access Points
	//uart_write(espUSART, "AT+CWLAP\n\r", strlen("AT+CWLAP\n\r"));
	//ri = uart_read(espUSART, buf, nbyte);
        //uart_write(myUSART, buf, strlen(buf));

	// Join an Access Point
	uart_write(espUSART, "AT+CWJAP=\"" WIFI_NET "\",\"" WIFI_PWD "\"\n\r", strlen( "AT+CWJAP=\"" WIFI_NET "\",\"" WIFI_PWD "\"\n\r" ));
	/*strcpy(out,"AT+CWJAP=\"");
	strcat(out, WIFI_NET);
	strcat(out, "\",\"");
	strcat(out, WIFI_PWD);
	strcat(out, "\"\n\r");
	uart_write(espUSART, out, strlen(out)); */
	ri = uart_read(espUSART, buf, nbyte);
        uart_write(myUSART, buf, strlen(buf));
    
}





__IO uint16_t IC1Value = 0;
__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;


#define MAX 40
uint32_t freq_arr[MAX];
uint32_t dc_arr[MAX];
uint8_t Ones[MAX];

uint8_t cnt = 0;
uint8_t idx = 0;

uint8_t HUM_h;
uint8_t HUM_l;
uint8_t TEMP_h;
uint8_t TEMP_l;
uint8_t PARITY;

uint16_t ain = 0;

void TIM3_IRQHandler (void)
{

	if( TIM_GetFlagStatus (TIM3 , TIM_FLAG_CC1 ) == SET )
	{
		GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);  // <-- TEST

		ADC_ITConfig (ADC1 , ADC_IT_EOC , DISABLE ); // disable ADC in this part

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		IC1Value = TIM_GetCapture1(TIM3);
		IC2Value = TIM_GetCapture2(TIM3);

		if (IC2Value != 0 && IC1Value != 0)
		{
			//// Duty cycle computation
			DutyCycle = (IC2Value * 100) / IC1Value;
			//// Frequency computation
			// since it's rising edge.. it's related to the next bit... NOOO
			Frequency = SystemCoreClock / (IC1Value);

			// we remove part of the long low frame
			// and the spike when the MCU put the pin floating (or push it up)
			// and when thesensor initially put down for 80us
			// and then when the Sensor put level up for 80us (when you see ducycycle a little higher than bit 1)
			if(cnt >= 3) {

				freq_arr[idx] = Frequency;
				dc_arr[idx] = DutyCycle;

				////////// Theory /////////////////777
				// 1 should have DC = 58% and freq = 120 us
				// 0 should have DC = 34/36% and freq = 76/78 us
				////////// Practice ///////////////////7
				// 1 have DC = 56% and freq = 126/129 us
				// 0 have DC = 30/31% and freq = 80us
				// FREQUENCY READING IS WRONG!!! (it's inverted)

				if((DutyCycle >= 50 && DutyCycle <= 60) /*&& ( Frequency >= 12500 && Frequency <= 13100)*/) {
					Ones[idx] =	1;
				}
				else if ((DutyCycle >= 25 && DutyCycle <= 35) /*&& ( Frequency >= 7500 && Frequency <= 8500)*/) {
					Ones[idx] =	0;
				}
				else
					Ones[idx] =	2;  // DISCARD
				idx++;
			}
			cnt++;

			if(idx >= MAX) {
				cnt = 0;
				idx = 0;
				TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
				//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE); // in Olimex is needed

				GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET);
				TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

				HUM_h = getValueFromOnes(0, 7);
				//HUM_l = getValueFromOnes(8, 15);  // all zeros in this sensor model
				TEMP_h = getValueFromOnes(16, 23);
				//TEMP_l = getValueFromOnes(24, 31); // all zeros in this sensor model
				PARITY = getValueFromOnes(32, 39);

				ADC_ITConfig (ADC1 , ADC_IT_EOC , ENABLE );  // enable ADC now :)
			}
		}
		else
		{
			DutyCycle = 0;
			Frequency = 0;
		}
		GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_RESET);  // <-- TEST
		return;
	}

	if( TIM_GetFlagStatus (TIM3 , TIM_FLAG_Update ) == SET )
	{
		GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_SET);  // <-- TEST
		TIM_ClearITPendingBit (TIM3 , TIM_IT_Update );

		ticks++;
		if(ticks == 18)
		{
			TIM_ITConfig (TIM3 , TIM_IT_Update , DISABLE );

			//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); // in Olimex is needed

			GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET);
			TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
		}
		else if(ticks == 1000) {
			//trace_printf("HUM_h = %d, HUM_l = %d, TEMP_h = %d, TEMP_l = %d, PARITY = %d \n", HUM_h, HUM_l, TEMP_h, TEMP_l, PARITY);
			//trace_printf("HUM = %d, TEMP = %d° C, PARITY = %d \n", HUM_h,  TEMP_h,  PARITY);
			//trace_printf("HUM = %d, TEMP = %d° C, GAS = %d \n", HUM_h,  TEMP_h,  ain);
			//strcpy(out,"OUT :) \n\r");
			//uart_write(myUSART, out, strlen(out));
			if(!mode_conf_wifi)
			{
				strcpy(out,"HUM = ");
				tmp[0] = '\0';
				itoa(tmp, 'd', HUM_h); //get_dec_str(tmp, 2, HUM_h);
				strcat(out, tmp);
				strcat(out, ", TEMP = ");
				tmp[0] = '\0';
				itoa(tmp, 'd', TEMP_h); //get_dec_str(tmp, 2,TEMP_h);
				strcat(out, tmp);
				strcat(out, ", GAS = ");
				tmp[0] = '\0';
				itoa(tmp, 'd', ain); //get_dec_str(tmp, 4, ain);;
				strcat(out, tmp);
				strcat(out, "\r\n");
				uart_write(myUSART, out, strlen(out));
			}
		}
		else if(ticks == 2000 - 18) {
			ticks = 0;
			GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
		}
		GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_RESET);  // <-- TEST
	}


}





//gets the value of BitArray
uint8_t getValueFromOnes(uint8_t start, uint8_t to)
{
	uint8_t value = 0;
	uint8_t bitValue = Ones[start];

	value |= bitValue;
	for(uint8_t i = start; i <= to; i++)
	{
		bitValue = Ones[i];
		bitValue <<= (to - i);
		value |= bitValue;
	}
	return value;
}





/*
MQ-2 normal air output  100
     Isopropile alcohol 540
     Ligther Gas        760
     Benzine            450
     Breath1            150
     Breath2            140
*/

void ADC1_IRQHandler (void)
{
	//GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);  // <-- TEST

	// read ADC DR
	ain = ADC_GetConversionValue (ADC1);
	ADC_ClearITPendingBit (ADC1 , ADC_IT_EOC );

	//GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_RESET);  // <-- TEST
}



// not mandatory
void ADC_Calibrate()
{
	// Check the end of ADC1 reset calibration register
	while ( ADC_GetResetCalibrationStatus (ADC1));
	// Start ADC1 calibration
	ADC_StartCalibration (ADC1);
	// Check the end of ADC1 calibration
	while ( ADC_GetCalibrationStatus (ADC1));

}


/////////////////////////////////
/////////////////////////////////


int strlen(char *s)
{
    char *p = s;
    while (*p != '\0')
        p++;
    return p - s;
}

char *strcpy(char *dst, char *src)
{
    while((*dst++ = *src++)!= '\0')
        ; // <<== Very important!!!
    return dst;
}

int strcmp(const char* s1, const char* s2)
{
    while(*s1 && (*s1==*s2))
        s1++,s2++;
    return *(const unsigned char*)s1-*(const unsigned char*)s2;
}

char *strcat(char *dest, const char *src) 
{
	char *s1 = dest;
	const char *s2 = src;
	char c;

	do {
		c = *s1++;
	} while (c != '\0');

	s1 -= 2;
	do {
		c = *s2++;
		*++s1 = c;
	} while (c != '\0');
	return dest;
}



/** 
 * Convert the integer D to a string and save the string in BUF. If
 * BASE is equal to 'd', interpret that D is decimal, and if BASE is
 * equal to 'x', interpret that D is hexadecimal.
 */
void itoa(char *buf, int base, int d) {
        char *p = buf;
        char *p1, *p2;
        unsigned long ud = d;
        int divisor = 10;

        /* If %d is specified and D is minus, put `-' in the head.  */
        if (base == 'd' && d < 0) {
                *p++ = '-';
                buf++;
                ud = -d;
        } else if (base == 'x') {
                divisor = 16;
        }

        /* Divide UD by DIVISOR until UD == 0.  */
        do {
                int remainder = ud % divisor;

                *p++ = (remainder < 10) ? remainder + '0' : remainder + 'a' - 10;
        } while (ud /= divisor);

        /* Terminate BUF.  */
        *p = 0;

        /* Reverse BUF.  */
        p1 = buf;
        p2 = p - 1;
        while (p1 < p2) {
                char tmp = *p1;
                *p1 = *p2;
                *p2 = tmp;
                p1++;
                p2--;
        }
}


void get_dec_str (uint8_t* str, uint8_t len, uint32_t val)
{
	uint8_t i;
	for(i=1; i<=len; i++)
	{
		str[len-i] = (uint8_t) ((val % 10UL) + '0');
		val/=10;
	}

	str[i-1] = '\0';
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    // Infinite loop
    // Use GDB to find out why we're here 
    while (1);
}
#endif



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
