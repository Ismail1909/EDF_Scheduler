/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

/*Task Handlers*/
TaskHandle_t Load1Handler = NULL;
TaskHandle_t Load2Handler = NULL;
TaskHandle_t Button1Handler = NULL;
TaskHandle_t Button2Handler = NULL;
TaskHandle_t PeriodicHandler = NULL;
TaskHandle_t UartrxHandler = NULL;

int task1_IN = 0, task1_OUT = 0, task1_Time;
int task2_IN = 0, task2_OUT = 0, task2_Time;
int task3_IN = 0, task3_OUT = 0, task3_Time;
int task4_IN = 0, task4_OUT = 0, task4_Time;
int task5_IN = 0, task5_OUT = 0, task5_Time;
int task6_IN = 0, task6_OUT = 0, task6_Time;
int sys_time = 0;
int cpu_load = 0;

QueueHandle_t BT1 = NULL;
QueueHandle_t BT2 = NULL;
QueueHandle_t per = NULL;


char runTimeStatsBuf[200];

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/* Load_1_Simulation */
void Load_1_Simulation( void * pvParameters )
{
	int i=0;
	
	TickType_t xLastWakeTime = xTaskGetTickCount();

	vTaskSetApplicationTaskTag(NULL,(void *) 1);
    for( ;; )
    {
		
		/* Task code goes here. */
		for(i=0;i<39380;i++)
		{
			/*Simulate heavy load*/
		}
		
		vTaskDelayUntil( &xLastWakeTime, 10 );
		}
}

/* Load_2_Simulation */
void Load_2_Simulation( void * pvParameters )
{
	int i=0;

	TickType_t xLastWakeTime = xTaskGetTickCount();

	vTaskSetApplicationTaskTag(NULL,(void *) 2);
    for( ;; )
    {
		
		/* Task code goes here. */
		for(i=0;i<219617;i++)
		{
			/*Simulate heavy load*/
		}
		
		vTaskDelayUntil(&xLastWakeTime, 100);
		}
}

void Button_1_Monitor(void *pvParameters)
{
	pinState_t previousState = GPIO_read(PORT_1,PIN1);
	pinState_t currentState;
	char EdgeState; //Edge Flag
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	vTaskSetApplicationTaskTag(NULL,(void *) 3);
	for(;;)
	{
		currentState = GPIO_read(PORT_1,PIN1);
		if(currentState == PIN_IS_HIGH && previousState == PIN_IS_LOW)
		{
			EdgeState = '+';
		}
		else if(currentState == PIN_IS_LOW && previousState == PIN_IS_HIGH)
		{
			EdgeState = '-';
		}
		else
		{
			EdgeState = 'x';
		}
		
		
		
		xQueueOverwrite( BT1 , &EdgeState );
		
		vTaskDelayUntil( &xLastWakeTime, 50 );
		
	}
	
	
}

void Button_2_Monitor(void *pvParameters)
{
	pinState_t previousState = GPIO_read(PORT_1,PIN2);
	pinState_t currentState;
	char EdgeState; 
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	vTaskSetApplicationTaskTag(NULL,(void *) 4);
	for(;;)
	{
		currentState = GPIO_read(PORT_1,PIN2);
		if(currentState == PIN_IS_HIGH && previousState == PIN_IS_LOW)
		{
			EdgeState = '+';
		}
		else if(currentState == PIN_IS_LOW && previousState == PIN_IS_HIGH)
		{
			EdgeState = '-';
		}
		else
		{
			EdgeState = 'x';
		}
		previousState = currentState;
		
		
		xQueueOverwrite( BT2 , &EdgeState );
		
		vTaskDelayUntil( &xLastWakeTime, 50 );
		
	}
	
	
}

void Periodic_Transmitter(void *pvParameters)
{
	char msg[18] = "\n100ms Checkpoint";
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint8_t i=0;
	
	vTaskSetApplicationTaskTag(NULL,(void *) 5);
	for(;;)
	{
		for( i=0;i<18;i++)
		{
			xQueueSend(per,msg+i,100);
		}
		
		vTaskDelayUntil( &xLastWakeTime, 100 );
	}
	
}

void Uart_Receiver (void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	char bT1,bT2;
	char msg[18];
	uint8_t i=0;
	const signed char str1[10] = "Button1: ";
	const signed char str2[10] = "Button2: ";
	
	vTaskSetApplicationTaskTag(NULL,(void *) 6);
	for(;;)
	{
		if(xQueueReceive(BT1,&bT1,0))
		{
			if(bT1 != 'x')
			{
				xSerialPutChar('\n');
				xSerialPutChar('B');
				xSerialPutChar('1');
				xSerialPutChar(':');
				xSerialPutChar(bT1);
			}
			else
			{
				xSerialPutChar('\n');
				xSerialPutChar('B');
				xSerialPutChar('1');
				xSerialPutChar(':');
				xSerialPutChar(bT1);
			}
		}
			
		if(xQueueReceive(BT2,&bT2,0))
		{
			if(bT2 != 'x')
			{
				xSerialPutChar('\n');
				xSerialPutChar('B');
				xSerialPutChar('2');
				xSerialPutChar(':');
				xSerialPutChar(bT2);
			}
			else
			{
				xSerialPutChar('\n');
				xSerialPutChar('B');
				xSerialPutChar('2');
				xSerialPutChar(':');
				xSerialPutChar(bT2);
			}
		}
			
		if(uxQueueMessagesWaiting(per) != 0)
		{
			for( i=0 ; i<18 ; i++)
			{
				xQueueReceive( per, (msg+i) , 0);
			}
			vSerialPutString( (signed char *) msg, 18);
			xQueueReset(per);
		}
		
		vTaskDelayUntil( &xLastWakeTime , 20);
	
}
	}
		





/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	BT1 = xQueueCreate( 1, sizeof(char) );
	BT2 = xQueueCreate( 1, sizeof(char) );
	per = xQueueCreate( 18, sizeof(char) );

	
    /* Create Tasks here */
	
	/* Create the task, storing the handle. */
    xTaskPeriodicCreate(
						Load_1_Simulation,      		/* Function that implements the task. */
						"Load_1_Simulation",        /* Text name for the task. */
						100,      						/* Stack size in words, not bytes. */
						( void * ) 0,    			/* Parameter passed into the task. */
						1,										/* Priority at which the task is created. */
						&Load1Handler,        /* Used to pass out the created task's handle. */
						10); 			            /* Deadline */
    xTaskPeriodicCreate(
						Load_2_Simulation,      		/* Function that implements the task. */
						"Load_2_Simulation",        /* Text name for the task. */
						100,      						/* Stack size in words, not bytes. */
						( void * ) 0,    			/* Parameter passed into the task. */
						1,										/* Priority at which the task is created. */
						&Load2Handler,        /* Used to pass out the created task's handle. */
						100); 			          /* Deadline */
	xTaskPeriodicCreate(
						Button_1_Monitor,      		/* Function that implements the task. */
						"Button_1_Monitor",        /* Text name for the task. */
						100,      						/* Stack size in words, not bytes. */
						( void * ) 0,    			/* Parameter passed into the task. */
						1,										/* Priority at which the task is created. */
						&Button1Handler,				/* Used to pass out the created task's handle. */
						50);                  /* Deadline */
						
	xTaskPeriodicCreate(
						Button_2_Monitor,      		/* Function that implements the task. */
						"Button_2_Monitor",        /* Text name for the task. */
						100,      						/* Stack size in words, not bytes. */
						( void * ) 0,    			/* Parameter passed into the task. */
						1,										/* Priority at which the task is created. */
						&Button2Handler,				/* Used to pass out the created task's handle. */
						50);                  /* Deadline */
						
	xTaskPeriodicCreate(
						Periodic_Transmitter,      		/* Function that implements the task. */
						"Periodic_Transmitter",        /* Text name for the task. */
						100,      						/* Stack size in words, not bytes. */
						( void * ) 0,    			/* Parameter passed into the task. */
						1,										/* Priority at which the task is created. */
						&PeriodicHandler,				/* Used to pass out the created task's handle. */
						100);                  /* Deadline */
						
	xTaskPeriodicCreate(
						Uart_Receiver,      		/* Function that implements the task. */
						"Uart_Receiver",        /* Text name for the task. */
						100,      						/* Stack size in words, not bytes. */
						( void * ) 0,    			/* Parameter passed into the task. */
						1,										/* Priority at which the task is created. */
						&UartrxHandler,				/* Used to pass out the created task's handle. */
						20);                  /* Deadline */

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Configure Trace timer 1 and read T1TC to get current tick*/
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


