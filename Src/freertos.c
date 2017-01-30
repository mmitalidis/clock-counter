/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "HD44780.h"
#include <inttypes.h>

#define SET(a)    (a = 1)
#define RESET(a)  (a = 0)
#define TOGGLE(a) (a = (a == 1 ? 0 : 1))
#define IS_SET(a) (a != 0)
#define WAIT_FOR_MUTEX(mutex_id, wait_time) \
	do { \
		if (mutex_id == NULL) { \
			Error_Handler(); \
		} \
		if (osMutexWait(mutex_id, wait_time) != osOK) { \
			Error_Handler(); \
		} \
	} while(0)
#define WAIT_FOR_WAKE_UP(evt) \
	do { \
		evt = osSignalWait(SIGNAL_WAKE_UP,osWaitForever); \
		if (evt.status != osEventSignal) { \
			Error_Handler(); \
		} \
	} while (0) \
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId taskDefaultHandle;
osThreadId taskCtrTickHandle;
osThreadId taskCtrResetHandle;
osThreadId taskCtrPauseHandle;
osThreadId taskCtrStartHandle;
osThreadId taskToggleHandle;
osThreadId taskClkTickHandle;
osThreadId taskClkHourHandle;
osThreadId taskClkMinHandle;
osThreadId taskClkSecHandle;
osThreadId taskUpdateLCDHandle;
osThreadId taskUpdateMsgHandle;
osMutexId mutexCtrPlayHandle;
osMutexId mutexCtrGoHandle;
osMutexId mutexStateHandle;
osMutexId mutexCtrTimeHandle;
osMutexId mutexCtrAppearTimeHandle;
osMutexId mutexClkTimeHandle;
osMutexId mutexButtonPressedHandle;

/* USER CODE BEGIN Variables */
char appear_time[APPEAR_TIME_LENGTH];

uint32_t counter_go          = 0;
uint32_t counter_play        = 1;
uint32_t state               = STATE_CLOCK;
uint32_t counter_time        = 0;
uint32_t counter_appear_time = 0;

uint32_t clock_time          = 0;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

extern osThreadId selectedTask;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void thread_Default(void const * argument);
void thread_CtrTick(void const * argument);
void thread_CtrReset(void const * argument);
void thread_CtrPause(void const * argument);
void thread_CtrStart(void const * argument);
void thread_Toggle(void const * argument);
void thread_ClkTick(void const * argument);
void thread_ClkHour(void const * argument);
void thread_ClkMin(void const * argument);
void thread_ClkSec(void const * argument);
void thread_UpdateLCD(void const * argument);
void thread_UpdateMsg(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
extern void Error_Handler(void);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of mutexCtrPlay */
  osMutexDef(mutexCtrPlay);
  mutexCtrPlayHandle = osMutexCreate(osMutex(mutexCtrPlay));

  /* definition and creation of mutexCtrGo */
  osMutexDef(mutexCtrGo);
  mutexCtrGoHandle = osMutexCreate(osMutex(mutexCtrGo));

  /* definition and creation of mutexState */
  osMutexDef(mutexState);
  mutexStateHandle = osMutexCreate(osMutex(mutexState));

  /* definition and creation of mutexCtrTime */
  osMutexDef(mutexCtrTime);
  mutexCtrTimeHandle = osMutexCreate(osMutex(mutexCtrTime));

  /* definition and creation of mutexCtrAppearTime */
  osMutexDef(mutexCtrAppearTime);
  mutexCtrAppearTimeHandle = osMutexCreate(osMutex(mutexCtrAppearTime));

  /* definition and creation of mutexClkTime */
  osMutexDef(mutexClkTime);
  mutexClkTimeHandle = osMutexCreate(osMutex(mutexClkTime));

  /* definition and creation of mutexButtonPressed */
  osMutexDef(mutexButtonPressed);
  mutexButtonPressedHandle = osMutexCreate(osMutex(mutexButtonPressed));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of taskDefault */
  osThreadDef(taskDefault, thread_Default, osPriorityIdle, 0, 128);
  taskDefaultHandle = osThreadCreate(osThread(taskDefault), NULL);

  /* definition and creation of taskCtrTick */
  osThreadDef(taskCtrTick, thread_CtrTick, osPriorityRealtime, 0, 128);
  taskCtrTickHandle = osThreadCreate(osThread(taskCtrTick), NULL);

  /* definition and creation of taskCtrReset */
  osThreadDef(taskCtrReset, thread_CtrReset, osPriorityHigh, 0, 128);
  taskCtrResetHandle = osThreadCreate(osThread(taskCtrReset), NULL);

  /* definition and creation of taskCtrPause */
  osThreadDef(taskCtrPause, thread_CtrPause, osPriorityRealtime, 0, 128);
  taskCtrPauseHandle = osThreadCreate(osThread(taskCtrPause), NULL);

  /* definition and creation of taskCtrStart */
  osThreadDef(taskCtrStart, thread_CtrStart, osPriorityRealtime, 0, 128);
  taskCtrStartHandle = osThreadCreate(osThread(taskCtrStart), NULL);

  /* definition and creation of taskToggle */
  osThreadDef(taskToggle, thread_Toggle, osPriorityNormal, 0, 128);
  taskToggleHandle = osThreadCreate(osThread(taskToggle), NULL);

  /* definition and creation of taskClkTick */
  osThreadDef(taskClkTick, thread_ClkTick, osPriorityRealtime, 0, 128);
  taskClkTickHandle = osThreadCreate(osThread(taskClkTick), NULL);

  /* definition and creation of taskClkHour */
  osThreadDef(taskClkHour, thread_ClkHour, osPriorityHigh, 0, 128);
  taskClkHourHandle = osThreadCreate(osThread(taskClkHour), NULL);

  /* definition and creation of taskClkMin */
  osThreadDef(taskClkMin, thread_ClkMin, osPriorityHigh, 0, 128);
  taskClkMinHandle = osThreadCreate(osThread(taskClkMin), NULL);

  /* definition and creation of taskClkSec */
  osThreadDef(taskClkSec, thread_ClkSec, osPriorityHigh, 0, 128);
  taskClkSecHandle = osThreadCreate(osThread(taskClkSec), NULL);

  /* definition and creation of taskUpdateLCD */
  osThreadDef(taskUpdateLCD, thread_UpdateLCD, osPriorityRealtime, 0, 512);
  taskUpdateLCDHandle = osThreadCreate(osThread(taskUpdateLCD), NULL);

  /* definition and creation of taskUpdateMsg */
  osThreadDef(taskUpdateMsg, thread_UpdateMsg, osPriorityRealtime, 0, 512);
  taskUpdateMsgHandle = osThreadCreate(osThread(taskUpdateMsg), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */

  // Init commands - should be placed after thread initialization.
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);

  selectedTask = taskToggleHandle;
  /* USER CODE END RTOS_QUEUES */
}

/* thread_Default function */
void thread_Default(void const * argument)
{

  /* USER CODE BEGIN thread_Default */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END thread_Default */
}

/* thread_CtrTick function */
void thread_CtrTick(void const * argument)
{
  /* USER CODE BEGIN thread_CtrTick */
  /* Infinite loop */
	osEvent evt;
	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// check if counter_time should be incremented
		WAIT_FOR_MUTEX(mutexCtrGoHandle,osWaitForever);
		if( IS_SET(counter_go) )
		{
			osMutexRelease(mutexCtrGoHandle);

			// increment counter go
			WAIT_FOR_MUTEX(mutexCtrTimeHandle,osWaitForever);
			counter_time = counter_time + 1;
			osMutexRelease(mutexCtrTimeHandle);
		}
		else
		{
			osMutexRelease(mutexCtrGoHandle);
		}


		// check if counter_appear_time should be updated
		WAIT_FOR_MUTEX(mutexCtrPlayHandle,osWaitForever);
		if ( IS_SET(counter_play) )
		{
			osMutexRelease(mutexCtrPlayHandle);

			// update counter appear time
			WAIT_FOR_MUTEX(mutexCtrAppearTimeHandle,osWaitForever);
			WAIT_FOR_MUTEX(mutexCtrTimeHandle,osWaitForever);

			counter_appear_time = counter_time;

			osMutexRelease(mutexCtrTimeHandle);
			osMutexRelease(mutexCtrAppearTimeHandle);
		}
		else
		{
			osMutexRelease(mutexCtrPlayHandle);
		}
	}
  /* USER CODE END thread_CtrTick */
}

/* thread_CtrReset function */
void thread_CtrReset(void const * argument)
{
  /* USER CODE BEGIN thread_CtrReset */
	osEvent evt;
	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// clear counter go
		WAIT_FOR_MUTEX(mutexCtrGoHandle,osWaitForever);
		RESET(counter_go);
		osMutexRelease(mutexCtrGoHandle);

		// clear counter play
		WAIT_FOR_MUTEX(mutexCtrPlayHandle,osWaitForever);
		RESET(counter_play);
		osMutexRelease(mutexCtrPlayHandle);

		// clear counter time
		WAIT_FOR_MUTEX(mutexCtrTimeHandle,osWaitForever);
		RESET(counter_time);
		osMutexRelease(mutexCtrTimeHandle);

		// clear counter appear time
		WAIT_FOR_MUTEX(mutexCtrAppearTimeHandle,osWaitForever);
		counter_appear_time = 0;
		osMutexRelease(mutexCtrAppearTimeHandle);
	}
  /* USER CODE END thread_CtrReset */
}

/* thread_CtrPause function */
void thread_CtrPause(void const * argument)
{
  /* USER CODE BEGIN thread_CtrPause */
	osEvent evt;
	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// toggle counter play
		WAIT_FOR_MUTEX(mutexCtrPlayHandle,osWaitForever);
		TOGGLE(counter_play);
		osMutexRelease(mutexCtrPlayHandle);
	}
  /* USER CODE END thread_CtrPause */
}

/* thread_CtrStart function */
void thread_CtrStart(void const * argument)
{
  /* USER CODE BEGIN thread_CtrStart */
	osEvent evt;
	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		WAIT_FOR_MUTEX(mutexCtrTimeHandle,osWaitForever);

		if ( counter_time != 0 )
		{
			osMutexRelease(mutexCtrTimeHandle);

			// we already have counted
			WAIT_FOR_MUTEX(mutexCtrGoHandle,osWaitForever);

				if ( IS_SET(counter_go) )
				{
					// stop counting
					RESET(counter_go);
					osMutexRelease(mutexCtrGoHandle);

					// set counter play
					WAIT_FOR_MUTEX(mutexCtrPlayHandle,osWaitForever);
					SET(counter_play);
					osMutexRelease(mutexCtrPlayHandle);
				}
				else
				{
					// continue counting
					SET(counter_go);
					osMutexRelease(mutexCtrGoHandle);

					// set counter play
					WAIT_FOR_MUTEX(mutexCtrPlayHandle,osWaitForever);
					SET(counter_play);
					osMutexRelease(mutexCtrPlayHandle);
				}
		}
		else
		{
			osMutexRelease(mutexCtrTimeHandle);

			// we are in starting from zero mode

			// set counter go
			WAIT_FOR_MUTEX(mutexCtrGoHandle,osWaitForever);
			SET(counter_go);
			osMutexRelease(mutexCtrGoHandle);

			// set counter play
			WAIT_FOR_MUTEX(mutexCtrPlayHandle,osWaitForever);
			SET(counter_play);
			osMutexRelease(mutexCtrPlayHandle);
		}


	}
  /* USER CODE END thread_CtrStart */
}

/* thread_Toggle function */
void thread_Toggle(void const * argument)
{
  /* USER CODE BEGIN thread_Toggle */
	osEvent evt;
	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// increment time count
		WAIT_FOR_MUTEX(mutexStateHandle,osWaitForever);
		TOGGLE(state);
		osMutexRelease(mutexStateHandle);

	}
  /* USER CODE END thread_Toggle */
}

/* thread_ClkTick function */
void thread_ClkTick(void const * argument)
{
  /* USER CODE BEGIN thread_ClkTick */
	osEvent evt;
	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// increment time count
		WAIT_FOR_MUTEX(mutexClkTimeHandle,osWaitForever);
		clock_time = (clock_time + 1) % DAY_CONST;
		osMutexRelease(mutexClkTimeHandle);

	}
  /* USER CODE END thread_ClkTick */
}

/* thread_ClkHour function */
void thread_ClkHour(void const * argument)
{
  /* USER CODE BEGIN thread_ClkHour */
	osEvent evt;
	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// increment hour count
		WAIT_FOR_MUTEX(mutexClkTimeHandle,osWaitForever);
		clock_time = (clock_time + HOUR_CONST) % DAY_CONST;
		osMutexRelease(mutexClkTimeHandle);
	}
  /* USER CODE END thread_ClkHour */
}

/* thread_ClkMin function */
void thread_ClkMin(void const * argument)
{
  /* USER CODE BEGIN thread_ClkMin */
	osEvent evt;
	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// increment minute count
		WAIT_FOR_MUTEX(mutexClkTimeHandle,osWaitForever);
		clock_time = (clock_time + MINUTE_CONST) % DAY_CONST;
		osMutexRelease(mutexClkTimeHandle);
	}
  /* USER CODE END thread_ClkMin */
}

/* thread_ClkSec function */
void thread_ClkSec(void const * argument)
{
  /* USER CODE BEGIN thread_ClkSec */
	osEvent evt;
	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// set seconds to zero
		WAIT_FOR_MUTEX(mutexClkTimeHandle,osWaitForever);
		clock_time = clock_time - (clock_time % MINUTE_CONST);
		osMutexRelease(mutexClkTimeHandle);
	}
  /* USER CODE END thread_ClkSec */
}

/* thread_UpdateLCD function */
void thread_UpdateLCD(void const * argument)
{
  /* USER CODE BEGIN thread_UpdateLCD */
	osEvent evt;
	while (1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// put the LCD to work
		LCD_ClockTick();
	}
  /* USER CODE END thread_UpdateLCD */
}

/* thread_UpdateMsg function */
void thread_UpdateMsg(void const * argument)
{
  /* USER CODE BEGIN thread_UpdateMsg */
	osEvent evt;

	uint32_t mil = 0;
	uint32_t sec = 0;
	uint32_t min = 0;
	uint32_t hr  = 0;
	uint32_t tmp = 0;

	while(1)
	{
		// wait for wake up call
		WAIT_FOR_WAKE_UP(evt);

		// initialize variables
		mil = 0;
		sec = 0;
		min = 0;
		hr  = 0;
		tmp = 0;

		// increment time count
		WAIT_FOR_MUTEX(mutexStateHandle,osWaitForever);

		switch (state) {

		case STATE_CLOCK:
			osMutexRelease(mutexStateHandle);

			WAIT_FOR_MUTEX(mutexClkTimeHandle,osWaitForever);
			tmp = clock_time;
			osMutexRelease(mutexClkTimeHandle);

			// get seconds
			sec = tmp % 60;

			// get minutes
			tmp = tmp - sec;
			min = (tmp / MINUTE_CONST) % 60;

			// get hours
			tmp = tmp - (min * 60);
			hr = (tmp / HOUR_CONST) % 24;

			if (hr < 10) {
				appear_time[0] = '0';

			}
			else if (hr < 20) {
				appear_time[0] = '1';
				hr = hr - 10;
			}
			else {
				appear_time[0] = '2';
				hr = hr - 20;
			}

			// next we get the ascii value
			appear_time[1] = hr + 48;
			appear_time[2] = ':';

			if (min < 10) {
				appear_time[3] = '0';
			}
			else if (min < 20) {
				appear_time[3] = '1';
				min = min - 10;
			}
			else if (min < 30) {
				appear_time[3] = '2';
				min = min - 20;
			}
			else if (min < 40) {
				appear_time[3] = '3';
				min = min - 30;
			}
			else if (min < 50) {
				appear_time[3] = '4';
				min = min - 40;
			}
			else {
				appear_time[3] = '5';
				min = min - 50;
			}
			appear_time[4] = min + 48;
			appear_time[5] = ':';

			if (sec < 10) {
				appear_time[6] = '0';
			}
			else if (sec < 20) {
				appear_time[6] = '1';
				sec = sec - 10;
			}
			else if (sec < 30) {
				appear_time[6] = '2';
				sec = sec - 20;
			}
			else if (sec < 40) {
				appear_time[6] = '3';
				sec = sec - 30;
			}
			else if (sec < 50) {
				appear_time[6] = '4';
				sec = sec - 40;
			}
			else {
				appear_time[6] = '5';
				sec = sec - 50;
			}
			appear_time[7]  = sec + 48;
			appear_time[8]  = ' ';
			appear_time[9]  = ' ';
			appear_time[10] = ' ';
			appear_time[11] = ' ';
			appear_time[12] = '\0';

			break;

		case STATE_COUNTER:
			osMutexRelease(mutexStateHandle);

			WAIT_FOR_MUTEX(mutexCtrAppearTimeHandle,osWaitForever);
			tmp = counter_appear_time;
			osMutexRelease(mutexCtrAppearTimeHandle);

			// get milliseconds
			mil = tmp % 1000;

			// get seconds
			tmp = tmp - mil;
			sec = (tmp/1000) % 60;

			// get minutes
			tmp = tmp - sec * 1000;
			min = (tmp / MINUTE_CONST / 1000) % 60;

			// get hours
			tmp = tmp - (min * 60 * 1000);
			hr = (tmp / HOUR_CONST / 1000) % 24;


			if (hr < 10) {
				appear_time[0] = '0';

			}
			else if (hr < 20) {
				appear_time[0] = '1';
				hr = hr - 10;
			}
			else {
				appear_time[0] = '2';
				hr = hr - 20;
			}

			// next we get the ascii value
			appear_time[1] = hr + 48;
			appear_time[2] = ':';

			if (min < 10) {
				appear_time[3] = '0';
			}
			else if (min < 20) {
				appear_time[3] = '1';
				min = min - 10;
			}
			else if (min < 30) {
				appear_time[3] = '2';
				min = min - 20;
			}
			else if (min < 40) {
				appear_time[3] = '3';
				min = min - 30;
			}
			else if (min < 50) {
				appear_time[3] = '4';
				min = min - 40;
			}
			else {
				appear_time[3] = '5';
				min = min - 50;
			}
			appear_time[4] = min + 48;
			appear_time[5] = ':';

			if (sec < 10) {
				appear_time[6] = '0';
			}
			else if (sec < 20) {
				appear_time[6] = '1';
				sec = sec - 10;
			}
			else if (sec < 30) {
				appear_time[6] = '2';
				sec = sec - 20;
			}
			else if (sec < 40) {
				appear_time[6] = '3';
				sec = sec - 30;
			}
			else if (sec < 50) {
				appear_time[6] = '4';
				sec = sec - 40;
			}
			else {
				appear_time[6] = '5';
				sec = sec - 50;
			}
			appear_time[7] = sec + 48;
			appear_time[8] = '.';


			if (mil < 100) {
				appear_time[9] = '0';
			}
			else if (mil < 200) {
				appear_time[9] = '1';
				mil = mil - 100;
			}
			else if (mil < 300) {
				appear_time[9] = '2';
				mil = mil - 200;
			}
			else if (mil < 400) {
				appear_time[9] = '3';
				mil = mil - 300;
			}
			else if (mil < 500) {
				appear_time[9] = '4';
				mil = mil - 400;
			}
			else if (mil < 600) {
				appear_time[9] = '5';
				mil = mil - 500;
			}
			else if (mil < 700) {
				appear_time[9] = '6';
				mil = mil - 600;
			}
			else if (mil < 800) {
				appear_time[9] = '7';
				mil = mil - 700;
			}
			else if (mil < 900) {
				appear_time[9] = '8';
				mil = mil - 800;
			}
			else {
				appear_time[9] = '9';
				mil = mil - 900;
			}

			if (mil < 10) {
				appear_time[10] = '0';
			}
			else if (mil < 20) {
				appear_time[10] = '1';
				mil = mil - 10;
			}
			else if (mil < 30) {
				appear_time[10] = '2';
				mil = mil - 20;
			}
			else if (mil < 40) {
				appear_time[10] = '3';
				mil = mil - 30;
			}
			else if (mil < 50) {
				appear_time[10] = '4';
				mil = mil - 40;
			}
			else if (mil < 60) {
				appear_time[10] = '5';
				mil = mil - 50;
			}
			else if (mil < 70) {
				appear_time[10] = '6';
				mil = mil - 60;
			}
			else if (mil < 80) {
				appear_time[10] = '7';
				mil = mil - 70;
			}
			else if (mil < 90) {
				appear_time[10] = '8';
				mil = mil - 80;
			}
			else {
				appear_time[10] = '9';
				mil = mil - 90;
			}

			appear_time[11] = mil + 48;
			appear_time[12] = '\0';


			break;
		}

		// update LCD
		LCD_MoveToPosition(0x0);
		LCD_Print(appear_time);
	}
  /* USER CODE END thread_UpdateMsg */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
