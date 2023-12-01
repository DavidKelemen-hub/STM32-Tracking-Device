/*
 * Servo.c
 *
 *  Created on: May 15, 2023
 *      Author: K. David
 */

#include "Servo.h"
#include "main.h"
#include "Std_Types.h"

extern TIM_HandleTypeDef htim2;
#define DEFAULT_POSITION 1350u

/*Private variables */
static uint8_t timer_debounce_button_door;
static uint8_t timer_betweenDebounce;
static uint8_t doors_last;
static uint8_t handle_door; /* even - door opened / odd - door is closed */

static uint8_t handle_debounce_Button_Door()
{
	uint8_t retVal = FALSE;

	if( HIGH == (HAL_GPIO_ReadPin(GPIOB, Button_Doors_Pin)) )
	{
		if(0u == timer_debounce_button_door)
		{
			timer_debounce_button_door = TIMER_2S + 1;
		}
		else if(1u == timer_debounce_button_door)
		{
			retVal = TRUE;
			timer_debounce_button_door = 0u;
			handle_door++;
		}
		else
		{
			timer_debounce_button_door--;
		}
	}
	else
	{
		timer_debounce_button_door = 0u;
	}
	return retVal;
}


void Servo_Start_Init()
{

	static uint8_t first_run = FALSE;
	timer_debounce_button_door = 0u;
	handle_door = 1u;

	if(first_run == FALSE)
	{
		first_run = TRUE;
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
		htim2.Instance->CCR1 = DEG90;
		htim2.Instance->CCR2 = DEG90;
	}
}

void Servo_Start_TaskCyclicEvent()
{

	uint8_t driverPresent;
	Rte_Read_DriverPresence_value(&driverPresent);

	if(FALSE == driverPresent)
	{
		uint8_t lu8_temp;
		Rte_Read_CommandBuffer_Door_status(&lu8_temp);

		if( (DOOR_CLOSED == lu8_temp) )
		{
			htim2.Instance->CCR1 = DEG180;   				/* Close Doors */
			htim2.Instance->CCR2 = DEG0;
			doors_last = DOOR_OPEN;
		}
		else
		{

			htim2.Instance->CCR1 = DEG90;      				/* Open Doors */
			htim2.Instance->CCR2 = DEG90;
			doors_last = DOOR_CLOSED;
		}
	}
	else
	{
		uint8_t lu8_debounce_doorStatus;

		lu8_debounce_doorStatus = handle_debounce_Button_Door();

		if( FALSE != lu8_debounce_doorStatus )
		{
			uint8_t status;
			Rte_Read_Door_Status(&status);

			if( (handle_door & 1) == 0)
			{
				htim2.Instance->CCR1 = DEG180;   /* Close Doors */
				htim2.Instance->CCR2 = DEG0;
			}
			else
			{
				htim2.Instance->CCR1 = DEG90;    /* Open Doors */
				htim2.Instance->CCR2 = DEG90;
			}


		}
	}




}
