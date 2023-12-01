/*
 * FanControl.c
 *
 *  Created on: Jul 1, 2023
 *      Author: K. David
 */

#include "FanControl.h"
#include "Std_Types.h"
#include "math.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

/*Private variables */
static uint8_t handleFan;
static timer_debounce_button_fan;
static uint8_t checkFanStatus;

/* Private function prototypes */
uint8_t handle_debounce_Button_Fan();
/* --------------------------------*/

uint8_t handle_debounce_Button_Fan()
{
	uint8_t retVal = FALSE;

	if( HIGH == (HAL_GPIO_ReadPin(Button_Fan_GPIO_Port, Button_Fan_Pin)) )
	{
		if(0u == timer_debounce_button_fan)
		{
			timer_debounce_button_fan = TIMER_2S + 1;
		}
		else if(1u == timer_debounce_button_fan)
		{
			retVal = TRUE;
			timer_debounce_button_fan = 0u;
			handleFan++;
		}
		else
		{
			timer_debounce_button_fan--;
		}
	}
	else
	{
		timer_debounce_button_fan = 0u;
	}
	return retVal;
}


/*Exported Functions*/
void FanControl_Init()
{

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	htim4.Instance->CCR1 = 0u;
	handleFan = 0u;
	checkFanStatus = 0u;

}

void FanControl_TaskCyclicEvent()
{

	uint8_t driverPresent;
	Rte_Read_DriverPresence_value(&driverPresent);

	if(FALSE == driverPresent)
	{

			  uint8_t fanStatus;
			  Rte_Read_CommandBuffer_Fan_status(&fanStatus);

			  if(ON == fanStatus)
			  {
				  HAL_GPIO_WritePin(In3_GPIO_Port, In3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(In4_GPIO_Port, In4_Pin, GPIO_PIN_SET);
			 	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
				  htim4.Instance->CCR1 = 1023u;
			  }
			  else
			  {
				  HAL_GPIO_WritePin(In3_GPIO_Port, In3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(In4_GPIO_Port, In4_Pin, GPIO_PIN_RESET);
			  }
	}
	else
	{

		uint8_t lu8_debounce;
		lu8_debounce = handle_debounce_Button_Fan();
		Rte_Write_CarSignals_FanStatus(handleFan&1);


		 if( (handleFan & 1u) == 1u ) /* need to check this */
		{
			uint8_t setpointTemp;
			uint8_t measuredTemp;
			uint16_t pwmValue;

			Rte_Read_Potentiometer_interval(&setpointTemp);
			Rte_Read_Temperature_Value(&measuredTemp);

			if(setpointTemp <= measuredTemp)
			{
				uint8_t lu8_percentage = map_values(abs(setpointTemp - measuredTemp),ZERO,TEN,ZERO_PERCENT,HUNDRED_PERCENT);
				pwmValue = map_values(lu8_percentage,ZERO,HUNDRED_PERCENT,650u,1023u);
				HAL_GPIO_WritePin(In3_GPIO_Port, In3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(In4_GPIO_Port, In4_Pin, GPIO_PIN_SET);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
				htim4.Instance->CCR1 = pwmValue;
				checkFanStatus = 1u;
			}
			else
			{
				HAL_GPIO_WritePin(In3_GPIO_Port, In3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(In4_GPIO_Port, In4_Pin, GPIO_PIN_RESET);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
				htim4.Instance->CCR1 = ZERO;
				checkFanStatus = 0u;
			}
		}
		else
		{
			HAL_GPIO_WritePin(In3_GPIO_Port, In3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(In4_GPIO_Port, In4_Pin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			htim4.Instance->CCR1 = ZERO;
			checkFanStatus = 0u;
		}

	}
	Rte_Write_CarSignals_FanStatus(checkFanStatus);

	}





