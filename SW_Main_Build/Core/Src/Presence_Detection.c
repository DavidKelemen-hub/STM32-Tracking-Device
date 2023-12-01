/*
 * PresenceDetection.c
 *
 *  Created on: Jul 1, 2023
 *      Author: K. David
 */

#include "Presence_Detection.h"
#include "Std_Types.h"
#include "main.h"

/* Private variables */
static uint8_t touch_sensor_timer;
static uint8_t handle_debounce;
static uint8_t driverPresence;

static uint8_t handle_debounce_Touch_Sensor()
{
	uint8_t retVal = FALSE;

	if( HIGH == (HAL_GPIO_ReadPin(Touch_Sensor_GPIO_Port, Touch_Sensor_Pin)) )
	{
		if(0u == touch_sensor_timer)
		{
			touch_sensor_timer = TIMER_2S + 1;
		}
		else if(1u == touch_sensor_timer)
		{
			retVal = TRUE;
			touch_sensor_timer = 0u;
			handle_debounce++;
		}
		else
		{
			touch_sensor_timer--;
		}
	}
	else
	{
		touch_sensor_timer = 0u;
	}
	return retVal;
}


void Presence_Detection_Init()
{
	touch_sensor_timer = 0u;
	handle_debounce = 0u;
	driverPresence = DRIVER_PRESENT;
	HAL_GPIO_WritePin(Blue_GPIO_Port, Blue_Pin, OFF);
	HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, ON);

}


void Presence_Detection_TaskCyclicEvent()
{
	uint8_t doorStatus;
	uint8_t debounce_touchSensor;
	uint8_t vehicleSpeed;

	Rte_Read_Door_Status(&doorStatus);
	Rte_Read_CarSignals_VehicleSpeed(&vehicleSpeed);

	debounce_touchSensor = handle_debounce_Touch_Sensor();

	/* Case when driver exits */
	if( (DOOR_OPEN == doorStatus) && (KM_H_2 >= vehicleSpeed) && (TRUE == debounce_touchSensor) &&
		( 1u == (handle_debounce&1)	) )
	{
	  driverPresence = DRIVER_ABSENT;
	  HAL_GPIO_WritePin(Blue_GPIO_Port, Blue_Pin, ON);
	  HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, OFF);
	}
	else if ( 0u == (handle_debounce&1) )
	{
	 driverPresence = DRIVER_PRESENT;
	 HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, ON);
	 HAL_GPIO_WritePin(Blue_GPIO_Port, Blue_Pin, OFF);
	}
	else
	{
		/* Do nothing */
	}

	Rte_Write_CarSignals_DriverPresence(driverPresence);
}


