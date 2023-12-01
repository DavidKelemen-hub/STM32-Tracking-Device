/*
 * Transmit_Arduino.c
 *
 *  Created on: Jul 4, 2023
 *      Author: K. David
 */

#include "Transmit_Arduino.h".h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "main.h"
#include "Rte.h"
#include "Std_Types.h"

/*Private variables*/
extern UART_HandleTypeDef huart9;
static uint8_t speed;
static uint8_t temperature;
static uint8_t temperature_setpoint;
static uint8_t posLights;
static uint8_t lowBeam;
static uint8_t highBeam;
static uint8_t autoMode;
static uint8_t engineStatus;
static uint8_t fanStatus;
static char TX_BUFFER[25];

/*Exported functions */
void Transmit_Arduino_Init()
{
	/* Do nothing */
}

void Transmit_Arduino_TaskCyclicEvent()
{
	static uint8_t debounce=0;
	Rte_Read_CarSignals_VehicleSpeed(&speed);
	Rte_Read_Temperature_Value(&temperature);
	Rte_Read_Potentiometer_interval(&temperature_setpoint);
	Rte_Read_Lights_PosLights(&posLights);
	Rte_Read_Lights_LowBeam(&lowBeam);
	Rte_Read_Lights_HighBeam(&highBeam);
	Rte_Read_Lights_Status_Auto(&autoMode);
	Rte_Read_Engine_Status(&engineStatus);
	Rte_Read_Fan_Status(&fanStatus);
	engineStatus = ON;
	debounce++;
	if(debounce == 50)
	{
		debounce=0;
	sprintf(TX_BUFFER,"%d|%d|%d|%d|%d|%d|%d|%d|%d",speed,temperature,temperature_setpoint,posLights,lowBeam,highBeam,autoMode,engineStatus,fanStatus);
	HAL_UART_Transmit(&huart9, (uint8_t*)TX_BUFFER, strlen(TX_BUFFER), HAL_MAX_DELAY);}
}

