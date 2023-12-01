/*
 * Lights_Module.c
 *
 *  Created on: May 16, 2023
 *      Author: K. David
 */

#include "Lights_Module.h"
#include "Std_Types.h"
#include "main.h"

#define PERCENTAGE_50 50u
#define PERCENTAGE_25 25u
#define PERCENTAGE_75 75u
#define PERCENTAGE_0 0u

/* Private function prototypes */
static void Lights_TurnOffAllLights();
static void Lights_TurnOnAllLights();
static void Lights_HandlePositionLights(uint8_t status);
static void Lights_HandleLowBeam(uint8_t status);
static void Lights_HandleHighBeam(uint8_t status);
/* ------------------------------------------------ */

void Lights_TurnOffAllLights()
{
	/* Turn off Position Lights */
	Lights_HandlePositionLights(OFF);

	/* Turn off Lowbeam */
	Lights_HandleLowBeam(OFF);

	/* Turn off Highbeam */
	Lights_HandleHighBeam(ON);
}

void Lights_TurnOnAllLights()
{
	/* Turn on Position Lights */
	Lights_HandlePositionLights(ON);

	/* Turn on Lowbeam */
	Lights_HandleLowBeam(ON);

	/* Turn on Highbeam */
	Lights_HandleHighBeam(OFF);
}

void Lights_HandlePositionLights(uint8_t status)
{
	/* Handle Position Lights */
	HAL_GPIO_WritePin(Lights_Pos_Left_GPIO_Port, Lights_Pos_Left_Pin, status);
	HAL_GPIO_WritePin(Lights_Pos_Right_GPIO_Port, Lights_Pos_Right_Pin, status);

	/* Turn off HighBeam */
	Lights_HandleHighBeam(OFF);
}

void Lights_HandleLowBeam(uint8_t status)
{
	/* Handle Left LowBeam */
	HAL_GPIO_WritePin(Lights_LowBeam_Bottom_Left_GPIO_Port, Lights_LowBeam_Bottom_Left_Pin, status);
	HAL_GPIO_WritePin(Lights_LowBeam_Top_Left_GPIO_Port, Lights_LowBeam_Top_Left_Pin, status);

	/* Handle Right LowBeam */
	HAL_GPIO_WritePin(Lights_LowBeam_Bottom_Right_GPIO_Port, Lights_LowBeam_Bottom_Right_Pin, status);
	HAL_GPIO_WritePin(Lights_LowBeam_Top_Right_GPIO_Port, Lights_LowBeam_Top_Right_Pin, status);

	/* Turn off HighBeam */
	Lights_HandleHighBeam(OFF);
}

void Lights_HandleHighBeam(uint8_t status)
{
	/* Handle HighBeam */
	HAL_GPIO_WritePin(Lights_HighBeam_GPIO_Port, Lights_HighBeam_Pin, status);

}

/* ---------------- Exported functions ---------------- */
void Lights_Module_Init()
{
	/* Initialize all lights by turning them off */
	Lights_TurnOffAllLights();
}

void Lights_Module_Start_TaskCyclicEvent()
{

	uint8_t driverPresent;
	Rte_Read_DriverPresence_value(&driverPresent);
	if(FALSE == driverPresent)
	{
			uint8_t light_status;
			Rte_Read_CommandBuffer_Lights_status(&light_status);

			/* Handle Position Lights */
			Lights_HandlePositionLights(light_status);

			/* Handle Lowbeam */
			Lights_HandleLowBeam(light_status);

			/* Highbeam cannot be controlled from distance - negative logic */
			Lights_HandleHighBeam(ON);

	}
	else
	{
		uint8_t turnedOff;
		uint8_t posLights;
		uint8_t lowBeam;
		uint8_t highBeam;
		uint8_t statusAuto;

		Rte_Read_Lights_OnOffStatus(&turnedOff);
		Rte_Read_Lights_PosLights(&posLights);
		Rte_Read_Lights_LowBeam(&lowBeam);
		Rte_Read_Lights_HighBeam(&highBeam);
		Rte_Read_Lights_Status_Auto(&statusAuto);

		/* Check if lights are turned on or off */
		if(FALSE != turnedOff)
		{
			Lights_TurnOffAllLights();
		}
		if( FALSE != posLights)
		{
			/* Turn on Position Lights */
			Lights_HandlePositionLights(ON);
		}
		if( FALSE != lowBeam)
		{
			/* Turn on Lowbeam */
			Lights_HandleLowBeam(ON);
		}
		else
		{
			Lights_HandleLowBeam(OFF);
		}
		if( FALSE != highBeam)
		{
			/* Turn on Highbeam */
			Lights_HandleHighBeam(OFF);
		}
		else
		{
			Lights_HandleHighBeam(ON);
		}
		if( FALSE != statusAuto)
		{
			uint8_t lu8_photosen_prcntg;

			/* Turn off all lights */
			Lights_TurnOffAllLights();

			/* Activate Position Lights */
			Lights_HandlePositionLights(ON);

			/* Read percentage */
			Rte_Read_PhotoSen_prcntg(&lu8_photosen_prcntg);

			if( (lu8_photosen_prcntg <= PERCENTAGE_75) && (lu8_photosen_prcntg >= PERCENTAGE_50) )			 /* Might calibrate it after the Photosen is integrated */
			{
				Lights_HandleLowBeam(ON);
				Lights_HandleHighBeam(ON); /* Turn off highbeam */
			}
			else if( (lu8_photosen_prcntg < PERCENTAGE_50) && (lu8_photosen_prcntg >= PERCENTAGE_0) )		 /* Might calibrate it after the Photosen is integrated */
			{
				Lights_HandleLowBeam(ON);
				Lights_HandleHighBeam(OFF); /* Turn on highbeam */
			}
		}
		else
		{
			/* Do nothing */
		}
	}

}

