/*
 * Inputs.c
 *
 *  Created on: May 23, 2023
 *      Author: K. David
 */

#include "Inputs.h"
#include "Std_Types.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
 uint16_t adcResultsDMA[2];
 uint8_t adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]);
 uint8_t adcConversionComplete = 0;
 static uint8_t timer_debounce_button_engine;
 static uint8_t handleEngine;

 uint8_t handle_debounce_Button_Engine()
 {
 	uint8_t retVal = FALSE;

 	if( HIGH == (HAL_GPIO_ReadPin(Button_Fan_GPIO_Port, Button_Fan_Pin)) )
 	{
 		if(0u == timer_debounce_button_engine)
 		{
 			timer_debounce_button_engine = TIMER_2S + 1;
 		}
 		else if(1u == timer_debounce_button_engine)
 		{
 			retVal = TRUE;
 			timer_debounce_button_engine = 0u;
 			handleEngine++;
 		}
 		else
 		{
 			timer_debounce_button_engine--;
 		}
 	}
 	else
 	{
 		timer_debounce_button_engine = 0u;
 	}
 	return retVal;
 }


void Handle_Inputs_Init()
{
	timer_debounce_button_engine = 0u;
	handleEngine = 0u;
}

void Handle_Inputs_TaskCyclicEvent()
{

		uint8_t lu8_debounce;
		uint8_t lu8_engineStatus;
		lu8_debounce = handle_debounce_Button_Engine();

		if( (TRUE == lu8_debounce) )
		{
			lu8_engineStatus = handleEngine & 1;
		}
		Rte_Write_CarSignals_EngineStatus(lu8_engineStatus);

		/* No calculations needed, just send the raw values through the RTE */
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcResultsDMA, adcChannelCount);
		Rte_Write_RawValues_rawLux(adcResultsDMA[0]);
		Rte_Write_RawValues_rawTemperature(adcResultsDMA[1]);

		/* Read ADC values for the potentiometer */
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, 100u);
		uint16_t pot_raw = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);

		Rte_Write_RawValues_rawPotentiometer(pot_raw);



}

