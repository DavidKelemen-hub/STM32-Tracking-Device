/*
 * Rte.c
 *
 *  Created on: May 15, 2023
 *      Author: K. David
 */

#include "Rte.h"
#include "Std_Types.h"
#include "Servo.h"
#include "Presence_Detection.h"
#include "Lights_Module.h"
#include "main.h"
#include "math.h"

extern TIM_HandleTypeDef htim2;


typedef struct CarSignals {

    uint8_t lowBeam;
    uint8_t highBeam;
    uint8_t posLights;
    uint8_t doorStatus;
    uint8_t fanStatus;
    uint8_t measuredTemperature;
    uint8_t engineStatus;
    uint8_t driverPresence;
    uint8_t vehicleSpeed;
} CarSignals_st;


typedef struct RawValues {
	uint16_t rawTemperature;
	uint16_t rawLux;
	uint16_t rawPotentiometer;
}RawValues_st;

CarSignals_st CarSignals = { 0u };
RawValues_st RawValues = { 0u };
uint8_t CommandValues[3] = { 0u, 1u, 0u }; /* Buffer used for control when Driver is not Present */

/* 0 - Speed ; 1 - Temperature ; 2 - Position Lights ; 3 - Lowbeam ; 4 - Highbeam ; 5 - Auto Mode * ; 6 - Temperature setpoint ;  7 - Engine Status ; 8 - Fan Status */
uint8_t DataPacket_Arduino[9] = { 0u };
/*this values will be sent to the Arduino */

/* Private function prototypes */
static uint8_t Get_Temperature(uint16_t adc_raw);
static double Get_Voltage(uint16_t adc_raw);
/* ------------------------------------------------ */


uint16_t map_values(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t Get_Temperature(uint16_t adc_raw)
{
	/* Local variables used for calculations */
	double Voltage,mV,temperature;

	/* Calculate Voltage[V] */
	Voltage = ( adc_raw / (pow(2,ADC_RESOLUTION) - 1u) ) * REF_5V;
	/* mV = V * 1000 */
	mV = Voltage * RESOLUTION_V_TO_mV;
	/* Calculate temperature */
	temperature = mV / 10;

	return (uint8_t)temperature;
}

double Get_Voltage(uint16_t adc_raw)
{
	/* Local variables used for calculations */
	double Voltage,mV;

	/* Calculate Voltage[V] */
	Voltage = ( adc_raw / ADC_RESOLUTION - 1u) * REF_5V;
	/* mV = V * 1000 */
	mV = Voltage * RESOLUTION_V_TO_mV;

	return mV;
}

/* -----------------------------------Exported Functions----------------------------------- */

/* ----------RTE READ---------- */
Std_ReturnType Rte_Read_Lights_PosLights (uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	uint8_t state = HAL_GPIO_ReadPin(GPIOD, Switch_POS_Pin);
	if(state == HIGH)
	{
		CarSignals.posLights = state;
	}

	*data = CarSignals.posLights;

	return Rte_Status;
}
Std_ReturnType Rte_Read_Lights_LowBeam (uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	uint8_t state = HAL_GPIO_ReadPin(GPIOD, Switch_LOWBEAM_Pin);
	if(state == HIGH)
	{
		CarSignals.lowBeam = HIGH;
	}
	if(HAL_GPIO_ReadPin(GPIOD, Switch_POS_Pin) == HIGH)
	{
		CarSignals.lowBeam = LOW;
	}
	*data = CarSignals.lowBeam;

	return Rte_Status;
}

Std_ReturnType Rte_Read_Lights_HighBeam (uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	uint8_t state = HAL_GPIO_ReadPin(GPIOD, Switch_HIGHBEAM_Pin);
	if(state == HIGH)
	{
		CarSignals.highBeam = HIGH;
	}
	if(HAL_GPIO_ReadPin(GPIOD, Switch_LOWBEAM_Pin) == HIGH)
	{
		CarSignals.highBeam = LOW;
	}
	if(HAL_GPIO_ReadPin(GPIOD, Switch_POS_Pin) == HIGH)
	{
		CarSignals.highBeam = LOW;
	}
	*data = CarSignals.highBeam;

	return Rte_Status;
}


Std_ReturnType Rte_Read_Lights_OnOffStatus (uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = HAL_GPIO_ReadPin(GPIOD, Switch_OFF_Pin);
	if(HAL_GPIO_ReadPin(GPIOD, Switch_OFF_Pin) == HIGH)
	{
		CarSignals.posLights = LOW;
		CarSignals.lowBeam = LOW;
		CarSignals.highBeam = LOW;
	}

	return Rte_Status;
}

Std_ReturnType Rte_Read_Lights_Status_Auto (uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = HAL_GPIO_ReadPin(GPIOD, Switch_AUTO_Pin);
	DataPacket_Arduino[5] = HAL_GPIO_ReadPin(GPIOD, Switch_AUTO_Pin);

	return Rte_Status;
}

Std_ReturnType Rte_Read_Door_Status(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;

	if( (DEG90 == htim2.Instance->CCR1) && (DEG90 == htim2.Instance->CCR2) )
	{
		CarSignals.doorStatus = DOOR_OPEN;
		*data = CarSignals.doorStatus;

	}
	else if( (DEG180 == htim2.Instance->CCR1) && (DEG0 == htim2.Instance->CCR2) )
	{
		CarSignals.doorStatus = DOOR_CLOSED;
		*data = CarSignals.doorStatus;
	}


	return Rte_Status;
}

Std_ReturnType Rte_Read_Fan_Status (uint8_t *data)
{

	Std_ReturnType Rte_Status = RTE_E_OK;
	DataPacket_Arduino[8] = CarSignals.fanStatus;
	*data = CarSignals.fanStatus;

	return Rte_Status;
}

Std_ReturnType Rte_Read_Engine_Status (uint8_t *data)
{

	Std_ReturnType Rte_Status = RTE_E_OK;
	DataPacket_Arduino[7] = CarSignals.engineStatus;
	*data = CarSignals.engineStatus;

	return Rte_Status;
}

Std_ReturnType Rte_Read_Temperature_Value(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	uint16_t lu8_temp = RawValues.rawTemperature;
	DataPacket_Arduino[1] = Get_Temperature(lu8_temp);
	*data = Get_Temperature(lu8_temp);

	return Rte_Status;

}

Std_ReturnType Rte_Read_PhotoSen_prcntg(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	uint16_t lu8_temp = RawValues.rawLux;

	uint8_t mV = Get_Voltage(lu8_temp);

	*data = map_values(mV, 0, 4880, 0, 100);

	return Rte_Status;
}

Std_ReturnType Rte_Read_Potentiometer_interval(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	uint16_t lu8_temp = RawValues.rawPotentiometer;
	DataPacket_Arduino[6] = map_values(lu8_temp, 55, 4095, 15, 25);
	*data = map_values(lu8_temp, 55, 4095, 15, 25);


	return Rte_Status;
}

Std_ReturnType Rte_Read_DriverPresence_value(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = CarSignals.driverPresence;

	return Rte_Status;
}

Std_ReturnType Rte_Read_CommandBuffer_Lights_status(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	CarSignals.posLights = CommandValues[0];
	CarSignals.lowBeam = CommandValues[0];
	CarSignals.highBeam = CommandValues[0];
	*data = CommandValues[0];

	return Rte_Status;
}

Std_ReturnType Rte_Read_CommandBuffer_Fan_status(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	CarSignals.fanStatus = CommandValues[2];
	*data = CommandValues[2];

	return Rte_Status;
}

Std_ReturnType Rte_Read_CommandBuffer_Door_status(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	CarSignals.doorStatus = CommandValues[1];
	*data = CommandValues[1];

	return Rte_Status;
}

Std_ReturnType Rte_Read_CarSignals_VehicleSpeed(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = CarSignals.vehicleSpeed;

	return Rte_Status;
}

Std_ReturnType Rte_Read_Lights_Status(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;

	if(CarSignals.posLights == ON || CarSignals.lowBeam == ON || CarSignals.highBeam == ON)
	{
		*data = ON;
	}
	else
	{
		*data = OFF;
	}
	return Rte_Status;
}

Std_ReturnType Rte_Read_Datapacket_Arduino_Speed(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = DataPacket_Arduino[0];
	return Rte_Status;
}

Std_ReturnType Rte_Read_Datapacket_Arduino_Temperature(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = DataPacket_Arduino[1];
	return Rte_Status;
}

Std_ReturnType Rte_Read_Datapacket_Arduino_PosLights(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = DataPacket_Arduino[2];
	return Rte_Status;
}

Std_ReturnType Rte_Read_Datapacket_Arduino_LowBeam(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = DataPacket_Arduino[3];
	return Rte_Status;
}

Std_ReturnType Rte_Read_Datapacket_Arduino_HighBeam(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = DataPacket_Arduino[4];
	return Rte_Status;
}

Std_ReturnType Rte_Read_Datapacket_Arduino_AutoMode(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = DataPacket_Arduino[5];
	return Rte_Status;
}

Std_ReturnType Rte_Read_Datapacket_Arduino_TemperatureSetpoint(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = DataPacket_Arduino[6];
	return Rte_Status;
}

Std_ReturnType Rte_Read_Datapacket_Arduino_EngineStatus(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = DataPacket_Arduino[7];
	return Rte_Status;
}

Std_ReturnType Rte_Read_Datapacket_Arduino_FanStatus(uint8_t *data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	*data = DataPacket_Arduino[8];
	return Rte_Status;
}
/* ----------RTE READ---------- */

Std_ReturnType Rte_Write_RawValues_rawTemperature(uint16_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	RawValues.rawTemperature = data;

	return Rte_Status;
}

Std_ReturnType Rte_Write_RawValues_rawLux(uint16_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	RawValues.rawLux = data;

	return Rte_Status;
}

Std_ReturnType Rte_Write_RawValues_rawPotentiometer(uint16_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	RawValues.rawPotentiometer = data;

	return Rte_Status;
}

Std_ReturnType Rte_Write_CarSignals_DriverPresence(uint8_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	CarSignals.driverPresence = data;

	return Rte_Status;
}

Std_ReturnType Rte_Write_CommandBuffer_Lights_status(uint8_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	CommandValues[0] = data;

	return Rte_Status;
}

Std_ReturnType Rte_Write_CommandBuffer_Door_status(uint8_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	CommandValues[1] = data;

	return Rte_Status;
}

Std_ReturnType Rte_Write_CommandBuffer_Fan_status(uint8_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	CommandValues[2] = data;

	return Rte_Status;
}

Std_ReturnType Rte_Write_CarSignals_VehicleSpeed(uint8_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	DataPacket_Arduino[0] = data;
	CarSignals.vehicleSpeed = data;

	return Rte_Status;
}

Std_ReturnType Rte_Write_CarSignals_FanStatus(uint8_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	CarSignals.fanStatus = data;

	return Rte_Status;
}

Std_ReturnType Rte_Write_CarSignals_EngineStatus(uint8_t data)
{
	Std_ReturnType Rte_Status = RTE_E_OK;
	CarSignals.engineStatus = data;

	return Rte_Status;
}



