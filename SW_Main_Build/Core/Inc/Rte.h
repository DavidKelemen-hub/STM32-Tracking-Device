/*
 * Rte.h
 *
 *  Created on: May 15, 2023
 *      Author: K. David
 */

#ifndef SRC_RTE_H_
#define SRC_RTE_H_
#include "stdio.h"

typedef uint8_t Std_ReturnType;

/* -----------------------------------Exported functions----------------------------------- */
Std_ReturnType Rte_Read_Lights_PosLights (uint8_t *data);
Std_ReturnType Rte_Read_Lights_LowBeam (uint8_t *data);
Std_ReturnType Rte_Read_Lights_HighBeam (uint8_t *data);
Std_ReturnType Rte_Read_Lights_OnOffStatus (uint8_t *data);
Std_ReturnType Rte_Read_Lights_Status_Auto (uint8_t *data);
Std_ReturnType Rte_Read_Door_Status (uint8_t *data);
Std_ReturnType Rte_Read_Fan_Status (uint8_t *data);
Std_ReturnType Rte_Read_Engine_Status (uint8_t *data);
Std_ReturnType Rte_Read_Temperature_Value(uint8_t *data);
Std_ReturnType Rte_Read_PhotoSen_prcntg(uint8_t *data);
Std_ReturnType Rte_Read_Potentiometer_interval(uint8_t *data);
Std_ReturnType Rte_Write_RawValues_rawTemperature(uint16_t data);
Std_ReturnType Rte_Write_RawValues_rawLux(uint16_t data);
Std_ReturnType Rte_Write_RawValues_rawPotentiometer(uint16_t data);
uint16_t map_values(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

#endif /* SRC_RTE_H_ */


