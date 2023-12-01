/*
 * Task_CyclicEvent.c
 *
 *  Created on: May 15, 2023
 *      Author: K. David
 */

#include "Rte.h"
#include "GSM_Transmission.h"
#include "Servo.h"
#include "Lights_Module.h"
#include "Inputs.h"
#include "FanControl.h"
#include "Presence_Detection.h"
#include "Transmit_Arduino.h"
#include "main.h"


void Start_Init()
{
	GSM_Start_Init();
	Presence_Detection_Init();
	Servo_Start_Init();
	Lights_Module_Init();
	Handle_Inputs_Init();
	FanControl_Init();
	Transmit_Arduino_Init();
}

void Start_TaskCyclicEvent()
{
	GSM_Start_TaskCyclicEvent();
	Presence_Detection_TaskCyclicEvent();
	Servo_Start_TaskCyclicEvent();
	Handle_Inputs_TaskCyclicEvent();
	Lights_Module_Start_TaskCyclicEvent();
	FanControl_TaskCyclicEvent();
	Transmit_Arduino_TaskCyclicEvent();
}



