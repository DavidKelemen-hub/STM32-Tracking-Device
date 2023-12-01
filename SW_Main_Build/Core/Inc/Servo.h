/*
 * Servo.h
 *
 *  Created on: May 15, 2023
 *      Author: K. David
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#define MINPULSE 500
#define MAXPULSE 2100
#define MINPERCENTAGE 0
#define MAXPERCENTAGE 100
#define MINDEG (-90)
#define MAXDEG (90)
#define POSITION_OPEN_BOTH_DOORS 1200u
#define POSITION_CLOSED_DOOR_LEFT 500u
#define POSITION_CLOSED_DOOR_RIGHT 2100u


void Servo_Start_Init();
void Servo_Start_TaskCyclicEvent();

#endif /* SRC_SERVO_H_ */
