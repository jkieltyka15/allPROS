/*
 * @file robot.h
 *
 * @brief Collection of files that make up VEX ALL PROSe.
 * Robot data structure and prototypes.
 * Creation of motors to be used by robots.
 * LCD Menus with autonomous selection.
 * This is stricly a BETA release and is not the final version.
 *
 * Copyright (C) 2016  Jordan M. Kieltyka
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <NDAPI.h>	//NDA API

//alliances
#define RED_ALLIANCE  1	//red alliance
#define BLUE_ALLIANCE 2	//blue alliance

//starting tiles
#define POS_1 1	//first position
#define POS_2 2	//second position

//robot modes
#define SKILLS 0
#define COMPETITION 1

//controller type
#define DRIVER  1	//the main driver controller
#define PARTNER 2	//the partner driver controller

//robot data structure
struct{
	char alliance;		//the robot's alliance
	char startPos;		//the robot's starting position
	bool skills;			//flag for if the robot is in a skills challenge or not
	bool record;			//flag for if the robot is in the recording state or not
	LCD lcd;					//the robot's LCD screen
	int liftPos;			//the robot's current lift position
	double liftConst;	//the robot's lift constant for PID, default is 0.7

	//motor systems
	MotorSystem rightDrive;		//robot's right drive
	MotorSystem leftDrive;		//robot's left drive
	MotorSystem intake;				//robot's intake motor system
	MotorSystem lift;					//robot's lift motor system

	//sensors
	Sensor rightDriveSensor; 	//robot's right drive
	Sensor leftDriveSensor;		//robot's left drive sensor
	Sensor liftSensor;				//robot's lift sensor
	Sensor turnSensor;				//robot's turn sensor
} Robot;

/* Generic robot functions */

void robot_init();	//initialize the robot

//getter methods
char robot_getAlliance();			//retrieve the robot's alliance
char robot_getStartPos();			//retrieve the robot's starting position
bool robot_getSkills();				//retrieve the robot's skills state
int robot_getLiftPos();				//retrieve the robot's lift position
bool robot_isRecording();			//determine if the robot is in recording mode
double robot_getLiftConst();	//get the PID lift constant value

//lcd methods
void robot_lcdMenu();	//lcd selection menu

//drive methods
void robot_joyDrive(unsigned char controller);													//control robot's drive via the vexNET joystick
void robot_setDrive(char velocity);																			//set both the right and left drives to the same motor velocities
void robot_setDriveSplit(char left, char right);												//set the robot's drive velocities independently
void robot_stop();																											//set the velocity of the drive to zero
void robot_setDriveFor(char velocity, unsigned int time);								//run drive for a certain amount of time at a certain velocity
void robot_setDriveForSplit(char left, char right, unsigned int time);	//run drive for a certain amount of time independently

//lift methods
void robot_liftToPosition(int pos);		//go to the specified position
void robot_setLiftConst(double val);	//set the PID lift constant value

//intake methods
void robot_intakeIn();		//set the robot's intake to in
void robot_intakeOut();		//set the robot's intake to out
void robot_intakeStop();	//stop the robot's intake

//free memmory
void robot_free();	//free the data associated with robot

#endif /* ROBOT_H_ */
