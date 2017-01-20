/** @file init.c
 * @brief File for initialization code
 *
 * This file should contain the user initialize() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 *  Modified on: 2016
 *       Author: Jordan Kieltyka
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO() {

}

/*
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */
void initialize() {

	//create motors for every port
	Motor m1 = motor_init(PORT_1, false);  //create motor for port 1
	Motor m2 = motor_init(PORT_2, true); 	 //create motor for port 2
	Motor m3 = motor_init(PORT_3, true); 	 //create motor for port 3
	Motor m4 = motor_init(PORT_4, true); 	 //create motor for port 4
	Motor m5 = motor_init(PORT_5, false);  //create motor for port 5
	Motor m6 = motor_init(PORT_6, true); 	 //create motor for port 6
	Motor m7 = motor_init(PORT_7, false);  //create motor for port 7
	Motor m8 = motor_init(PORT_8, false);  //create motor for port 8
	Motor m9 = motor_init(PORT_9, false);  //create motor for port 9
	Motor m10 = motor_init(PORT_10, true); //create motor for port 10

	//initialize the robot
	robot_init();							//initialize the robot
	robot_setLiftConst(0.7);	//set the lift constant

	//initialize motor systems
	Robot.rightDrive = motorSystem_init(3, &m7, &m8, &m9); //initialize right drive
	Robot.rightDrive = motorSystem_init(3, &m2, &m3, &m4); //initialize left drive
	Robot.lift = motorSystem_init(2, &m5, &m6);						 //initialize lift
	Robot.intake = motorSystem_init(2, &m1, &m10);				 //initialize claw

	//initialize sensors
	Robot.liftSensor = sensor_init(IME, I2C_1);	//set lift sensor to I2C 1

	//LCD
	Robot.lcd = lcd_init(uart2);    //setup the robot's lcd
	robot_lcdMenu();                //begin robot start up menu
}
