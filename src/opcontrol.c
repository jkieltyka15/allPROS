/** @file opcontrol.c\
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Modified on: 2016
 *      Author: Jordan Kieltyka
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
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {

	lcd_centerPrint(&Robot.lcd, TOP, "Driver");				//print to lcd
	lcd_centerPrint(&Robot.lcd, BOTTOM, "Control Mode");	//print to lcd

	//continue to loop until competition is ended
	while(!robot_isRecording()){
		userControl();
		delay(20);
	}

	//do record sequence for skills challenge (60 seconds)
	if(robot_getSkills())
		robot_record("sk.txt", 60000);

	//do record sequence for red alliance (15 seconds)
	else if(robot_getAlliance() == RED_ALLIANCE){
		if(robot_getStartPos() == POS_1)
			robot_record("r1.txt", 15000);	//record autonomous at position 1
		else
			robot_record("r2.txt", 15000);	//record autonomous at position 2
	}

	//do record sequence for blue alliance (15 seconds)
	else if(robot_getAlliance() == BLUE_ALLIANCE){
		if(robot_getStartPos() == POS_1)
			robot_record("b1.txt", 15000);	//record autonomous at position 1
		else
			robot_record("b2.txt", 15000);	//record autonomous at position 2
	}

	lcd_centerPrint(&Robot.lcd, TOP, "Rebooting");	//print to lcd
	lcd_centerPrint(&Robot.lcd, BOTTOM, "System");	//print to lcd
	delay(1000);																		//delay to read LCD message
	robot_free();																		//free memmory from the robot
	exit(EXIT_SUCCESS);															//restart program with exit code EXIT_SUCCESS
}
