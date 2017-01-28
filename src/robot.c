/*
 * @file robot.c
 *
 * @brief Implementation of robot methods.
 *        LCD Menus with autonomous selection.
 *        This is strictly a BETA release and is not the final version.
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

#include <robot.h>
#include <main.h>

/*
 * Initialize the robot.
 */
void robot_init(){
	Robot.liftConst = 0.7;		//set default value for PID lift constant
}

/*
 * Retrieve the robot's alliance.
 *
 * @return Robot's alliance.
 */
char robot_getAlliance(){
	return Robot.alliance;
}

/*
 * Retrieve the robot's starting position.
 *
 * @return Robot's starting position.
 */
char robot_getStartPos(){
	return Robot.startPos;
}

/*
 * Retrieve the robot's skills state
 *
 * @return Robot's starting position.
 */
bool robot_getSkills(){
	return Robot.skills;
}

/*
 * Retrieve the robot's current lift position.
 *
 * @return The robot's current lift position.
 */
int robot_getLiftPos(){
	return Robot.liftPos;
}

/*
 * Retrieve if the robot is currently in record mode.
 *
 * @return If the robot is in record mode or not.
 */
bool robot_isRecording(){
	return Robot.record;
}

/*
 * Retrieve the robot's PID lift constant value.
 *
 * @return Robot's PID lift constant value.
 */
double robot_getLiftConst(){
	return Robot.liftConst;
}

/*
 * Display battery voltages.
 * Alliance color selection menu.
 * Starting positions selection menu.
 */
void robot_lcdMenu(){

	lcd_backLight(&Robot.lcd, ON);	//turn on the robot's lcd backlight
	lcd_clear(&Robot.lcd);					//clear lcd

	//splash-screen
	lcd_centerPrint(&Robot.lcd, TOP, "Property of:");					//print lcd
	lcd_centerPrint(&Robot.lcd, BOTTOM, "NDA ROBOTICS");			//print lcd
	delay(1500);																							//delay to make lcd readable
	lcd_clear(&Robot.lcd);																		//clear lcd

	//display low battery warning until battery is replace
	while(powerLevelMain() <= 6000){
		lcd_centerPrint(&Robot.lcd, TOP, "WARNING!!!");						//print lcd warning
		lcd_centerPrint(&Robot.lcd, BOTTOM, "Battery Critical");	//print lcd warning
		lcd_backLight(&Robot.lcd, !lcd_backLightIsOn(Robot.lcd));	//toggle the robot's lcd backlight
		delay(250);																								//delay to make strobe visible
	}

	lcd_backLight(&Robot.lcd, ON);	//turn on the robot's lcd backlight
	lcd_clear(&Robot.lcd);					//clear lcd
	lcd_waitForRelease(Robot.lcd);	//wait for the button to be released before proceeding

	//select a mode
	mode:
	Robot.record = false;		//set the default record to false

	while(!robot_isRecording() && !lcd_buttonIsPressed(Robot.lcd, LCD_BTN_RIGHT)){
		lcd_centerPrint(&Robot.lcd, TOP, "Select Mode");		//print lcd prompt
		lcd_print(&Robot.lcd, BOTTOM, "REC         COMP");	//print lcd prompt

		//recording selected
		if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_LEFT))
			Robot.record = true;

		//competition selected
		else if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_RIGHT))
			Robot.record = false;
	}

	lcd_waitForRelease(Robot.lcd);	//wait for the button to be released before proceeding

	//select a mode
	skills:
	Robot.skills = false;						//set the defualt skills to false

	while(!robot_getSkills() && !lcd_buttonIsPressed(Robot.lcd, LCD_BTN_RIGHT)){
		lcd_centerPrint(&Robot.lcd, TOP, "Select Mode");		//print lcd prompt
		lcd_print(&Robot.lcd, BOTTOM, "SKILLS x    COMP");	//print lcd prompt

		//competition selected
		if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_LEFT))
			Robot.skills = true;

		//skills selected
		else if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_RIGHT))
			Robot.skills = false;

		//go back a step in the menu
		else if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_CENTER)){
			lcd_waitForRelease(Robot.lcd);	//wait for the button to be released before proceeding
			goto mode;											//go back to mode section
		}
	}

	lcd_waitForRelease(Robot.lcd);	//wait for the button to be released before proceeding

	//select competition settings
	if(!robot_getSkills()){

		//select an alliance
		alliance:
		Robot.alliance = NULL;	//set default alliance to null

		while(robot_getAlliance() == NULL){
			lcd_centerPrint(&Robot.lcd, TOP, "Select Alliance");	//print lcd prompt
			lcd_print(&Robot.lcd, BOTTOM, "RED    x    BLUE");		//print lcd prompt

			//red alliance selected
			if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_LEFT))
				Robot.alliance = RED_ALLIANCE;

			//blue alliance selected
			else if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_RIGHT))
				Robot.alliance = BLUE_ALLIANCE;

			//go back a step in the menu
			else if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_CENTER)){
				lcd_waitForRelease(Robot.lcd);		//wait for the button to be released before proceeding
				goto skills;											//go back to the skills section
			}
		}

		lcd_waitForRelease(Robot.lcd);	//wait for the button to be released before proceeding
		Robot.startPos = NULL;					//set default starting position to null

		//select a starting position
		while(robot_getStartPos() == NULL){
			lcd_centerPrint(&Robot.lcd, TOP, "Select Position");	//print lcd prompt
			lcd_print(&Robot.lcd, BOTTOM, "POS 1  x   POS 2");		//print lcd prompt

			//red alliance selected
			if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_LEFT))
				Robot.startPos = POS_1;

			//blue alliance selected
			else if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_RIGHT))
				Robot.startPos = POS_2;

			//go back a step in the menu
			else if(lcd_buttonIsPressed(Robot.lcd, LCD_BTN_CENTER)){
				lcd_waitForRelease(Robot.lcd);	//wait for the button to be released before proceeding
				goto alliance;									//go back to the alliance section
			}
		}
		lcd_waitForRelease(Robot.lcd);	//wait for the button to be released before proceeding
	}
	lcd_clear(&Robot.lcd);	//clear the lcd screen
}

/*
 *	Control robot's drive via the vexNET joystick
 *
 *	@param controller The joystick that will be controlling the drive.
 */
void robot_joyDrive(unsigned char controller){

	//used for dead zoning joystick
	if(abs(joystickGetAnalog(controller, 2)) > 10)
		motorSystem_setVelocity(&Robot.rightDrive, joystickGetAnalog(controller, 2));	//set robot's right drive velocity
	else
		motorSystem_stop(&Robot.rightDrive);	//stop robot's right drive

	//used for dead zoning joystick
	if(abs(joystickGetAnalog(controller, 3)) > 10)
		motorSystem_setVelocity(&Robot.leftDrive, joystickGetAnalog(controller, 3));	//set robot's left drive velocity
	else
		motorSystem_stop(&Robot.leftDrive);	//stop robot's left drive
}

/*
 *	Set both of the robot's drives to the desired
 *	velocity.
 *
 *	@param velocity The desired velocity for the left and right drive.
 */
void robot_setDrive(char velocity){
	robot_setDriveSplit(velocity, velocity);	//set the right and left drive to the desired velocity
}

/*
 * Set the desired velocity for the left and right drive.
 *
 * @param left The desired velocity for the left drive.
 * @param right The desired velocity for the right drive.
 */
void robot_setDriveSplit(char left, char right){
	motorSystem_setVelocity(&Robot.leftDrive, left);		//set the left drive velocity
	motorSystem_setVelocity(&Robot.rightDrive, right);	//set the right drive velocity
}

/*
 * Set the robot's drive train velocities to zero.
 */
void robot_stop(){
	robot_setDrive(0);	//set both right and left drive to zero
}

/*
 * Set the robot's drives for a desired amount of time then stop.
 *
 * @param velocity The desired drive velocity.
 * @param time The amount of time in ms for the drive to run.
 */
void robot_setDriveFor(char velocity, unsigned int time){
	robot_setDriveForSplit(velocity, velocity, time);
}

/*
 * Set the robot's drives independently for a desired amount of time
 * then stop.
 *
 * @param left The desired velocity of the left drive.
 * @param right The desired velocity of the right drive.
 * @param time The amount of time in ms for the drives to run.
 */
void robot_setDriveForSplit(char left, char right, unsigned int time){
	robot_setDriveSplit(left, right);	//set the right and left drive to the desired velocities
	delay(time);											//pause for the desired amount of time
	robot_stop();											//stop the drive
}

/*
 * Have the robot's lift go to the desired position
 *
 * @param pos The desired lift position.
 */
void robot_liftToPosition(int pos){

	//it is the autonomous period
	if(isAutonomous())
		motorSystem_setTillPID(&Robot.lift, &Robot.liftSensor, robot_getLiftConst(), pos);

	//it is op control period
	else{
		//update motor in PID loop until sensor target value is near
		if(abs(sensor_getValue(Robot.liftSensor)) != pos || (pos - sensor_getValue(Robot.liftSensor)) * robot_getLiftConst() < 10)
			motorSystem_setVelocity(&Robot.lift, (pos - sensor_getValue(Robot.liftSensor)));

		//target position has been reached
		else
			motorSystem_stop(&Robot.lift);	//stop motor
	}
}

/**
 * Set the PID lift constant value for the
 * desired lift behavior.
 *
 * @param val The new value for the lift constant.
 */
void robot_setLiftConst(double val){
	Robot.liftConst = val;
}

/*
 * Set the robot's intake to the on state.
 */
void robot_intakeIn(){
	motorSystem_setVelocity(&Robot.intake, 127);
}

/*
 * Set the robot's intake to the out state.
 */
void robot_intakeOut(){
	motorSystem_setVelocity(&Robot.intake, -127);
}

/*
 * Turn off the robot's intake.
 */
void robot_intakeStop(){
	motorSystem_stop(&Robot.intake);
}

/*
 * Free all sensors, sensor systems and motor systems that are associated
 * with the robot.
 */
void robot_free(){

	//free motor systems
	motorSystem_free(&Robot.rightDrive);	//free the right drive
	motorSystem_free(&Robot.leftDrive);		//free the left drive
	motorSystem_free(&Robot.intake);			//free the intake
	motorSystem_free(&Robot.lift);				//free the lift

	//free sensors
	sensor_free(&Robot.rightDriveSensor);	//free the right drive sensor
	sensor_free(&Robot.leftDriveSensor);	//free the left drive sensor
	sensor_free(&Robot.liftSensor);				//free the lift sensor
	sensor_free(&Robot.turnSensor);				//free the turn sensor
}
