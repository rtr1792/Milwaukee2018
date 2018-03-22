/*
 * autonomous.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Scouting1792-PC
 */
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <Talon.h>
#include "autonomous.h"
#include "math.h"
#include <Timer.h>
#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SpeedController.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
#include <Encoder.h>
#include "AHRS.h"
#include <SPI.h>
#include <Robot.h>

AutoManager::AutoManager(LiftManager *lift, DriveManager *drive, IntakeManager *intake) {
	this->liftManager = lift;
	this->driveManager = drive;
	this->intakeManager = intake;
}
//SwitchRight
void AutoManager::SwitchRight() { //starts on right scores on right switch
	//Switch Right
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1 : this->driveManager->Drive(FirstDriveSpeed, ToSwitch);  //find dist to switch
				this->intakeManager->Intakemove(0, true);
			break;
		case 2 : this->driveManager->Turn(-90); // Negative to Turn Left
				this->intakeManager->Intakemove(0, true);
			break;
		case 3: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 4: this->driveManager->Drive(FirstDriveSpeed, NexttoSwitchForward); //find dist to switch from pt
				this->intakeManager->Intakemove(0, true);
			break;
		case 5: this->intakeManager->Intakemove(-1, false);
			break;
	}
}

//Switch Left
void AutoManager::SwitchLeft() { //stats on left scores on left switch
	//Switch Left
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1 : this->driveManager->Drive(FirstDriveSpeed, ToSwitch);  //find dist to switch
				this->intakeManager->Intakemove(0, true);
			break;
		case 2 : this->driveManager->Turn(90); // Positive to turn Right
				this->intakeManager->Intakemove(0, true);
			break;
		case 3: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 4: this->driveManager->Drive(FirstDriveSpeed, NexttoSwitchForward); //find dist to switch from pt
				this->intakeManager->Intakemove(0, true);
			break;
		case 5: this->intakeManager->Intakemove(-1, false);
			break;
	}
}
//Score on the Right Side of the Switch
void AutoManager::CenterRight(){ //starts at 2 and sccores on right side of switch
	//Switch Center
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);                  // do things from same thing above but reverse also DIFFERENT ANGLES!!!!!!!!!!!!!
			break;
		case 1 : this->driveManager->Drive(0.75, SwitchCenterStraight); // negative to Turn left
				this->intakeManager->Intakemove(0, true);
			break;
		case 2: this->intakeManager->Intakemove(-1, false);
			break;
	}
}
//Score on the Left Side of the Switch
void AutoManager::CenterLeft(){ //starts at 2 and scores on left side of switch
	//Switch Center
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1: this->driveManager->Drive(0.75, 20);
				this->intakeManager->Intakemove(0, true);
			break;
		case 2: this->driveManager->Turn(-90); // Postive to Turn Right
				this->intakeManager->Intakemove(0, true);
			break;
		case 3: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 4 : this->driveManager->Drive(0.75, 78);
				 this->intakeManager->Intakemove(0, true);
			break;
		case 5: this->driveManager->Turn(90);
				this->intakeManager->Intakemove(0, true);
			break;
		case 6: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 7: this->driveManager->Drive(0.75, 65);
				this->intakeManager->Intakemove(0, true);
			break;
		case 8: this->intakeManager->Intakemove(-1, false);
			break;
	}
}

void AutoManager::ScaleRight(){ // starts at 3 and scores on right side of scale
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1: this->driveManager->Drive(FirstDriveSpeed, 233);
				this->intakeManager->Intakemove(0, true);
			break;
		case 2: this->driveManager->Turn(-45);
				this->intakeManager->Intakemove(0, true);
			break;
		case 3: this->liftManager->Liftmove(scaleheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 4: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 5: this->driveManager->Drive(0.40, 16);
				this->intakeManager->Intakemove(0, true);
			break;
		case 6: this->intakeManager->Intakemove(-1, false);
			break;
		case 7: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 8: this->driveManager->Turn(-50);
			break;
		case 9: this->liftManager->Liftmove(0, scaleheight, 0);
			break;
	}
}

void AutoManager::ScaleLeft(){ // starts at 1 and scores on left side of scale
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1: this->driveManager->Drive(FirstDriveSpeed, 233);
				this->intakeManager->Intakemove(0, true);
			break;
		case 2: this->driveManager->Turn(45);
				this->intakeManager->Intakemove(0, true);
			break;
		case 3: this->liftManager->Liftmove(scaleheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 4: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 5: this->driveManager->Drive(0.40, 16);
				this->intakeManager->Intakemove(0, true);
			break;
		case 6: this->intakeManager->Intakemove(-1, false);
			break;
		case 7: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 8: this->driveManager->Turn(50);
			break;
		case 9: this->liftManager->Liftmove(0, scaleheight, 0);
			break;
	}
}

//Straight Line Drive Auto Line
void AutoManager::StraightLine() {
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1 : this->driveManager->Drive(FirstDriveSpeed, Autoline);
				this->intakeManager->Intakemove(0, true);
			break;
		case 2: this->driveManager->Drive(0, 10000);
			break;
	}
}


void AutoManager::crossScoreLeft() { //starting at 3 and scoring on the left side of the scale
	switch(autostep) {
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1: this->driveManager->Drive(FirstDriveSpeed, 174);
				this->intakeManager->Intakemove(0, true);
			break;
		case 2: this->driveManager->Turn(-90);
				this->intakeManager->Intakemove(0, true);
			break;
		case 3: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 4: this->driveManager->Drive(FirstDriveSpeed, 180);
				this->intakeManager->Intakemove(0, true);
			break;
		case 5: this->driveManager->Turn(135);
				this->intakeManager->Intakemove(0, true);
			break;
		case 6: this->liftManager->Liftmove(scaleheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 7: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 8: this->driveManager->Drive(0.40, 16);
				this->intakeManager->Intakemove(0, true);
			break;
		case 9: this->intakeManager->Intakemove(-1, false);
			break;
	}
}

void AutoManager::crossScoreRight() { //starting at 1 scoring on the right side of the scale
	switch(autostep) {
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1: this->driveManager->Drive(FirstDriveSpeed, 174);
				this->intakeManager->Intakemove(0, true);
			break;
		case 2: this->driveManager->Turn(90);
				this->intakeManager->Intakemove(0, true);
			break;
		case 3: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 4: this->driveManager->Drive(FirstDriveSpeed, 180);
				this->intakeManager->Intakemove(0, true);
			break;
		case 5: this->driveManager->Turn(-135);
				this->intakeManager->Intakemove(0, true);
			break;
		case 6: this->liftManager->Liftmove(scaleheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 7: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 8: this->driveManager->Drive(0.40, 16);
				this->intakeManager->Intakemove(0, true);
			break;
		case 9: this->intakeManager->Intakemove(-1, false);
			break;
	}
}



