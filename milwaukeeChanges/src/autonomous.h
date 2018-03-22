/*
 * autonomous.h
 *
 *  Created on: Feb 6, 2018
 *      Author: Scouting1792-PC
 */
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <Talon.h>
#include <Timer.h>
#include "lift.h"
#include "drive.h"
#include "intake.h"

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

class AutoManager {
private:
	LiftManager *liftManager;
	DriveManager *driveManager;
	IntakeManager *intakeManager;

public:
	AutoManager(LiftManager *lift, DriveManager *drive, IntakeManager *intake);

	//Probably Good Autos
	void SwitchRight(); //Switch is to the Right of the Robot to Score
	void SwitchLeft();	//Switch is to the Left of the Robot to Score
	void CenterRight(); //Diddo as above
	void CenterLeft(); //Diddo as above
	void StraightLine(); //Just Drive
	void ScaleRight();  // Scale is on the Right side to Score
	void ScaleLeft();  // Scale is on the Left side to Score
	void crossScoreLeft();
	void crossScoreRight();
};



#endif /* SRC_AUTONOMOUS_H_ */
