/*
 * climber.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: RTR
 */

#include <iostream>
#include <string>
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SpeedController.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
//#include "climber.h"
#include <iostream>
#include <Encoder.h>
#include <climber.h>

ClimberManager::ClimberManager() {
//	srx1 = new WPI_TalonSRX(9);

//	this->stick = new Joystick { 0 };
//	xbox = new XboxController { 1 };
}

void ClimberManager::Climber() {
//if (xbox->GetRawButton(6)) {
//	this->srx1->Set(0.5);
//	}
//else {
//	this->srx1->Set(0);
//	}
	frc::SmartDashboard::PutNumber("climber",0);
}




