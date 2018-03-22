/*
 * intake.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: RTR
 */

//xbox buttons 1,2,3,4,6

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
#include <iostream>
#include <Encoder.h>
//#include <segments/intake.h>
#include "intake.h"
#include <Ultrasonic.h>
#include <Robot.h>

IntakeManager::IntakeManager() {
	srx2 = new WPI_TalonSRX(7);
	srx1 = new WPI_TalonSRX(8);

	this->stick = new Joystick { 0 };
	this->xbox = new XboxController { 1 };
//output,input
	ult = new Ultrasonic(2,3);
	ult->SetAutomaticMode(true);
	ult->SetEnabled(true);

	//left
	ult2 = new Ultrasonic(6,7);
	ult2->SetAutomaticMode(true);
	ult2->SetEnabled(true);

	rd = new int;
	ld = new int;
	reverse = new int;
}

void IntakeManager::Intake() {

	frc::SmartDashboard::PutNumber("test",1);

	if (xbox->GetRawButton(2)) { //outake slow
		srx1->Set(-0.5);
		srx2->Set(0.5);
	}
	else if (xbox->GetRawButton(3)) { //outake fast
		srx1->Set(-1);
		srx2->Set(1);
	}
	else if (xbox->GetRawButton(1)) { //intake
		srx1->Set(0.75);
		srx2->Set(-0.75);
	}
	else{
		srx1->Set(0);
		srx2->Set(0);
	}

/*
	if ((*ld == *one) and (*reverse == *zero) and xbox->GetRawButton(1)) {
		srx1->Set(0.5);
		srx2->Set(0.3);
	}
	else if ((*rd == *one) and (*reverse == *zero) and xbox->GetRawButton(1)) {
		srx1->Set(0.3);
		srx2->Set(0.5);
	} */

	//gets values for ultrasonics
	double ultd = ult->GetRangeInches();
	frc::SmartDashboard::PutNumber("In",ultd);
	double ultd2 = ult2->GetRangeInches();
	frc::SmartDashboard::PutNumber("In2",ultd2);

	//ultrasonic p loop to keep the box in
	double k = 0.07;
	if(!xbox->GetRawButton(6)){
		if ((!xbox->GetRawButton(1) and !xbox->GetRawButton(2)) and !xbox->GetRawButton(3)) {
			if ((ultd > ultramin and ultd < ultramax)) {
				srx1->Set(ultd * k);
			}

			if ((ultd2 > ultramin and ultd2 < ultramax)) {
				srx2->Set(-(ultd2 * k));
			}
		}
	}

	frc::SmartDashboard::PutNumber("intakeC",srx1->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("intakeC2",srx2->GetOutputCurrent());
}

void IntakeManager::Intakemove(double speed, bool ultraenable) {
	srx1->Set(speed);
	srx2->Set(-speed);
	double k = 0.07;
	double ultd = ult->GetRangeInches();
	frc::SmartDashboard::PutNumber("In",ultd);
	double ultd2 = ult2->GetRangeInches();
	frc::SmartDashboard::PutNumber("In2",ultd2);
	if(ultraenable){  // Ultrasonic tells the intake to always run per Ultraenable
		if ((ultd > ultramin and ultd < ultramax)) {
			srx1->Set(ultd * k);
		}

		if ((ultd2 > ultramin and ultd2 < ultramax)) {
			srx2->Set(-(ultd2 * k));
		}
	}
	if(ultd > 16 && ultd2 > 16){
		autostep++;
		srx1->Set(0);
		srx2->Set(0);
	}
}


