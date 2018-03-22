#include <iostream>
#include <string>
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <climber.h>
#include <drive.h>
#include <intake.h>
#include <lift.h>
#include <autonomous.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
#include <PowerDistributionPanel.h>
#include <Robot.h>
#include "Timer.h"


int autostep = 0;
std::string gameData;
int autonum; // What auto we are running
int location; // Where the robot is starting
double autodelay = 0;
double timersec = 0;

class Robot : public frc::IterativeRobot {

public:
	Robot() {
	this->driveManager = new DriveManager();
	this->climberManager = new ClimberManager();
	this->liftManager = new LiftManager();
	this->intakeManager = new IntakeManager();
	this->autoManager = new AutoManager(this->liftManager, this->driveManager, this->intakeManager);

	timer = new Timer();
	}
private:
	DriveManager *driveManager;
	ClimberManager *climberManager;
	LiftManager *liftManager;
	IntakeManager *intakeManager;
	AutoManager *autoManager;
	Timer *timer;

	frc::Joystick stick { 0 };

//	frc::XboxController xbox { 1 };
	//double test;
//	frc::PowerDistributionPanel pdp;





	void RobotInit() {  //init the robot
	}


	void AutonomousInit() override {
		timer->Start();
		this->driveManager->ResetSensors();
		this->driveManager->setCoast();
		//this->liftManager->Lift(scaleheight, switchheight, driveheight); //Setup Talon PID Loop Just incase and to Ensure everything is new
		autostep = 0;
		//frc::SmartDashboard::PutNumber("AutoDelay", 0);
		//frc::SmartDashboard::PutNumber("AutoNumber", 0);
		//frc::SmartDashboard::PutNumber("AutoLocation", 0);
	}

	void AutonomousPeriodic() {
		autodelay = frc::SmartDashboard::GetNumber("AutoDelay", 0);
		autonum = frc::SmartDashboard::GetNumber("AutoNumber", 0);
		location = frc::SmartDashboard::GetNumber("AutoLocation", 0);
		frc::SmartDashboard::PutNumber("AutoNumberResult",autonum);
		frc::SmartDashboard::PutNumber("locationResult", location);

		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		frc::SmartDashboard::PutString("Field Data", gameData);

		//this->driveManager->Drive(0.75, ToSwitch);
		frc::SmartDashboard::PutNumber("AutoStep", autostep);
		if(autodelay <= 0){
			timer->Stop();
			timer->Reset();
		}
		timersec = timer->Get();
		if(timersec >= autodelay || autodelay == 0){
			if(autonum == 8){
				this->autoManager->StraightLine();
			}
			if(autonum == 2 && location == 3){
				if(gameData[1] == 'R'){ 	//Look only for second character - Hope for right
					this->autoManager->ScaleRight();
				}
				else {
					this->autoManager->StraightLine();
				}
			}
			if(autonum == 2 && location == 1){
				if(gameData[1] == 'L'){		// Look only for second character - Hope for left
					this->autoManager->ScaleLeft();
				}
				else {
					this->autoManager->StraightLine();
				}
			}
			if(location == 3 && autonum == 1){
				if(gameData[0] == 'R'){		//look for first character only - Hope for Right
					this->autoManager->SwitchRight();
				}
				else{
					this->autoManager->StraightLine();
				}
			}
			if(location == 1 && autonum == 1){
				if(gameData[0] == 'L'){		//look for first character only - Hope for Left
					this->autoManager->SwitchLeft(); //Score on the Left when starting on the Left
				}
				else{
					this->autoManager->StraightLine(); //Just Drive Straight
				}
			}
			if(location == 2 && autonum == 1){
				if(gameData[0] == 'L'){		//look for first character only
					this->autoManager->CenterLeft(); //Score on the Left when starting Center
					//this->autoManager->StraightLine();
				}
				else{//Not Left Do Right
					this->autoManager->CenterRight(); // Score on the Right when starting Center
				}
			}
			if(location == 1 && autonum == 3) { 	 //robot pritorizes switch but goes to scale if we don't own the switch
				if(gameData[0] == 'L') {
					this->autoManager->SwitchLeft();
				}
				else if (gameData[1] == 'L' && gameData[0] == 'R') {
					this->autoManager->ScaleLeft();
				}
				else if (gameData[0] == 'R' && gameData[1] == 'R') {
					this->autoManager->StraightLine();
				}
			}
			if(location == 1 && autonum == 4) { 	//robot pritorizes scale but goes to switch if we don't own the scale
				if(gameData[1] == 'L') {
					this->autoManager->ScaleLeft();
				}
				else if (gameData[0] == 'L' && gameData[1] == 'R') {
					this->autoManager->SwitchLeft();
				}
				else if (gameData[0] == 'R' && gameData[1] == 'R') {
					this->autoManager->StraightLine();
				}
			}
			if(location == 3 && autonum == 3) { 	//robot pritorizes switch but goes to scale if we don't own the switch
				if(gameData[0] == 'R') {
					this->autoManager->SwitchRight();
				}
				else if (gameData[1] == 'R' && gameData[0] == 'L') {
					this->autoManager->ScaleRight();
				}
				else if (gameData[0] == 'L' && gameData[1] == 'L') {
					this->autoManager->StraightLine();
				}
			}
			if(location == 3 && autonum == 4) { 	//robot pritorizes scale but goes to switch if we don't own the scale
				if(gameData[1] == 'R') {
					this->autoManager->ScaleRight();
				}
				else if (gameData[0] == 'R' && gameData[1] == 'L') {
					this->autoManager->SwitchRight();
				}
				else if (gameData[0] == 'L' && gameData[1] == 'L') {
					this->autoManager->StraightLine();
				}
			}
			if (location == 3 and autonum == 6) { // starts on right and scores on our side of the scale
				if (gameData[1] == 'R') {
					this->autoManager->ScaleRight();
				}
				else {
					this->autoManager->crossScoreLeft();
				}
			}
			if (location == 1 and autonum == 6) { // starts on left and scores on our side of the scale
				if (gameData[1] == 'L') {
					this->autoManager->ScaleLeft();
				}
				else {
					this->autoManager->crossScoreRight();
				}
			}
		}
	}

	void TeleopInit() {
		this->driveManager->ResetSensors();

		this->driveManager->setCoast();
	}

	void TeleopPeriodic() {
		//test = frc::SmartDashboard::GetNumber("batterySet", 0);
		//frc::SmartDashboard::PutNumber("battery#",test);
	//	frc::SmartDashboard::PutNumber("battery voltage",pdp.GetVoltage());


		this->driveManager->driveTrain();


		this->climberManager->Climber();


		this->intakeManager->Intake();


		this->liftManager->Lift(scaleheight, switchheight, driveheight);

		frc::SmartDashboard::PutNumber("Team Number", testglobal);
/*		bool x1 = xbox.GetRawButton(1);
		double x2 = xbox.GetRawAxis(5);

		frc::SmartDashboard::PutNumber("x1",x1);
		frc::SmartDashboard::PutNumber("x2",x2); */

}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
