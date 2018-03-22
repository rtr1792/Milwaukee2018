/*
 * drive.h
 *
 *  Created on: Jan 24, 2018
 *      Author: RTR
 */


#ifndef SRC_SUBCLASS_DRIVE_H_
#define SRC_SUBCLASS_DRIVE_H_
#include "AHRS.h"

class DriveManager {
private:
		WPI_TalonSRX *srx1;
		WPI_TalonSRX *srx12;
		WPI_TalonSRX *srx13;

		WPI_TalonSRX *srx2;
		WPI_TalonSRX *srx21;
		WPI_TalonSRX *srx22;

		DifferentialDrive *m_robotDrive;

		Joystick *stick;
		XboxController *xbox;

		double *rightStickValue;
		double *leftStickValue;
		double *vel1;
		double *vel2;
		double *dis;
		double *dis2;
		int *init;
		int *one;


	    AHRS *ahrs;
public:
	DriveManager();
	void driveTrain();
	void Drive(double speed, double goDistance);
	void Turn(int angle);
	void ResetSensors();
	void setCoast();
	void FindStartEnc();
	void DriveNew(double speed, double goDistance);
};
//	null ArcadeDrive(double, double, bool);

#endif /* SRC_SUBCLASS_DRIVE_H_ */
