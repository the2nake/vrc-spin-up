/**
 * Filename: main.cpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
 */

#include "main.h"

#include "HolonomicXDrive.hpp"
#include "MotorGroup.hpp"
#include "APS.hpp"
#include "utility-functions.hpp"

#include "portDefinitions.h"

#include <chrono>
#include <algorithm>
#include <string>

/**
 * A convenient place to store all variables and motor access points
 */
namespace syndicated
{
	pros::Controller *controller;
	pros::Imu *imu;

	pros::Motor *intake;
	pros::Motor *indexer;
	pros::Motor *flywheel;
	pros::Motor *expansion;

	HolonomicXDrive *drivetrain;
	APS *odometry; // remember to set this to nullptr after calling delete syndicated::odometry;

	double imuDriftPerMsec;

	double intakeSpeed;
	double flywheelSpeed;	  // overall percentage
	double flywheelIdleSpeed; // percentage of cross-court shot
	bool flywheelAlwaysOn;
	double indexerSpeed;

	bool shotReady;
	double timeSinceLastShot;

	bool autonomousSkills;
	bool startingOnRoller;
	bool soloAuton;

	double prevFlywheelVelocityError;
	double flywheelVelocityTBH;
	double flywheelVelocityIntegral;

	double targetCycleTime;
	double trueTimeElapsed;

	pros::Task *APSUpdateTask;
	int APSUpdateFrequency; // in Hz
};

void updateAPSTask(void *param)
{
	using namespace syndicated;
	int updateDelay = (int)(1000 / APSUpdateFrequency); // in ms
	while (odometry != nullptr)
	{
		odometry->updateAbsolutePosition();

		pros::delay(updateDelay);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	using namespace syndicated;

	/**
	 * ================================
	 *    FLAGS
	 * ================================
	*/

	autonomousSkills = false;
	startingOnRoller = false;
	soloAuton = false;
	
	APSUpdateFrequency = 200;
	targetCycleTime = 40;

	indexerSpeed = 0.5;

	flywheelAlwaysOn = false;
	flywheelSpeed = 0.60;
	flywheelIdleSpeed = 0 / flywheelSpeed;

	intakeSpeed = 1.0;

	/**
	 * ================================
	*/

	imu = new pros::Imu(IMU_PORT);
	imu->reset();

	while (imu->is_calibrating())
	{
		pros::delay(16);
	}

	double startHeading = imu->get_heading();
	pros::delay(160);
	imuDriftPerMsec = (imu->get_heading() - startHeading) / 160.0;

	drivetrain = new HolonomicXDrive(DRIVE_FL_PORT, DRIVE_FR_PORT, DRIVE_BR_PORT, DRIVE_BL_PORT, IMU_PORT);

	intake = new pros::Motor(INTAKE_PORT);
	intake->set_brake_mode(MOTOR_BRAKE_BRAKE);
	flywheel = new pros::Motor(FLYWHEEL_PORT, true);
	flywheel->set_encoder_units(MOTOR_ENCODER_ROTATIONS);
	flywheel->set_brake_mode(MOTOR_BRAKE_COAST); // Important!
	flywheel->set_gearing(MOTOR_GEAR_600);

	indexer = new pros::Motor(INDEXER_PORT, 1);
	indexer->set_gearing(MOTOR_GEAR_200);
	indexer->set_brake_mode(MOTOR_BRAKE_HOLD);
	indexer->set_encoder_units(MOTOR_ENCODER_DEGREES);

	indexer->move_velocity(200.0 * indexerSpeed);
	pros::delay(200);
	while (indexer->get_efficiency() > 0.5)
	{
		indexer->move_velocity(200.0 * indexerSpeed);
		pros::delay(16);
	}
	indexer->brake();
	indexer->tare_position();
	shotReady = true;

	expansion = new pros::Motor(EXPANSION_PORT, 1);
	expansion->set_gearing(MOTOR_GEAR_RED);
	expansion->set_brake_mode(MOTOR_BRAKE_BRAKE);
	expansion->set_encoder_units(MOTOR_ENCODER_ROTATIONS);
	expansion->brake();

	flywheelVelocityTBH = 0;
	flywheelVelocityIntegral = 0;
	prevFlywheelVelocityError = 600.0 * flywheelSpeed;

	trueTimeElapsed = targetCycleTime;

	APSUpdateTask = new pros::Task{updateAPSTask, nullptr, "APS Update Task"};
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

void doAutonRoller(double mult = 1.0)
{
	using namespace syndicated;

	double frictionCoef = 2;

	drivetrain->drive(0.2, findMod(180 + imu->get_heading(), 360));

	pros::delay(750);

	intake->move_relative(mult * frictionCoef * 360.0, 100);

	drivetrain->brake();

	pros::delay(100);

	while (intake->get_actual_velocity() > 0.1)
	{
		pros::delay(20);
	}

	drivetrain->drive(0.5, imu->get_heading());

	pros::delay(250);

	drivetrain->brake();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
	using namespace syndicated;

	if (autonomousSkills)
	{
		imu->set_heading(0);
		doAutonRoller();

		double targetHeading = 90.0;
		double delta = targetHeading - imu->get_heading();
		while (std::abs(delta) > 1)
		{
			drivetrain->driveAndTurn(0.5, 315, std::max(0.33, std::min(1.0, (0.5 * delta / 90))));
			pros::delay(20);

			delta = targetHeading - imu->get_heading();
		}

		drivetrain->brake();

		drivetrain->drive(0.5, 315);

		pros::delay(100);

		drivetrain->brake();

		drivetrain->drive(0.5, 270);

		pros::delay(500);

		drivetrain->brake();

		doAutonRoller();

		targetHeading = 45;
		delta = targetHeading - imu->get_heading();
		while (std::abs(delta) > 1)
		{
			drivetrain->driveAndTurn(0.5, 135, std::min(-0.33, (0.5 * delta / 90))); // delta would be negative here
			pros::delay(20);

			delta = targetHeading - imu->get_heading();
		}

		drivetrain->brake();
		expansion->move_relative(2, 100);
	}
	else
	{
		if (soloAuton)
		{
		}
		else
		{
			if (startingOnRoller)
			{
				doAutonRoller(0.5);
			}
			else
			{
				double diagTime = 800;

				drivetrain->drive(1, 45);
				pros::delay(diagTime);
				drivetrain->brake();

				pros::delay(100);

				drivetrain->drive(1, 135);
				pros::delay(1.1 * diagTime);
				drivetrain->brake();
				doAutonRoller(0.5);

				imu->set_heading(270);
			}
		}
	}
}

void handleIntakeControls()
{
	using namespace syndicated;

	if (controller->get_digital(DIGITAL_R1))
	{
		intake->move_velocity(200 * intakeSpeed);
	}
	else if (controller->get_digital(DIGITAL_R2))
	{
		intake->move(-127.0 * intakeSpeed);
	}
	else
	{
		intake->brake();
	}
}

void handleFlywheelControls()
{
	using namespace syndicated;

	if (controller->get_digital_new_press(DIGITAL_DOWN))
	{
		flywheelAlwaysOn = !flywheelAlwaysOn;
	}

	if (controller->get_digital(DIGITAL_L2) || flywheelAlwaysOn)
	{
		// use PID control for the flywheel, adding TBH (take back half) method
		/**
		 * Programmer's notes:
		 * A PI controller (kD = 0) may be more accurate, as the flywheel's friction will fight against P and I, so there's no need for D
		 * A derivative component is probably best for applications where it is okay to reverse in order to settle the velocity. However,
		   spinning the flywheel in reverse would be a terrible idea.
		 * TODO: Research take back half in more detail. What exactly is being taken half of? Is TBH even necessary, since the flywheel will
		   use friction to slow down?
		*/

		double target = 600.0 * flywheelSpeed; // proportion of 600 rpm
		double actualVelocity = flywheel->get_actual_velocity();
		double error = target - actualVelocity;
		double kP = 1.0, kI = 0.0, kD = 0.0; // TODO: tune values, try PI only before doing PID
		syndicated::flywheelVelocityIntegral *= 0.9;
		syndicated::flywheelVelocityIntegral += error * syndicated::trueTimeElapsed;
		if (std::signbit(syndicated::prevFlywheelVelocityError) != std::signbit(error))
		{
			// take back half
			// TODO: test if necessary
			syndicated::flywheelVelocityIntegral = 0.5 * (syndicated::flywheelVelocityIntegral + syndicated::flywheelVelocityTBH);
			syndicated::flywheelVelocityTBH = syndicated::flywheelVelocityIntegral;
		}
		double derivative = (error - syndicated::prevFlywheelVelocityError) / syndicated::trueTimeElapsed;
		double power = error * kP + syndicated::flywheelVelocityIntegral * kI + derivative * kD;
		flywheel->move(std::max(127.0 * power / 600.0, 0.0));
		syndicated::prevFlywheelVelocityError = error;
	}
	else
	{
		flywheel->move_velocity(600 * flywheelIdleSpeed * flywheelSpeed);
	}
}

void handleHeadingReset()
{
	using namespace syndicated;

	if (controller->get_digital(DIGITAL_B))
	{
		imu->set_heading(0);
		odometry->setAbsolutePosition(APS_NO_CHANGE, APS_NO_CHANGE, 0);
	}
}

void handleIndexer()
{
	using namespace syndicated;

	if (controller->get_digital(DIGITAL_L1))
	{
		indexer->move_velocity(-200 * indexerSpeed);
		timeSinceLastShot = 0;
		shotReady = false;
	}
	else if (!shotReady)
	{
		indexer->move_velocity(200 * indexerSpeed);

		if (timeSinceLastShot > 1500)
		{
			indexer->brake();
			shotReady = true;
		}
	}
	else
	{
		indexer->brake();
	}
}

void handleExpansionControls()
{
	using namespace syndicated;

	if (controller->get_digital(DIGITAL_UP))
	{
		expansion->move(127.0);
	}
	else
	{
		expansion->brake();
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	using namespace syndicated;

	syndicated::controller = new pros::Controller(pros::E_CONTROLLER_MASTER);

	if (!autonomousSkills && !startingOnRoller)
	{
		imu->set_heading(270);
		odometry->setAbsolutePosition(APS_NO_CHANGE, APS_NO_CHANGE, 270);
	}

	double headingToMaintain = imu->get_heading();

	while (true)
	{
		auto cycleStart = std::chrono::high_resolution_clock::now();

		double crx = controller->get_analog(ANALOG_RIGHT_X) / 127.0;
		double cry = controller->get_analog(ANALOG_RIGHT_Y) / 127.0;
		double clx = controller->get_analog(ANALOG_LEFT_X) / 127.0;
		double cly = controller->get_analog(ANALOG_LEFT_Y) / 127.0;

		/*
		controller->clear();
		if (flywheelAlwaysOn)
		{
			controller->print(0, 0, "Shoot mode: ALWAYS ON");
		}
		else
		{
			controller->print(0, 0, (std::string("Shoot mode: IDLE at") + std::to_string(flywheelIdleSpeed)).c_str());
		}

		controller->print(1, 0, (std::string("Flywheel RPM:  ") + std::to_string(flywheel->get_actual_velocity())).c_str());
		controller->print(2, 0, (std::string("Flywheel Temp: ") + std::to_string(flywheel->get_temperature())).c_str());
		*/

		polarPoint translationVector = polarFromCartesian(crx, cry);
		translationVector.rho = std::min(translationVector.rho, 1.0);

		if (std::abs(clx) < 0.01)
		{
			headingToMaintain = imu->get_heading();
		}

		if (std::abs(crx) + std::abs(cry) < 0.01)
		{
			if (std::abs(cly) < 0.01 && std::abs(clx) < 0.01)
			{
				drivetrain->brake();
			}
			else
			{
				drivetrain->driveAndTurn(cly, imu->get_heading(), clx);
			}
		}
		else if (std::abs(clx) < 0.01)
		{
			// use a PID controller

			double angleToHeadingAnticlockwise = findMod(360 + imu->get_heading() - headingToMaintain, 360);
			bool shouldTurnAnticlockwise = angleToHeadingAnticlockwise < 180;

			double turnVelocity = (shouldTurnAnticlockwise ? -1 : 1) * 0.5 * (shouldTurnAnticlockwise ? sinDeg(angleToHeadingAnticlockwise / 2.0) : sinDeg(180 - angleToHeadingAnticlockwise / 2.0));

			drivetrain->driveAndTurn(translationVector.rho, 90 - translationVector.theta, turnVelocity);
		}
		else
		{
			drivetrain->driveAndTurn(translationVector.rho, 90 - translationVector.theta, clx);
		}

		handleIntakeControls();
		handleFlywheelControls();
		handleIndexer();
		handleExpansionControls();

		handleHeadingReset();

		double cycleTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cycleStart).count();
		pros::delay(std::max(0.0, targetCycleTime - cycleTime));
		trueTimeElapsed = std::max(cycleTime, targetCycleTime);
		// imu sensor drift correction
		imu->set_heading(imu->get_heading() - (trueTimeElapsed * imuDriftPerMsec));
		timeSinceLastShot += trueTimeElapsed;
	}
}
