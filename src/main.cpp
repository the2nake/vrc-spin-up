/**
 * Filename: main.cpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
 */

#include "main.h"

#include "PTOMotor.hpp"
#include "StarDrive.hpp"
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

	pros::Motor *flywheel;

	pros::Motor *driveFrontLeft;
	pros::Motor *driveFrontRight;
	pros::Motor *driveMidRight;
	pros::Motor *driveBackRight;
	pros::Motor *driveBackLeft;
	pros::Motor *driveMidLeft;

	StarDrive *drivetrain;
	APS *odometry; // remember to set this to nullptr after calling delete syndicated::odometry;

	pros::ADIDigitalOut *pto;

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

	bool ptoIsOn;
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

	APSUpdateFrequency = 100;
	targetCycleTime = 40;

	indexerSpeed = 0.5;

	flywheelAlwaysOn = false;
	flywheelSpeed = 0.60;
	flywheelIdleSpeed = 0 / flywheelSpeed;

	intakeSpeed = 1.0;

	/**
	 * ================================
	 */
	flywheel = new pros::Motor(FLYWHEEL_PORT, true);
	flywheel->set_encoder_units(MOTOR_ENCODER_ROTATIONS);
	flywheel->set_brake_mode(MOTOR_BRAKE_COAST); // Important!
	flywheel->set_gearing(MOTOR_GEAR_600);

	flywheelVelocityTBH = 0;
	flywheelVelocityIntegral = 0;
	prevFlywheelVelocityError = 600.0 * flywheelSpeed;

	trueTimeElapsed = targetCycleTime;

	APSUpdateTask = new pros::Task{updateAPSTask, nullptr, "APS Update Task"};
	odometry = new APS({'A', 'B', true}, {'C', 'D', true}, {'E', 'F', true}, 7.0, 7.0, 0.5, {4, 4, 2.75});

	driveFrontLeft = new pros::Motor(DRIVE_FL_PORT);
	driveFrontRight = new pros::Motor(DRIVE_FR_PORT, 1);
	driveBackRight = new pros::Motor(DRIVE_BR_PORT, 1);
	driveBackLeft = new pros::Motor(DRIVE_BL_PORT);

	driveMidLeft = new PTOMotor(DRIVE_ML_PORT);
	driveMidRight = new PTOMotor(DRIVE_MR_PORT, 1);

	drivetrain = new StarDrive(driveFrontLeft, driveFrontRight, driveMidRight, driveBackRight,
							   driveBackLeft, driveMidLeft, odometry);

	ptoIsOn = false;
	pto = new pros::ADIDigitalOut(PTO_PORT);
	dynamic_cast<PTOMotor *>(driveMidLeft)->set_pto_mode(false);
	dynamic_cast<PTOMotor *>(driveMidRight)->set_pto_mode(false);
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
}

void handleIntakeControls()
{
	using namespace syndicated;

	PTOMotor *ml_ptr = dynamic_cast<PTOMotor *>(driveMidLeft);
	PTOMotor *mr_ptr = dynamic_cast<PTOMotor *>(driveMidRight);

	if (controller->get_digital(DIGITAL_R1))
	{
		// reverse is intake (because of geartrain)
		ml_ptr->move_velocity_if_pto(-200.0 * intakeSpeed);
		mr_ptr->move_velocity_if_pto(-200.0 * intakeSpeed);
	}
	else if (controller->get_digital(DIGITAL_R2))
	{
		// forward is remove/index (because of geartrain)
		ml_ptr->move_velocity_if_pto(200.0 * intakeSpeed);
		mr_ptr->move_velocity_if_pto(200.0 * intakeSpeed);
	}
	else
	{
		// brake the motor only if the drivetrain is not controlling the motor
		ml_ptr->brake_if_pto();
		mr_ptr->brake_if_pto();
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

	if (controller->get_digital(DIGITAL_Y))
	{
		pros::screen::print(TEXT_MEDIUM, 4, "Reseting heading");
		odometry->setAbsolutePosition(APS_NO_CHANGE, APS_NO_CHANGE, 0);
	}
}

void handlePTOControls()
{
	using namespace syndicated;

	if (controller->get_digital_new_press(DIGITAL_B))
	{
		ptoIsOn = !ptoIsOn;
		PTOMotor *mr_ptr = dynamic_cast<PTOMotor *>(driveMidRight);
		PTOMotor *ml_ptr = dynamic_cast<PTOMotor *>(driveMidLeft);
		pto->set_value(ptoIsOn);
		mr_ptr->set_pto_mode(ptoIsOn);
		ml_ptr->set_pto_mode(ptoIsOn);
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
		odometry->setAbsolutePosition(APS_NO_CHANGE, APS_NO_CHANGE, 270);
	}

	double headingToMaintain = odometry->getAbsolutePosition().heading;

	while (true)
	{
		auto cycleStart = std::chrono::high_resolution_clock::now();

		pros::screen::erase();
		pros::screen::print(TEXT_MEDIUM, 1, ptoIsOn ? "PTO: on" : "PTO: off");
		pros::screen::print(TEXT_MEDIUM, 2, std::to_string(odometry->getAbsolutePosition().heading).c_str());

		double crx = controller->get_analog(ANALOG_RIGHT_X) / 127.0;
		double cry = controller->get_analog(ANALOG_RIGHT_Y) / 127.0;
		double clx = controller->get_analog(ANALOG_LEFT_X) / 127.0;
		double cly = controller->get_analog(ANALOG_LEFT_Y) / 127.0;

		polarPoint translationVector = polarFromCartesian(crx, cry);
		translationVector.rho = std::min(translationVector.rho, 1.0);

		if (std::abs(clx) > 0.01)
		{
			headingToMaintain = odometry->getAbsolutePosition().heading;
		}

		if (std::abs(crx) < 0.01 && std::abs(cry) < 0.01 && std::abs(clx) < 0.01)
		{
			drivetrain->brake();
		}
		else if (std::abs(clx) < 0.01)
		{
			drivetrain->driveAndMaintainHeading(translationVector.rho, 90 - translationVector.theta, headingToMaintain);
		}
		else
		{
			drivetrain->driveAndTurn(translationVector.rho, 90 - translationVector.theta, clx);
		}

		handlePTOControls();
		handleIntakeControls();
		// handleFlywheelControls();

		handleHeadingReset();

		double cycleTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cycleStart).count();
		pros::delay(std::max(0.0, targetCycleTime - cycleTime));
		trueTimeElapsed = std::max(cycleTime, targetCycleTime);
		timeSinceLastShot += trueTimeElapsed;
	}
}
