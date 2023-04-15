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
#include "TwoEncoderAPS.hpp"
#include "utility-functions.hpp"
#include "PIDController.hpp"
#include "TBHController.hpp"

#include "portDefinitions.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <string>

/**
 * A convenient place to store all variables and motor access points
 */
namespace syndicated
{
	pros::Controller *controller;

	pros::Motor *flywheel;

	pros::Motor *indexer;

	pros::Motor *driveFrontLeft;
	pros::Motor *driveFrontRight;
	pros::Motor *driveMidRight;
	pros::Motor *driveBackRight;
	pros::Motor *driveBackLeft;
	pros::Motor *driveMidLeft;

	StarDrive *drivetrain;
	TwoEncoderAPS *odometry; // remember to set this to nullptr after calling delete syndicated::odometry;

	pros::ADIDigitalOut *pto;

	pros::Imu *imu;
	double imuMult;

	double intakeSpeed;

	double flywheelSpeed;	  // overall percentage
	double flywheelSpeedFar;  // overall percentage
	double flywheelIdleSpeed; // percentage of cross-court shot
	bool flywheelAlwaysOn;

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
	double ptoMaxDriveRPM;
	double baseMaxDriveRPM;

	PIDConfig drivetrainTranslationPID;
	PIDConfig drivetrainAngularPID;

	double indexerSpeed;
	double indexerSpeedFar;
	double indexerTravel;

	bool farShooting;
	bool shooting;

	TBHController *flywheelTBH;
	pros::Task *flywheelTBHUpdateTask;

	pros::controller_digital_e_t flywheelKeybind;
	pros::controller_digital_e_t shootKeybind;
	pros::controller_digital_e_t intakeKeybind;
	pros::controller_digital_e_t outtakeKeybind;

	pros::controller_digital_e_t farShootingToggleKeybind;
	pros::controller_digital_e_t flywheelToggleKeybind;
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

	double defaultPTOSetting = true;

	APSUpdateFrequency = 100;
	targetCycleTime = 10;

	imuMult = 360.0 / 362;

	indexerSpeed = 0.9;
	indexerSpeedFar = 0.75;
	indexerTravel = 240.0;

	flywheelAlwaysOn = false;
	flywheelSpeed = 0.5;
	flywheelSpeedFar = 0.9;
	flywheelIdleSpeed = 0 / flywheelSpeed;

	intakeSpeed = 1.0;

	ptoMaxDriveRPM = 200.0;
	baseMaxDriveRPM = 200.0;

	drivetrainTranslationPID = {1.0 / 400.0, 0.0, 0.0005, true, 0.0};
	// NOTE: high accuracy angle turning: drivetrainAngularPID = {1.0 / 131.7, 0.0, 0.001, true};
	drivetrainAngularPID = {1.0 / 131.5, 0.0015, 0.0005, true};

	shootKeybind = DIGITAL_L1;
	flywheelKeybind = DIGITAL_L2;
	outtakeKeybind = DIGITAL_R1;
	intakeKeybind = DIGITAL_R2;

	farShootingToggleKeybind = DIGITAL_UP;
	flywheelToggleKeybind = DIGITAL_LEFT;

	/**
	 * ================================
	 */
	flywheel = new pros::Motor(FLYWHEEL_PORT, true);
	flywheel->set_encoder_units(MOTOR_ENCODER_ROTATIONS);
	flywheel->set_brake_mode(MOTOR_BRAKE_COAST); // Important!
	flywheel->set_gearing(MOTOR_GEAR_600);
	flywheelTBH = nullptr;
	flywheelTBHUpdateTask = nullptr;

	flywheelVelocityTBH = 0;
	flywheelVelocityIntegral = 0;
	prevFlywheelVelocityError = 600.0 * flywheelSpeed;

	trueTimeElapsed = targetCycleTime;

	imu = new pros::Imu(IMU_PORT);
	imu->reset();
	while (imu->is_calibrating())
	{
		pros::delay(20);
	}

	odometry = new TwoEncoderAPS({'A', 'B', true}, {'C', 'D', false}, -13.0, 104.0, {220.0, 220.0, 220.858895706}, imu, imuMult);

	APSUpdateTask = new pros::Task{updateAPSTask, nullptr, "APS Update Task"};

	pros::delay(50);

	odometry->setAbsolutePosition(0.0, 0.0, 0.0);

	driveFrontLeft = new pros::Motor(DRIVE_FL_PORT, MOTOR_GEAR_GREEN, 0);
	driveFrontRight = new pros::Motor(DRIVE_FR_PORT, MOTOR_GEAR_GREEN, 1);
	driveBackRight = new pros::Motor(DRIVE_BR_PORT, MOTOR_GEAR_GREEN, 1);
	driveBackLeft = new pros::Motor(DRIVE_BL_PORT, MOTOR_GEAR_GREEN, 0);

	driveMidLeft = new PTOMotor(DRIVE_ML_PORT, MOTOR_GEAR_GREEN, 0);
	driveMidRight = new PTOMotor(DRIVE_MR_PORT, MOTOR_GEAR_GREEN, 1);

	drivetrain = new StarDrive(driveFrontLeft, driveFrontRight, driveMidRight, driveBackRight,
							   driveBackLeft, driveMidLeft, odometry);
	drivetrain->setBrakeMode(MOTOR_BRAKE_COAST);
	drivetrain->setOutputRPMs({200.0, 200.0, 200.0, 200.0, 200.0, 200.0});
	drivetrain->configTranslationalPID(drivetrainTranslationPID);
	drivetrain->configAngularPID(drivetrainAngularPID);

	ptoIsOn = defaultPTOSetting;
	pto = new pros::ADIDigitalOut(PTO_PORT, !defaultPTOSetting);
	dynamic_cast<PTOMotor *>(driveMidLeft)->set_pto_mode(defaultPTOSetting);
	dynamic_cast<PTOMotor *>(driveMidRight)->set_pto_mode(defaultPTOSetting);
	drivetrain->setMaxRPM(ptoIsOn ? ptoMaxDriveRPM : baseMaxDriveRPM);

	indexer = new pros::Motor(INDEXER_PORT, MOTOR_GEAR_BLUE, 0, MOTOR_ENCODER_DEGREES);
	indexer->tare_position();
	indexer->move_absolute(0.0, indexerSpeed * rpmFromGearset(indexer->get_gearing()));

	farShooting = false;
	shooting = false;

	pros::delay(500);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	using namespace syndicated;

	flywheelTBH->setActive(false);
	flywheelTBH->resetTBH();
	flywheel->brake();
}

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

void updatePTOState(bool state)
{
	using namespace syndicated;

	ptoIsOn = state;
	PTOMotor *mr_ptr = dynamic_cast<PTOMotor *>(driveMidRight);
	PTOMotor *ml_ptr = dynamic_cast<PTOMotor *>(driveMidLeft);
	pto->set_value(!ptoIsOn);
	mr_ptr->set_pto_mode(ptoIsOn);
	ml_ptr->set_pto_mode(ptoIsOn);
	drivetrain->setMaxRPM(ptoIsOn ? ptoMaxDriveRPM : baseMaxDriveRPM);
}

void moveToPose(absolutePosition pose)
{
	using namespace syndicated;

	drivetrain->initiateMotionTo(pose);

	while (drivetrain->isPIDActive())
	{
		drivetrain->moveFollowingMotionProfile();
		pros::delay(10);
	}
}

void turnToPoint(double x, double y, double offset = 0.0)
{
	using namespace syndicated;

	auto currentPose = odometry->getAbsolutePosition();
	moveToPose({currentPose.x, currentPose.y, headingToPoint(x - currentPose.x, y - currentPose.y) + offset});
}

void turnToHeading(double heading)
{
	using namespace syndicated;

	auto currentPose = odometry->getAbsolutePosition();
	moveToPose({currentPose.x, currentPose.y, heading});
}

void shootDisc()
{
	using namespace syndicated;

	indexer->move_relative(indexerTravel, indexerSpeedFar * 600.0);
	if (flywheelTBH != nullptr)
	{
		flywheelTBH->setSettled(false);
	}

	do
	{
		pros::delay(20);
	} while (!indexer->is_stopped());
}

void flywheelTBHLoop(void *param)
{
	using namespace syndicated;

	while (true)
	{
		if (flywheelTBH != nullptr)
		{
			if (flywheelTBH->isActive())
			{
				auto rpm = flywheel->get_actual_velocity();
				flywheelTBH->updateTBH(rpm);
				auto output = flywheelTBH->getOutput();
				flywheel->move_voltage(output);
				pros::screen::print(TEXT_MEDIUM, 0, "%f voltage input and %f rpm", output, rpm);
			}
			else
			{
				flywheel->brake();
			}
		}
		pros::delay(10);
	}
}

void waitUntilFlywheelSettled(double msecTimeout = 0)
{
	using namespace syndicated;
	if (flywheelTBH == nullptr)
	{
		return;
	}

	bool timeoutDisabled = msecTimeout == 0;

	pros::delay(150);

	do
	{
		pros::delay(150);

		if (!timeoutDisabled)
		{
			msecTimeout -= 150;
		}

		if (msecTimeout < 0)
		{
			return;
		}
	} while (!flywheelTBH->isSettled());
}

void intakeOn(bool moveIndexer = false)
{
	using namespace syndicated;

	PTOMotor *ml_ptr = dynamic_cast<PTOMotor *>(driveMidLeft);
	PTOMotor *mr_ptr = dynamic_cast<PTOMotor *>(driveMidRight);

	ml_ptr->move_velocity_if_pto(200.0 * intakeSpeed);
	mr_ptr->move_velocity_if_pto(200.0 * intakeSpeed);

	if (moveIndexer)
	{
		indexer->move_velocity(-600.0 * intakeSpeed);
	}
}

void outtakeOn()
{

	using namespace syndicated;

	PTOMotor *ml_ptr = dynamic_cast<PTOMotor *>(driveMidLeft);
	PTOMotor *mr_ptr = dynamic_cast<PTOMotor *>(driveMidRight);

	ml_ptr->move_velocity_if_pto(-200.0 * intakeSpeed);
	mr_ptr->move_velocity_if_pto(-200.0 * intakeSpeed);
}

void intakeOff(bool brakeIndexer = false)
{
	using namespace syndicated;

	PTOMotor *ml_ptr = dynamic_cast<PTOMotor *>(driveMidLeft);
	PTOMotor *mr_ptr = dynamic_cast<PTOMotor *>(driveMidRight);

	ml_ptr->brake_if_pto();
	mr_ptr->brake_if_pto();

	if (brakeIndexer)
	{
		indexer->brake();
	}
}

void driveAndMaintainHeading(double vt, double ht, double heading)
{
	using namespace syndicated;

	auto pose = odometry->getAbsolutePosition();
	auto delta = findShorterTurn(pose.heading, heading, 360.0);
	auto vr = 0.0;
	if (delta > 1.0)
	{
		vr = 0.05;
	}
	else if (delta < -1.0)
	{
		vr = -0.05;
	}

	drivetrain->driveAndTurn(vt, ht, vr);
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

	updatePTOState(true);

	if (autonomousSkills)
	{
	}
	else if (soloAuton)
	{
	}
	else if (startingOnRoller)
	{
	}
	else
	{
		odometry->setAbsolutePosition(-60.0, 1800.0, 270.0);

		flywheelTBH = new TBHController(146.0, 13000.0); // TODO: tune gain
		flywheelTBH->setTarget(370.0);
		flywheelTBH->setThresholds(5.0, 5.0); // TODO: confirm maximum thresholds
		flywheelTBHUpdateTask = new pros::Task(flywheelTBHLoop, nullptr, "Flywheel TBH Update");

		intakeOn(true);

		for (int i = 0; i < 90; i++)
		{
			driveAndMaintainHeading(0.60, 277.5, 270.0);
			pros::delay(10);
		}
		drivetrain->brake();
		pros::delay(200);
		turnToPoint(-2700.0, 2700.0, -3.5);
		pros::delay(200);
		intakeOff(true);

		waitUntilFlywheelSettled(3000);
		pros::screen::print(TEXT_MEDIUM, 3, "%f", flywheel->get_actual_velocity());
		shootDisc();
		waitUntilFlywheelSettled(700);
		pros::screen::print(TEXT_MEDIUM, 4, "%f", flywheel->get_actual_velocity());
		shootDisc();
		waitUntilFlywheelSettled(400);
		pros::screen::print(TEXT_MEDIUM, 5, "%f", flywheel->get_actual_velocity());
		shootDisc();

		pros::delay(100);
		moveToPose({-160.0, 2400.0, 270.0});
		driveAndMaintainHeading(0.1, 90.0, 270.0);
		pros::delay(800);
		indexer->move_relative(220.0, 300.0);
		pros::delay(200);
		driveAndMaintainHeading(0.1, 270.0, 270.0);
		pros::delay(750);
		drivetrain->brake();
		disabled();

	}

	/* testing area */
	{
		/*
		drivetrain->initiateMotionTo({0.0, 0.0, 90.0});

		while (drivetrain->isPIDActive())
		{
			drivetrain->moveFollowingMotionProfile();
			pros::delay(10);
		}
		*/
	}

	/* demo route */
	{
		/*
		drivetrain->initiateMotionTo({400.0, 400.0, 90.0});

		while (drivetrain->isPIDActive())
		{
			drivetrain->moveFollowingMotionProfile();
			pros::delay(10);
		}

		drivetrain->initiateMotionTo({0.0, 800.0, 180.0});

		while (drivetrain->isPIDActive())
		{
			drivetrain->moveFollowingMotionProfile();
			pros::delay(10);
		}

		drivetrain->initiateMotionTo({-400.0, 400.0, 270.0});

		while (drivetrain->isPIDActive())
		{
			drivetrain->moveFollowingMotionProfile();
			pros::delay(10);
		}

		drivetrain->initiateMotionTo({0.0, 0.0, 0.0});

		while (drivetrain->isPIDActive())
		{
			drivetrain->moveFollowingMotionProfile();
			pros::delay(10);
		}*/
	}
	/* demo route 2 */
	{
		/*
		drivetrain->initiateMotionTo({1200.0, 2000.0, 0.0});

		while (drivetrain->isPIDActive())
		{
			drivetrain->moveFollowingMotionProfile();
			pros::delay(10);
		}

		drivetrain->initiateMotionTo({400.0, 2000.0, 0.0});

		while (drivetrain->isPIDActive())
		{
			drivetrain->moveFollowingMotionProfile();
			pros::delay(10);
		}

		drivetrain->initiateMotionTo({0.0, 1200.0, 90.0});

		while (drivetrain->isPIDActive())
		{
			drivetrain->moveFollowingMotionProfile();
			pros::delay(10);
		}

		drivetrain->initiateMotionTo({4800.0, 1200.0, 90.0});

		while (drivetrain->isPIDActive())
		{
			drivetrain->moveFollowingMotionProfile();
			pros::delay(10);
		}
		*/
	}
}

void handleIntakeControls()
{
	using namespace syndicated;

	if (controller->get_digital(intakeKeybind))
	{
		intakeOn();
	}
	else if (controller->get_digital(outtakeKeybind))
	{
		outtakeOn();
	}
	else
	{
		// brake the motor only if the drivetrain is not controlling the motor
		intakeOff();
	}
}

void handleIndexerControls()
{
	using namespace syndicated;

	if (controller->get_digital_new_press(shootKeybind)) // should be get_digital_new_press
	{
		double indexerRPM = (farShooting ? indexerSpeedFar : indexerSpeed) * rpmFromGearset(indexer->get_gearing());
		indexer->move_absolute((shooting ? indexer->get_target_position() : indexer->get_position()) + indexerTravel, indexerRPM);
		shooting = true;
	}

	if (std::abs(flywheel->get_actual_velocity()) < 200.0)
	{
		shooting = false;
	}

	if (std::abs(indexer->get_target_position() - indexer->get_position()) < 1.5)
	{
		shooting = false;
	}

	if (controller->get_digital(intakeKeybind) && !shooting)
	{
		indexer->move_velocity(-600.0 * intakeSpeed);
	}
	else if (!shooting && !(controller->get_digital(intakeKeybind)))
	{
		indexer->brake();
	}
}

void handleFlywheelControls()
{
	using namespace syndicated;

	if (controller->get_digital_new_press(flywheelToggleKeybind))
	{
		flywheelAlwaysOn = !flywheelAlwaysOn;
	}

	if (controller->get_digital(flywheelKeybind) || flywheelAlwaysOn)
	{
		// use PID control for the flywheel, adding TBH (take back half) method
		/**
		 * Programmer's notes:
		 * A PI controller (kD = 0) may be more accurate, as the flywheel's friction will fight against P and I, so there's no need for D
		 * A derivative component is probably best for applications where it is okay to reverse in order to settle the velocity. However,
		   spinning the flywheel in reverse would be a terrible idea.
		*/
		/*
		double target = 600.0 * flywheelSpeed; // proportion of 600 rpm
		double actualVelocity = flywheel->get_actual_velocity();
		double error = target - actualVelocity;
		double kP = 3.0, kI = 0.0, kD = 0.0; // TODO: tune values, try PI only before doing PID
		syndicated::flywheelVelocityIntegral *= 0.95;
		syndicated::flywheelVelocityIntegral += error * syndicated::trueTimeElapsed;
		if (std::signbit(syndicated::prevFlywheelVelocityError) != std::signbit(error))
		{
			// take back half
			syndicated::flywheelVelocityIntegral = 0.5 * (syndicated::flywheelVelocityIntegral + syndicated::flywheelVelocityTBH);
			syndicated::flywheelVelocityTBH = syndicated::flywheelVelocityIntegral;
		}
		double derivative = (error - syndicated::prevFlywheelVelocityError) / syndicated::trueTimeElapsed;
		double power = error * kP + syndicated::flywheelVelocityIntegral * kI + derivative * kD;
		flywheel->move(std::min(std::max(127.0 * power / 600.0, 0.0), 127.0));
		syndicated::prevFlywheelVelocityError = error;
		*/
		flywheel->move_velocity(600.0 * (farShooting ? flywheelSpeedFar : flywheelSpeed));
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
		odometry->setAbsolutePosition(APS_NO_CHANGE, APS_NO_CHANGE, 0);
	}
}

void handlePTOControls()
{
	using namespace syndicated;

	if (controller->get_digital_new_press(DIGITAL_B))
	{
		updatePTOState(!ptoIsOn);
	}
}

void handleRangeSwitching()
{
	using namespace syndicated;

	if (controller->get_digital_new_press(farShootingToggleKeybind))
	{
		farShooting = !farShooting;
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

	double previousTurnVelocity = 0.0;
	double turnMaxAccel = 0.1;
	std::vector<std::pair<double, double>> points = {};

	while (true)
	{
		auto cycleStart = std::chrono::high_resolution_clock::now();

		pros::screen::erase();
		pros::screen::print(TEXT_MEDIUM, 1, ptoIsOn ? "PTO: on" : "PTO: off");
		auto pos = odometry->getAbsolutePosition();
		pros::screen::print(TEXT_MEDIUM, 2, std::to_string(pos.heading).c_str());
		pros::screen::print(TEXT_MEDIUM, 3, "X: %f, Y: %f", pos.x, pos.y);
		auto encPair = odometry->getEncoderReadings();
		pros::screen::print(TEXT_MEDIUM, 5, "X Encoder: %f", encPair.x);
		pros::screen::print(TEXT_MEDIUM, 6, "Y Encoder: %f", encPair.y);
		pros::screen::print(TEXT_MEDIUM, 7, "dX: %f", encPair.x * 220.0 / 360.0);
		pros::screen::print(TEXT_MEDIUM, 8, "dY: %f", encPair.y * 220.0 / 360.0);

		pros::screen::set_pen(COLOR_CORNFLOWER_BLUE);
		double x1 = 300, y1 = 50, x2 = 450, y2 = 200;
		pros::screen::draw_rect(x1, y1, x2, y2);
		double originX = (x2 + x1) / 2.0;
		double originY = (y2 + y1) / 2.0;
		pros::screen::set_pen(COLOR_PALE_VIOLET_RED);
		double scale = 0.15;
		auto drawX = originX + scale * pos.x;
		auto drawY = originY - scale * pos.y;
		points.push_back({drawX, drawY});
		if (points.size() > 150)
		{
			points.erase(points.begin());
		}
		for (auto p : points)
		{
			pros::screen::draw_circle(p.first, p.second, 1);
		}

		double crx = controller->get_analog(ANALOG_RIGHT_X) / 127.0;
		double cry = controller->get_analog(ANALOG_RIGHT_Y) / 127.0;
		double clx = controller->get_analog(ANALOG_LEFT_X) / 127.0;
		double cly = controller->get_analog(ANALOG_LEFT_Y) / 127.0;

		polarPoint translationVector = polarFromCartesian(crx, cry);
		translationVector.rho = std::min(translationVector.rho, 1.0);
		auto translationHeading = 90 - translationVector.theta;

		if (std::abs(crx) < 0.01 && std::abs(cry) < 0.01 && std::abs(clx) < 0.01)
		{
			drivetrain->brake();
			previousTurnVelocity = 0.0;
		}
		else if (std::abs(clx) < 0.01)
		{
			drivetrain->drive(translationVector.rho, translationHeading);
		}
		else
		{
			double turnVelocity = (clx >= 0 ? 1.0 : -1.0) * (cos(3.141592 * (clx + 1.0)) / 2.0 + 0.5);
			turnVelocity = std::max(std::min(turnVelocity, previousTurnVelocity + turnMaxAccel), previousTurnVelocity - turnMaxAccel);
			drivetrain->driveAndTurn(translationVector.rho, translationHeading, turnVelocity);
			previousTurnVelocity = turnVelocity;
		}

		handlePTOControls();
		handleRangeSwitching();
		handleHeadingReset();

		handleIntakeControls();
		handleIndexerControls();
		handleFlywheelControls();

		double cycleTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cycleStart).count();
		pros::delay(std::max(0.0, targetCycleTime - cycleTime));
		trueTimeElapsed = std::max(cycleTime, targetCycleTime);
		timeSinceLastShot += trueTimeElapsed;
	}
}
