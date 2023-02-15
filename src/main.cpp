#include "main.h"

#include "HolonomicXDrive.hpp"
#include "MotorGroup.hpp"
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
	pros::Imu *imuSensor;
	pros::ADIAnalogIn *indexerLineTracker;

	pros::Motor *intake;
	pros::Motor *indexer;
	MotorGroup *flywheel;
	HolonomicXDrive *drivetrain;

	double intakeSpeed;
	double flywheelSpeed;	  // overall percentage
	double flywheelIdleSpeed; // percentage of cross-court shot
	double indexerSpeed;

	bool shotReady;
	double timeSinceLastShot;
};

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	using namespace syndicated;

	imuSensor = new pros::Imu(IMU_PORT);
	imuSensor->reset();

	while (imuSensor->is_calibrating())
	{
		pros::delay(16);
	}

	drivetrain = new HolonomicXDrive(DRIVE_FL_PORT, DRIVE_FR_PORT, DRIVE_BR_PORT, DRIVE_BL_PORT, IMU_PORT);

	intake = new pros::Motor(INTAKE_PORT);
	intake->set_brake_mode(MOTOR_BRAKE_BRAKE);
	intakeSpeed = 1.0;

	flywheel = new MotorGroup({motorConfiguration{FLYWHEEL_PORT_A, false}, motorConfiguration{FLYWHEEL_PORT_R, true}});
	flywheel->set_gearing(MOTOR_GEAR_600);
	flywheelSpeed = 0.7;
	flywheelIdleSpeed = 0 / flywheelSpeed;

	indexer = new pros::Motor(INDEXER_PORT, 1);
	indexer->set_gearing(MOTOR_GEAR_200);
	indexer->set_brake_mode(MOTOR_BRAKE_HOLD);
	indexer->set_encoder_units(MOTOR_ENCODER_DEGREES);
	indexerSpeed = 0.4;

	indexer->move_velocity(200.0 * indexerSpeed);
	pros::delay(200);
	while (indexer->get_efficiency() > 1)
	{
		indexer->move_velocity(200.0 * indexerSpeed);
		pros::delay(16);
	}
	indexer->brake();
	indexer->tare_position();
	shotReady = true;
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
void autonomous() {}

void handleIntakeControls()
{
	using namespace syndicated;

	if (!shotReady)
	{
		intake->brake();
		return;
	}

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

	if (controller->get_digital(DIGITAL_L2))
	{
		flywheel->move_velocity(600 * flywheelSpeed);
	}
	else
	{
		flywheel->move_velocity(600 * flywheelIdleSpeed * flywheelSpeed);
	}
}

void handleImuReset()
{
	using namespace syndicated;

	if (controller->get_digital(DIGITAL_B))
	{
		imuSensor->set_heading(0);
	}
}

void handleIndexer()
{
	using namespace syndicated;

	if (std::abs(indexer->get_position()) < 5)
	{
		shotReady = true;
	} else {
		shotReady = false;
	}

	if (controller->get_digital(DIGITAL_L1))
	{
		indexer->move_velocity(-200 * indexerSpeed);
	} else if (!shotReady)
	{
		indexer->move_velocity(200 * indexerSpeed);
	} else {
		indexer->brake();
	}

	pros::screen::print(TEXT_MEDIUM, 100, 100, std::to_string(indexer->get_efficiency()).c_str());
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

	double startHeading = imuSensor->get_heading();
	pros::delay(160);
	double imuDriftPerMsec = (imuSensor->get_heading() - startHeading) / 160.0;

	syndicated::controller = new pros::Controller(pros::E_CONTROLLER_MASTER);

	double headingToMaintain = imuSensor->get_heading();

	while (true)
	{
		auto cycleStart = std::chrono::high_resolution_clock::now();

		double crx = controller->get_analog(ANALOG_RIGHT_X) / 127.0;
		double cry = controller->get_analog(ANALOG_RIGHT_Y) / 127.0;
		double clx = controller->get_analog(ANALOG_LEFT_X) / 127.0;
		double cly = controller->get_analog(ANALOG_LEFT_Y) / 127.0;

		polarPoint translationVector = polarFromCartesian(crx, cry);
		translationVector.rho = std::min(translationVector.rho, 1.0);

		if (std::abs(clx) < 0.01)
		{
			headingToMaintain = imuSensor->get_heading();
		}

		if (std::abs(crx) + std::abs(cry) < 0.01)
		{
			if (std::abs(cly) < 0.01 && std::abs(clx) < 0.01)
			{
				drivetrain->brake();
			}
			else
			{
				drivetrain->driveAndTurn(cly, imuSensor->get_heading(), clx);
			}
		}
		else if (std::abs(clx) < 0.01)
		{
			// use a PID controller

			double angleToHeadingAnticlockwise = findMod(360 + imuSensor->get_heading() - headingToMaintain, 360);
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

		handleImuReset();

		double cycleTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cycleStart).count();
		double targetTime = 16.0;
		pros::delay(std::max(0.0, targetTime - cycleTime));
		double trueTimeElapsed = std::max(cycleTime, targetTime);
		// imu sensor drift correction
		imuSensor->set_heading(imuSensor->get_heading() - (trueTimeElapsed * imuDriftPerMsec));
		timeSinceLastShot += trueTimeElapsed;
	}
}
