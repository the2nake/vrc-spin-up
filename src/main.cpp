/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision1              vision        1               
// RollerMotor          motor         2               
// Drivetrain           drivetrain    3, 4, 5, 6, 8   
// Controller1          controller                    
// WindUpMotor          motor         7               
// PneumaticOut         digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

#define BLUETEAM true
#define REDTEAM false

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

float redOnTop(int measurements) {
  float confidence = 0;
  for (int i = 0; i < measurements; i++) {
    Vision1.takeSnapshot(Vision1__RED1);
    for (int j = 0; j < Vision1.objectCount; j++) {
      confidence +=
          (Vision1.objects[j].width * Vision1.objects[j].height) / 400;
    }
    Vision1.takeSnapshot(Vision1__RED2);
    for (int j = 0; j < Vision1.objectCount; j++) {
      confidence +=
          (Vision1.objects[j].width * Vision1.objects[j].height) / 400;
    }
    wait(25, msec);
  }
  return confidence / measurements;
}

float gyroDrift;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  RollerMotor.setVelocity(100, percent);
  Drivetrain.setDriveVelocity(75, percent);
  Drivetrain.setTurnVelocity(50, percent);
  WindUpMotor.setVelocity(34, percent);
  WindUpMotor.setStopping(hold);
  WindUpMotor.stop();
  
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void spinRoller(bool color) {

  Drivetrain.setDriveVelocity(50, percent);
  Drivetrain.drive(reverse);
  for (int i = 0; i < 10; i++) {
    float conf = redOnTop(5);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.setFont(monoS);
    Brain.Screen.print(conf);
    if (conf < 170) { // if there is not enough red on top
      // this function will start high if conf is low, then go to 0 when there's
      // a ton of red
      RollerMotor.spinFor(forward, 35 * cos(3.141 * conf / 170) + 35, degrees);
    }
  }
  // now it's red
  RollerMotor.stop();

  wait(200, msec);
  if (color) {
    RollerMotor.spinFor(reverse, 0.45, turns);
  }
  Drivetrain.stop();
}

void spinRollerRed() { spinRoller(REDTEAM); }

void spinRollerBlue() { spinRoller(BLUETEAM); }

void autonomous(void) {
  spinRollerRed();
  Drivetrain.driveFor(forward, 500, mm);
  Drivetrain.turnFor(right, 90, degrees);
  Drivetrain.driveFor(reverse, 650, mm);
  spinRollerRed();
  Drivetrain.driveFor(forward, 650, mm);
  Drivetrain.turnFor(left, 45, degrees);
  Drivetrain.driveFor(forward, 2880, mm);

  Drivetrain.driveFor(forward, 100, mm);
  Drivetrain.driveFor(reverse, 100, mm);

  Drivetrain.turnFor(right, 45, degrees);
  Drivetrain.driveFor(reverse, 100, mm);
  Drivetrain.turnFor(right, 90, degrees);
  Drivetrain.driveFor(reverse, 200, mm);

  spinRollerRed();
  Drivetrain.driveFor(forward, 650, mm);
  Drivetrain.turnFor(right, 90, degrees);
  Drivetrain.driveFor(reverse, 500, mm);
  spinRollerRed();
  Drivetrain.driveFor(forward, 650, mm);
  Drivetrain.driveFor(reverse, 100, mm);
  Drivetrain.turnFor(left, 40, degrees);
  PneumaticOut.set(true);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void spinRollerManualFwd() { RollerMotor.spinFor(forward, 60, degrees); }

void spinRollerManualRev() { RollerMotor.spinFor(reverse, 60, degrees); }

void windString() { WindUpMotor.spin(forward); }

void unwindString() { WindUpMotor.spin(reverse); }

void usercontrol(void) {
  // User control code here, inside the loop
  Controller1.ButtonX.pressed(spinRollerBlue);
  Controller1.ButtonL1.pressed(spinRollerManualFwd);
  Controller1.ButtonL2.pressed(spinRollerManualRev);

  while (1) {
    WindUpMotor.stop();

    if (Controller1.ButtonR1.pressing()) {
      windString();
    }
    if (Controller1.ButtonR2.pressing()) {
      unwindString();
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
