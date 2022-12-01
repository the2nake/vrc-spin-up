using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern signature Vision1__RED1;
extern signature Vision1__RED2;
extern signature Vision1__SIG_3;
extern signature Vision1__SIG_4;
extern signature Vision1__SIG_5;
extern signature Vision1__SIG_6;
extern signature Vision1__SIG_7;
extern vision Vision1;
extern motor RollerMotor;
extern smartdrive Drivetrain;
extern controller Controller1;
extern motor WindUpMotor;
extern digital_out PneumaticOut;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );