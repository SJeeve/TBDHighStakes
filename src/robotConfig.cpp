#include "Headers/robotConfig.h"
#include "vex.h"
using namespace vex;

vex::brain Brain = vex::brain();
double leftTrackingWheelDistance = 6.5;
double rightTrackingWheelDistance = 6;
double backTrackingWheelDistance = 6.5;

double wheelTravel = 2.75 * M_PI;
double trackWidth = 16;
double wheelBase = 12;
FalconPathPlanner smoothedPath;
double alGearRatio = 5 / 3;
vex::controller Controller = vex::controller();
vex::motor intake = vex::motor(PORT4, ratio18_1,true);

// initializing Drivetrain vex:::motors
vex::motor rightFront = vex::motor(PORT12, ratio6_1, true);
vex::motor rightMiddle = vex::motor(PORT13, ratio6_1, true);
vex::motor rightBack = vex::motor(PORT14, ratio6_1, true);
vex::motor leftFront = vex::motor(PORT15, ratio6_1, false);
vex::motor leftMiddle = vex::motor(PORT16, ratio6_1, false);
vex::motor leftBack = vex::motor(PORT17, ratio6_1, false);
vex::motor Arm = vex::motor(PORT20, ratio36_1,false);

// vex::motor groups
vex::motor_group leftDriveSmart = vex::motor_group(leftFront, leftMiddle, leftBack);
vex::motor_group rightDriveSmart = vex::motor_group(rightFront, rightMiddle, rightBack);
vex::drivetrain Drivetrain = vex::drivetrain(leftDriveSmart, rightDriveSmart, wheelTravel, trackWidth, wheelBase, inches, alGearRatio);

vex::digital_out MobileGoalSolenoid = vex::digital_out(Brain.ThreeWirePort.A);
vex::digital_out PusherSolenoid = vex::digital_out(Brain.ThreeWirePort.H);
// change controls here
vex::controller::button SpinIntakeForward = Controller.ButtonR1;
vex::controller::button SpinIntakeBackward = Controller.ButtonR2;
vex::controller::button ActivateFineControl = Controller.ButtonX;
vex::controller::button ActivateMobileGoalSolenoid = Controller.ButtonA;
vex::controller::button ReverseControlsButton = Controller.ButtonB;
vex::controller::button SpinArmForward = Controller.ButtonL1;
vex::controller::button SpinArmBackward = Controller.ButtonL2;
vex::controller::button ActivatePusherSolenoid = Controller.ButtonX;

bool MobileGoalSolenoidIsActive = false;
bool PusherSolenoidIsActive = false;
// tracking wheels
vex::rotation leftTrackingWheel = vex::rotation(PORT5, true);
vex::rotation rightTrackingWheel = vex::rotation(PORT8, true);
vex::rotation backTrackingWheel = vex::rotation(PORT19, true);
//positionSensing position = positionSensing(1.0, 1.0, 0.0);





