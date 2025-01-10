#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include <vex.h>
using namespace vex;
    // These values are in inches
extern double leftTrackingWheelDistance;
extern double rightTrackingWheelDistance;
extern double backTrackingWheelDistance;

extern double alGearRatio;

extern double wheelTravel;
extern double trackWidth;
extern double wheelBase;

extern double startingX;
extern double startingY;
extern double startingOrientation;

extern vex::brain Brain;
extern vex::controller Controller;
extern vex::motor intake;

// Initializing drivetrain vex::motors
extern vex::motor rightFront;
extern vex::motor rightMiddle;
extern vex::motor rightBack;
extern vex::motor leftFront;
extern vex::motor leftMiddle;
extern vex::motor leftBack;
extern vex::motor Arm;

extern vex::controller::button SpinIntakeForward;
extern vex::controller::button SpinIntakeBackward;
extern vex::controller::button ActivateFineControl;
extern vex::controller::button ActivateMobileGoalSolenoid;
extern vex::controller::button ReverseControlsButton;
extern vex::controller::button SpinArmForward;
extern vex::controller::button SpinArmBackward;

// vex::motor groups
extern vex::motor_group leftDriveSmart;
extern vex::motor_group rightDriveSmart;
extern vex::drivetrain Drivetrain;

extern vex::digital_out MobileGoalSolenoid;

//extern vex::controller::button SpinArmForward;
//extern vex::controller::button SpinArmBackward;

extern bool MobileGoalSolenoidIsActive;

// Tracking wheels
extern rotation leftTrackingWheel;
extern rotation rightTrackingWheel;
extern rotation backTrackingWheel;
extern positionSensing position;

#endif ROBOT_CONFIG_H
