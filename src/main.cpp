#include "vex.h"
using namespace vex;

vex::competition Competition;
brain Brain;
vex::controller Controller = vex::controller();

vex::motor conveyorBelt = vex::motor(PORT4, false);
vex::motor intake = vex::motor(PORT3, false);
//Initializing drivetrain vex::motors
vex::motor rightFront = vex::motor(PORT12, true);
vex::motor rightMiddle = vex::motor(PORT13, true);
vex::motor rightBack = vex::motor(PORT14, true);
vex::motor leftFront = vex::motor(PORT15, false);
vex::motor leftMiddle = vex::motor(PORT16, false);
vex::motor leftBack = vex::motor(PORT17, false);
//vex::motor groups
vex::motor_group leftDriveSmart = vex::motor_group(leftFront, leftMiddle, leftBack);
vex::motor_group rightDriveSmart = vex::motor_group(rightFront, rightMiddle, rightBack);

//These values are in inches
float wheelTravel = 2.75 * M_PI;
float trackWidth = 18;
float wheelBase = 18;
bool FineControl = false;
bool ReverseControls = false;
//Gear ratio
//Forgot if it's driven to driver or the other way around
float externalGearRatio = 1;

vex::drivetrain Drivetrain = vex::drivetrain(leftDriveSmart, rightDriveSmart, wheelTravel, trackWidth, wheelBase, inches, externalGearRatio);

vex::digital_out MobileGoalSolenoid = vex::digital_out(Brain.ThreeWirePort.A);
vex::digital_out FineControlLED = vex::digital_out(Brain.ThreeWirePort.B);
//Change controls here

const vex::controller::button SpinIntakeForward = Controller.ButtonR1; 
const vex::controller::button SpinIntakeBackward = Controller.ButtonL1;
const vex::controller::button ActivateFineControl = Controller.ButtonX;
const vex::controller::button ActivateMobileGoalSolenoid = Controller.ButtonA;
const vex::controller::button ReverseControlsButton = Controller.ButtonB;

bool MobileGoalSolenoidIsActive = false;

//Edit these values for different starting positions
const float startingX = 0;
const float startingY = 0;
const float startingOrientation = 0;

//These should stay the same for every version, unless changes are made to the tracking wheels' position
const float leftTrackingWheelDistance = 1;
const float rightTrackingWheelDistance = 1;
const float backWheelTrackingWheelDistance = 1;

//Tracking wheels
rotation leftTrackingWheel = rotation(PORT1, false);
rotation rightTrackingWheel = rotation(PORT2, false);
rotation backTrackingWheel = rotation(PORT3, false);

positionSensing position = positionSensing(startingX, startingY, leftTrackingWheelDistance, rightTrackingWheelDistance, backWheelTrackingWheelDistance, startingOrientation);

void resetRotationSensors()
{
  leftTrackingWheel.resetPosition();
  rightTrackingWheel.resetPosition();
  backTrackingWheel.resetPosition();
}

void pre_auton(void) { 
  resetRotationSensors();
}



void autonomous(void) {
  position.positionSensing::UpdatePosition(leftTrackingWheel.position(degrees), rightTrackingWheel.position(degrees), backTrackingWheel.position(degrees));
  resetRotationSensors();
  vex::wait(20, msec);
}

void usercontrol(void) {
leftDriveSmart.spin(forward);
  rightDriveSmart.spin(forward);
  intake.spin(forward);
  while (1) {
    float leftDrive = Controller.Axis3.position();
    float rightDrive = Controller.Axis3.position();
    if(!ReverseControls)
    {
      leftDrive *= -1;
      rightDrive *= -1;
    }  
    leftDrive -= Controller.Axis1.position();
    rightDrive += Controller.Axis1.position();
    if(ActivateMobileGoalSolenoid.pressing())
    {
      MobileGoalSolenoidIsActive = !MobileGoalSolenoidIsActive;
      MobileGoalSolenoid.set(MobileGoalSolenoidIsActive);
    }
    //If we decide to keep this I would want an LED so it's easier to tell when it's on or off
    if(ActivateFineControl.pressing())
    {
      FineControl = !FineControl;
      FineControlLED.set(FineControl);
    }

    if(ReverseControlsButton.pressing())
    {
      ReverseControls = !ReverseControls;
    }

    if(FineControl){
      leftDrive = pow(leftDrive, 3);
      rightDrive = pow(rightDrive, 3);
    }




    leftDriveSmart.spin(forward, leftDrive * 6, volt);
    rightDriveSmart.spin(forward, rightDrive * 6, volt);

    //kinda shitty code but it should work
    if(SpinIntakeForward.pressing())
    {
      intake.spin(forward, 12, volt);
      conveyorBelt.spin(forward, -12, volt);
    } else if(SpinIntakeBackward.pressing())
    {
      intake.spin(forward, -12, volt);
      conveyorBelt.spin(forward, 12, volt);
    } else {
      intake.stop();
      conveyorBelt.stop();
    }
    while(ActivateMobileGoalSolenoid.pressing() || ReverseControlsButton.pressing())
      vex::wait(20, msec);
    vex::wait(30, msec);  
  }
}


int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  while (true) {
    vex::wait(100, msec);
  }
}
