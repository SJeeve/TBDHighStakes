#include "vex.h"
#include "positionsensing.cpp"
using namespace vex;

competition Competition;
brain Brain;
controller Controller;

//ALL MEASUREMENTS IN INCHES UNLESS STATED OTHERWISE

//Intake motors
motor intake = motor(PORT11, false);

//Initializing drivetrain motors
motor rightFront = motor(PORT12, true);
motor rightMiddle = motor(PORT13, true);
motor rightBack = motor(PORT14, true);
motor leftFront = motor(PORT15, false);
motor leftMiddle = motor(PORT16, false);
motor leftBack = motor(PORT17, false);

//Motor groups
motor_group leftDriveSmart = motor_group(leftFront, leftMiddle, leftBack);
motor_group rightDriveSmart = motor_group(rightFront, rightMiddle, rightBack);

//These values are used to construct the drivetrain object
const float wheelTravel = 2.75 * M_PI;
const float trackWidth = 18;
const float wheelBase = 18;
const float externalGearRatio = 1;

//Drivetrain object
drivetrain Drivetrain = drivetrain(leftDriveSmart, rightDriveSmart, wheelTravel, trackWidth, wheelBase, inches, externalGearRatio);

//Pneumatic stuff
digital_out MobileGoalSolenoid = digital_out(Brain.ThreeWirePort.A);
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

//Change controls here
const vex::controller::button SpinIntakeForwardButton = Controller.ButtonR2; 
const vex::controller::button SpinIntakeBackwardButton = Controller.ButtonL2;
const vex::controller::button ActivateFineControlButton = Controller.ButtonX;
const vex::controller::button ActivateMobileGoalSolenoidButton = Controller.ButtonA;


positionsensing position = positionsensing(startingX, startingY, leftTrackingWheelDistance, rightTrackingWheelDistance, backWheelTrackingWheelDistance, startingOrientation);

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
  position.positionsensing::UpdatePosition(leftTrackingWheel.position(degrees), rightTrackingWheel.position(degrees), backTrackingWheel.position(degrees));
  resetRotationSensors();
  vex::wait(20, msec);
}

void usercontrol(void) {
  bool FineControl = false;
  leftDriveSmart.spin(vex::forward);
  rightDriveSmart.spin(vex::forward);
  intake.spin(vex::forward);
  while (1) {
    float leftDrive = Controller.Axis4.position() - Controller.Axis1.position();
    float rightDrive = Controller.Axis4.position() + Controller.Axis1.position();

    //If we decide to keep this I would want an LED so it's easier to tell when it's on or off    
    if(ActivateFineControlButton.pressing())
      FineControl = !FineControl;
    
    if(FineControl){
      leftDrive = pow(leftDrive, 3);
      rightDrive = pow(rightDrive, 3);
    }

    if(ActivateMobileGoalSolenoidButton.pressing())
    {
      if(MobileGoalSolenoidIsActive)
        MobileGoalSolenoid.set(false);
      else 
        MobileGoalSolenoid.set(true); 
      MobileGoalSolenoidIsActive = !MobileGoalSolenoidIsActive;
    }
      
    leftDriveSmart.spin(vex::forward, leftDrive * 6, volt);
    rightDriveSmart.spin(vex::forward, rightDrive * 6, volt);

    if(SpinIntakeForwardButton.pressing())
      intake.spin(vex::forward, 12, volt);
    else if(SpinIntakeBackwardButton.pressing())
      intake.spin(vex::forward, -12, volt);
    else
      intake.stop();

    vex::wait(20, msec);  
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
