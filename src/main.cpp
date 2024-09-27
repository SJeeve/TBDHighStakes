#include "vex.h"
#include "Header.hpp"
using namespace vex;
// A global instance of competition
competition Competition;
//Edit these values for different starting positions
const long startingX = 0;
const long startingY = 0;
//These should stay the same for every version, unless changes are made to the tracking wheels' position
const long leftTrackingWheelDistance = 1;
const long rightTrackingWheelDistance = 1;
const long backWheelTrackingWheelDistance = 1;
//Assign ports for tracking wheels here
const int32_t leftTrackingPort = PORT1;
const int32_t rightTrackingPort = PORT2;
const int32_t backTrackingPort = PORT3;
vex::rotation leftTrackingWheel = vex::rotation(leftTrackingPort, false);
vex::rotation rightTrackingWheel = vex::rotation(rightTrackingPort, false);
vex::rotation backTrackingWheel = vex::rotation(backTrackingPort, false);
PositionSensing position = PositionSensing(startingX, startingY, leftTrackingWheelDistance, rightTrackingWheelDistance, backWheelTrackingWheelDistance);

void pre_auton(void) { 
  resetRotationSensors();
}

void resetRotationSensors()
{
  leftTrackingWheel.resetPosition();
  rightTrackingWheel.resetPosition();
  backTrackingWheel.resetPosition();
}

void autonomous(void) {
  position.UpdatePosition(leftTrackingWheel.angle(degrees), rightTrackingWheel.angle(degrees), backTrackingWheel.angle(degrees));
  resetRotationSensors(); 
  wait(0.02, seconds);
}

void usercontrol(void) {
  while (1) {
    wait(20, msec);
  }
}


int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
