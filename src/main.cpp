//I hate c++ I hate c++ I hate c++ I hate c++ I hate c++
#include "Headers/robotConfig.h"
#include "vex.h"


using namespace vex;

vex::competition Competition;
std::vector<std::vector<double>> PathPoints = {{0,0}, {0,1}};
//Pathing pathing = Pathing(PathPoints);

bool lockArm = false;
bool FineControl = false;
bool ReverseControls = false;

void pre_auton(void) {
  //pathing.resetRotationSensors();
}

void updateTracking()
{

}
/*void test_serial_output()
{
  double count = 0;
  while(true)
  {
    Brain.Screen.print("THingy %f", 4);
    printf("%f\n",count);
    fflush(stdout);
    wait(100,msec);
    count++;
  }
}*/
void lockArmInPlace()
{
  lockArm = true;
  Arm.stop(hold);
}
void comeToRest()
{
  // rightDriveSmart.spin(forward, 0, volt);
  // leftDriveSmart.spin(forward, 0, volt);

  // vex::wait(50, msec);

}
void otherAuton()
{
  double moveSpeed = 8;
  double travelUnit = 350;
  double turnUnit = 3;


  intake.spin(forward, 12, volt);
  vex::wait(1000, msec);
  
  rightDriveSmart.spin(reverse, moveSpeed, volt);
  leftDriveSmart.spin(reverse, moveSpeed, volt);
  vex::wait(travelUnit * 1.5, msec);

  rightDriveSmart.spin(reverse, moveSpeed, volt);
  leftDriveSmart.spin(forward, moveSpeed, volt);
  vex::wait(turnUnit * 90, msec);

  rightDriveSmart.spin(forward, moveSpeed, volt);
  leftDriveSmart.spin(forward, moveSpeed, volt);
  vex::wait(travelUnit * 1.35, msec);

  vex::wait(100, msec);
  MobileGoalSolenoid.set(true);

  rightDriveSmart.spin(forward, moveSpeed, volt);
  leftDriveSmart.spin(reverse, moveSpeed, volt);
  vex::wait(turnUnit * 190, msec);

  rightDriveSmart.spin(reverse, moveSpeed, volt);
  leftDriveSmart.spin(reverse, moveSpeed, volt);
  vex::wait(travelUnit * 2.5, msec);
  rightDriveSmart.stop();
  leftDriveSmart.stop();
  intake.spin(forward, 12, volt);
  rightDriveSmart.spin(reverse, 2, volt);
  leftDriveSmart.spin(reverse, 2, volt);
  vex::wait(3000, msec);

  rightDriveSmart.spin(reverse, moveSpeed, volt);
  leftDriveSmart.spin(forward, moveSpeed, volt);
  vex::wait(turnUnit * 140, msec);

  rightDriveSmart.spin(forward, moveSpeed, volt);
  leftDriveSmart.spin(forward, moveSpeed, volt);
  vex::wait(travelUnit * 5, msec);
  rightDriveSmart.stop();
  leftDriveSmart.stop();
  vex::wait(1000, msec);
  intake.spin(reverse, 12, volt);
  MobileGoalSolenoid.set(false);
  vex::wait(500, msec);
  rightDriveSmart.spin(reverse, 4, volt);
  leftDriveSmart.spin(reverse, 4, volt);
  vex::wait(1000, msec);

  rightDriveSmart.stop();
  leftDriveSmart.stop();
}
void shittyerAuton()
{

  rightDriveSmart.spin(forward);
  double moveSpeed = 8;
  double travelUnit = 350;
  double turnUnit = 3;

  intake.spin(forward, 8, volt);

  rightDriveSmart.spin(forward, moveSpeed, volt);
  leftDriveSmart.spin(forward, moveSpeed, volt);
  vex::wait(travelUnit, msec);

  
  MobileGoalSolenoid.set(true);
  leftDriveSmart.spin(reverse,  moveSpeed, volt);
  rightDriveSmart.spin(forward, moveSpeed, volt);
  vex::wait(turnUnit * (180 - 19.7468), msec);
  Brain.Screen.print("here");

    leftDriveSmart.stop();
  rightDriveSmart.stop();
  leftDriveSmart.spin(forward, moveSpeed, volt);
  rightDriveSmart.spin(forward, moveSpeed, volt  );
  vex::wait(travelUnit * 2.302, msec);

  
  MobileGoalSolenoid.set(false);
  rightDriveSmart.spin(reverse, 2  / 3 * moveSpeed, volt);
  leftDriveSmart.spin(forward, 2  / 3 * moveSpeed, volt);
  vex::wait(turnUnit * 11.956, msec);

  
  rightDriveSmart.spin(reverse, moveSpeed, volt);
  leftDriveSmart.spin(reverse, moveSpeed, volt);
  vex::wait(travelUnit * 4.664, msec);

  rightDriveSmart.spin(reverse, 2  / 3 * moveSpeed, volt);
  leftDriveSmart.spin(forward, 2  / 3 * moveSpeed, volt);
  vex::wait(turnUnit * 180, msec);

  
  rightDriveSmart.stop();
  leftDriveSmart.stop();
  MobileGoalSolenoid.set(true);
  vex::wait(100, msec); 

  
  rightDriveSmart.spin(forward, 2  / 3 * moveSpeed, volt);
  leftDriveSmart.spin(reverse, 2  / 3 * moveSpeed, volt);
  vex::wait(turnUnit * 227, msec);

  
  rightDriveSmart.spin(reverse, moveSpeed, volt);
  leftDriveSmart.spin(reverse, moveSpeed, volt);
  vex::wait(500, msec);

  
}

void shittyAuton(void)
{
 /* MobileGoalSolenoid.set(false);
  intake.spin(forward);
  intake.setVelocity(0, percent);
  Drivetrain.setDriveVelocity(80, percent);
  Drivetrain.setTurnVelocity(70, percent);
  Drivetrain.drive(forward);
  vex::wait(800, msec);
  Drivetrain.stop();
  vex::wait(500, msec);
  MobileGoalSolenoid.set(true);
  MobileGoalSolenoidIsActive = true;
  vex::wait(500, msec);
  Drivetrain.stop();
  intake.setVelocity(100, percent);
  intake.spin(forward);
  vex::wait(3, sec);
  Drivetrain.driveFor(reverse, 30, inches, true);
  Drivetrain.stop();
  intake.stop();*/
}



void autonomous(void) {
  //pathing.Path();
}

void usercontrolcode()
{
  double leftDrive = Controller.Axis3.position()/100.0;
  double rightDrive = Controller.Axis3.position()/100.0;
  double* leftDrivePtr = &leftDrive;
  double* rightDrivePtr = &rightDrive; 

  if(!ReverseControls)
  {
    leftDrive *= -1;
    rightDrive *= -1;
  }  
  leftDrive -= 0.8f * Controller.Axis1.position()/100.0;
  rightDrive += 0.8f * Controller.Axis1.position()/100.0;
  if(ActivateMobileGoalSolenoid.pressing())
  {
    MobileGoalSolenoidIsActive = !MobileGoalSolenoidIsActive;
    MobileGoalSolenoid.set(MobileGoalSolenoidIsActive);
  }
  //If we decide to keep this I would want an LED so it's easier to tell when it's on or off
  if(ActivateFineControl.pressing())
  {
    FineControl = !FineControl;
  }

  if(ReverseControlsButton.pressing())
  {
    ReverseControls = !ReverseControls;
  }

  if(FineControl){
    leftDrive = pow(leftDrive, 3);
    rightDrive = pow(rightDrive, 3);
  }

  leftDriveSmart.spin(vex::forward, leftDrive * 12, volt);
  rightDriveSmart.spin(vex::forward, rightDrive * 12, volt);

  //kinda shitty code but it'll work
  if(SpinIntakeForward.pressing())
  {
    intake.spin(vex::forward, 12, volt);
  } else if(SpinIntakeBackward.pressing())
  {
    intake.spin(vex::forward, -12, volt);
  } else {
    intake.stop();
  }

  if(SpinArmForward.pressing())
  {
    Arm.spin(forward, 8, volt);
    lockArm = false;
  } else if(SpinArmBackward.pressing())
  {
    Arm.spin(forward, -6, volt);
    lockArm = false;
  } else if(!lockArm) {
    Arm.stop();
  }

  SpinArmForward.released(lockArmInPlace);
  while(ActivateMobileGoalSolenoid.pressing())
  {
    vex::wait(20, msec);
  }
  while(ReverseControlsButton.pressing())
  {
    vex::wait(20, msec);
  }
  delete leftDrivePtr;
  delete rightDrivePtr;
}


void usercontrol(void) {
  // leftDriveSmart.spin(vex::forward);
  // rightDriveSmart.spin(vex::forward);
  // intake.spin(vex::forward);
    //position.resetRotationSensors();
  while (1) {
    usercontrolcode();
    //position.UpdatePosition();
    vex::wait(10, msec);

  }
}

int myTask()
{
  while(true)
  {
    Brain.Screen.print("I am myTask");
    vex::task::sleep(25);
  }

  return 0;

}
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  // Run the preautonomous function.
 
  pre_auton();

  while (true) {
    Brain.Screen.printAt(100, 30, "Left Tracking: %0.3f", leftTrackingWheel.position(degrees));
    Brain.Screen.printAt(100, 50, "Right Tracking: %0.3f", rightTrackingWheel.position(degrees));
    Brain.Screen.printAt(100, 70, "Back Tracking: %0.3f", backTrackingWheel.position(degrees));
    vex::wait(10, msec);
  }
}
