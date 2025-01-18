//I hate c++ I hate c++ I hate c++ I hate c++ I hate c++
#include "Headers/robotConfig.h"
#include "vex.h"


using namespace vex;

vex::competition Competition;
std::vector<std::vector<double>> PathPoints = {{0,0}, {0,1}};
//Pathing pathing = Pathing(PathPoints);
vex::inertial Inertial = inertial(PORT10);
bool lockArm = false;
bool FineControl = false;
bool ReverseControls = false;

void pre_auton(void) {
  Inertial.calibrate();
  while(Inertial.isCalibrating())
  {
    wait(50, msec);
  }
}


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
/// @brief Turns robot to specified heading using inertial sensor. Will calculate whether turning left or right will be faster
/// @param targetHeading Heading robot needs to turn to
/// @return When heading is within 3% of target heading
bool turnToHeading(double targetHeading, bool clockwise)
{
  rightDriveSmart.stop();
  leftDriveSmart.stop();

  if(targetHeading == 0)
  {
    targetHeading += 0.5;
  }

  double ThetaDelta = std::abs(Inertial.heading() - targetHeading);
  
  while(( std::abs(Inertial.heading() - targetHeading) >= 5))
  {
    Brain.Screen.printAt(0, 20, "Heading: %f", Inertial.heading());
    if(clockwise)
    {
      rightDriveSmart.spin(vex::reverse,  (4), volt);
      leftDriveSmart.spin(vex::forward,  (4), volt);
    } else {
      rightDriveSmart.spin(vex::forward,  (4), volt);
      leftDriveSmart.spin(vex::reverse,  (4), volt);
    }

    //vex::wait(5, msec);
  }
  rightDriveSmart.stop();
  leftDriveSmart.stop();
  return true;
}
bool SpinIntake(double time)
{
  intake.spin(forward, 12, volt);
  wait(time, msec);
  intake.stop();
  return true;
}
bool DriveDirection(int direction, double moveSpeed, double time)
{
  leftDriveSmart.spin(forward, direction * moveSpeed, volt);
  rightDriveSmart.spin(forward, 0.875 * direction * moveSpeed, volt);
  wait(time, msec);
  leftDriveSmart.stop();
  rightDriveSmart.stop();
  return true;
}
bool changeMobileGoalSolenoid()
{
  MobileGoalSolenoidIsActive = !MobileGoalSolenoidIsActive;
  MobileGoalSolenoid.set(MobileGoalSolenoidIsActive);
  wait(150, msec);
  return true;
}
void Auton32points(void)
{
  Inertial.calibrate();

  while(Inertial.isCalibrating())
  {
    wait(50, msec);
  }
  Inertial.setHeading(0, degrees);
  //First ring
  SpinIntake(500);
  //Picking up first mobilegoal
  DriveDirection(-1, 4, 300);
  turnToHeading(265, false);
  DriveDirection(1, 6, 875);
  changeMobileGoalSolenoid();
  //Intaking ring for first mobile goal
  turnToHeading(90, true);
  intake.spin(forward, 12, volt);
  DriveDirection(-1, 5, 800);
  turnToHeading(100, false);
  Inertial.setHeading(90, degrees);
  turnToHeading(120, true);
  wait(2000, msec);
  intake.stop();
  //Keep move speed low
  //Keep turn speed lower
  //Make sure battery never drops below ~70%
  //Slapping in an intertial sensor should make it go pretty quick

  //Turn around
  //drive to mobile goal
  //clamp
  //Drive to corner 
  //Drop it
  //grab other 
  //put one ring on it
  //drop it
  //Get other
  //put in corner
  //Get last one 
  //Put in corner
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
  rightDrive *= .875f;
  if(ActivateMobileGoalSolenoid.pressing())
  {
    MobileGoalSolenoidIsActive = !MobileGoalSolenoidIsActive;
    MobileGoalSolenoid.set(MobileGoalSolenoidIsActive);
  }
  if(ActivatePusherSolenoid.pressing())
  {
    PusherSolenoidIsActive = !PusherSolenoidIsActive;
    PusherSolenoid.set(PusherSolenoidIsActive);
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
  while(ActivateMobileGoalSolenoid.pressing() || ActivatePusherSolenoid.pressing())
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

void AutonWinPoint(void)
{

  DriveDirection(1, 7, 1000);
  wait(200, msec);
  changeMobileGoalSolenoid();
  SpinIntake(1000);
  wait(1000, msec);
  turnToHeading(285, true);
  intake.spin(forward, 12, volt);
  wait(200, msec);
  DriveDirection(-1, 7, 800);
  wait(1500, msec);
  intake.stop();
  wait(250, msec);
  Inertial.setHeading(0, degrees);
  turnToHeading(185, false);
  DriveDirection(-1, 6, 2000);

}
void AutonWinPointLeft(void)
{

  DriveDirection(1, 7, 1000);
  wait(200, msec);
  changeMobileGoalSolenoid();
  SpinIntake(1000);
  wait(1000, msec);
  intake.spin(reverse, 12, volt);
  wait(500, msec);
  intake.stop();
  turnToHeading(70, false);
  intake.spin(forward, 12, volt);
  wait(200, msec);
  DriveDirection(-1, 7, 800);
  wait(1300, msec);
  intake.stop();
  wait(250, msec);
  Inertial.setHeading(0, degrees);
  turnToHeading(180, true);
  DriveDirection(-1, 6, 2000);

}
void usercontrol(void) {
  // leftDriveSmart.spin(vex::forward);
  // rightDriveSmart.spin(vex::forward);
  // intake.spin(vex::forward);

  while (1) {
    usercontrolcode();
    vex::wait(5, msec);
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
void autonomous(void) {
  AutonWinPoint();
}
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  // Run the preautonomous function.
 
  pre_auton();

  while (true) {

    vex::wait(10, msec);
  }
}
