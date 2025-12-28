#include "vex.h"
#include "vex_motor.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <thread>
#include <string>
using namespace vex;

class RPMEntryData {
  public:
  double rpm;
  double timestamp;
};

competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/



motor leftMotor1 = motor(PORT11, ratio6_1, true);
motor leftMotor2 = motor(PORT12, ratio6_1, true);
motor leftMotor3 = motor(PORT13, ratio6_1, true);
motor rightMotor1 = motor(PORT20, ratio6_1, false);
motor rightMotor2 = motor(PORT19, ratio6_1, false);
motor rightMotor3 = motor(PORT18, ratio6_1, false);
motor intakeMotor = motor(PORT9, ratio6_1, false); // forward for intake, reverse for outake
motor outtakeMotor = motor(PORT10, ratio6_1, true); // forward for intake, reverse for outake
controller Controller = controller(primary);

motor_group LeftDriveSmart = motor_group(leftMotor1, leftMotor2, leftMotor3);
motor_group RightDriveSmart = motor_group(rightMotor1, rightMotor2, rightMotor3);

bool matchloader_status = false;
bool outputblocker_status = false;

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_NO_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(leftMotor1, leftMotor2, leftMotor3),

//Right Motors:
motor_group(rightMotor1, rightMotor2, rightMotor3),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT5,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.6,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

int current_auton_selection = 0;
bool auto_started = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "Auton 1");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "Auton 2");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "Auton 3");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "Auton 4");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "Auton 5");
        break;
      case 5:
        Brain.Screen.printAt(5, 140, "Auton 6");
        break;
      case 6:
        Brain.Screen.printAt(5, 140, "Auton 7");
        break;
      case 7:
        Brain.Screen.printAt(5, 140, "Auton 8");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    wait(10, msec);
  }
}



 void loader() {
  chassis.set_heading(0);
  // start directly in front of loader for testing
  LeftDriveSmart.setVelocity(100, percent);
  RightDriveSmart.setVelocity(100, percent);
  // intakes from loader
  matchloader.set(true);
  intakeMotor.spin(forward, 100, percent);
  chassis.set_drive_exit_conditions(1, 300, 500);
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_distance(17);
  wait(0.6, seconds);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  chassis.drive_distance(-10);
  chassis.turn_to_angle(-190);
  wait(0.4, seconds);
  matchloader.set(false);
  chassis.drive_distance(16);

  // loads long goal
  intakeMotor.spin(forward, 90, percent);
  outtakeMotor.spin(forward, 100, percent);
}


void leftMiddle() {
  outputblocker.set(true);
  matchloader.set(false);
  //move forward 28.7 inches
  // starts from park zone corner, the bot's left bottom corner touches the park zone's right top corner
  chassis.set_drive_constants(9, 1.5, 0, 10, 0);
  // goes to and intakes 3 balls
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);             
  intakeMotor.spin(forward, 100, percent);
  chassis.drive_distance(13.842);
  chassis.turn_to_angle(-32.1);
  //drives to center to get the balls
  LeftDriveSmart.setVelocity(25, percent);
  RightDriveSmart.setVelocity(25, percent);
  chassis.drive_distance(4.72);
  matchloader.set(true);
  chassis.drive_distance(8.72);
  chassis.drive_distance(-2);
  intakeMotor.stop();
  matchloader.set(false);
  //stops intake
  chassis.turn_to_angle(-32.1-106.5);
  LeftDriveSmart.setVelocity(100, percent);
  RightDriveSmart.setVelocity(100, percent);
  chassis.drive_distance(39.8);
  chassis.turn_to_angle(-180);
  //goes to loader
  intakeMotor.spin(forward, 100, percent);
  chassis.drive_distance(-5);
  //drops matchloader and collects balls
  matchloader.set(true);
  wait(0.5, seconds);
  LeftDriveSmart.setVelocity(67, percent);
  RightDriveSmart.setVelocity(67, percent);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(10.15);
  wait(0.5, seconds);
  chassis.drive_distance(-5.15);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  matchloader.set(false);
  //drives back and pulls matchloader up
  intakeMotor.stop();
  //stops intake
  LeftDriveSmart.setVelocity(100, percent);
  RightDriveSmart.setVelocity(100, percent);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(-22);
  intakeMotor.spin(forward, 100, percent);
  outtakeMotor.spin(forward, 100, percent);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
}
void rightMiddle() {
  outputblocker.set(true);
  matchloader.set(false);
  //move forward 28.7 inche
  // starts from park zone corner, the bot's left bottom corner touches the park zone's right top corner
  chassis.set_drive_constants(9, 1.5, 0, 10, 0);
  // goes to and intakes 3 balls
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);             
  intakeMotor.spin(forward, 100, percent);
  chassis.drive_distance(13.842);
  chassis.turn_to_angle(32.1);
  //drives to center to get the balls
  LeftDriveSmart.setVelocity(25, percent);
  RightDriveSmart.setVelocity(25, percent);
  chassis.drive_distance(4.72);
  matchloader.set(true);
  chassis.drive_distance(8.72);
  chassis.drive_distance(-2);
  intakeMotor.stop();
  matchloader.set(false);
  //stops intake
  chassis.turn_to_angle(32.1+106.5);
  LeftDriveSmart.setVelocity(100, percent);
  RightDriveSmart.setVelocity(100, percent);
  chassis.drive_distance(39.8);
  chassis.turn_to_angle(180);
  //goes to loader
  intakeMotor.spin(forward, 100, percent);
  chassis.drive_distance(-5);
  //drops matchloader and collects balls
  matchloader.set(true);
  wait(0.5, seconds);
  LeftDriveSmart.setVelocity(67, percent);
  RightDriveSmart.setVelocity(67, percent);
  // Sets the chassis' drive exit conditions:
  // (1.5, 300, 500) = Stop driving if error is less than 1.5 inches, or speed is less than 300 deg/sec, or after 500 ms timeout.
  // This function ensures the robot stops precisely or safely, preventing overruns in the physical field.
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(10.15);
  wait(0.5, seconds);
  chassis.drive_distance(-5.15);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  matchloader.set(false);
  //drives back and pulls matchloader up
  intakeMotor.stop();
  //stops intake
  LeftDriveSmart.setVelocity(100, percent);
  RightDriveSmart.setVelocity(100, percent);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(-22);
  intakeMotor.spin(forward, 100, percent);
  outtakeMotor.spin(forward, 100, percent);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  // goes and scores in long goal

  // // go to middle goal bottom
  // LeftDriveSmart.setVelocity(35, percent);
  // RightDriveSmart.setVelocity(35, percent);
  // chassis.turn_to_angle(-45);
  // chassis.drive_distance(9);
  // // loads bottom middle goal
  // intakeMotor.spin(reverse, 60, percent);
  // wait(2.1, seconds);
  // LeftDriveSmart.setVelocity(90, percent);
  // RightDriveSmart.setVelocity(90, percent);
  // chassis.drive_distance(2.5);

  // // goes to right loader and turns around to intake from loader
  // chassis.drive_distance(-46.5);
  // chassis.turn_to_angle(180);
  
  // loader();
}


void cheese() {
  // descores parking zone and parks
  chassis.set_drive_constants(9, 1.5, 0, 10, 0);
  LeftDriveSmart.setVelocity(80,percent);
  RightDriveSmart.setVelocity(80,percent);
  chassis.drive_distance(18);
}

void spicyCheese() {
  // descores parking zone and parks
  chassis.set_drive_constants(9, 1.5, 0, 10, 0);
  LeftDriveSmart.setVelocity(80,percent);
  RightDriveSmart.setVelocity(80,percent);
  chassis.drive_distance(18);
  wait(1, sec);
  chassis.drive_distance(-10);
}
void spicyCheeseWithJalapeñoBits() {
  // descores parking zone and parks
  chassis.set_drive_constants(9, 1.5, 0, 10, 0);
  LeftDriveSmart.setVelocity(80,percent);
  RightDriveSmart.setVelocity(80,percent);
  chassis.drive_distance(18);
  wait(1, sec);
  chassis.drive_distance(-10);
  wait(0.2, sec);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(12);
  intakeMotor.spin(forward, 100, percent);
  outtakeMotor.spin(forward, 100, percent);
  matchloader.set(true);
}
// if you need the accented n for future purposes: ñ
/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void skills() {
  // chassis.set_drive_constants(9, 1.5, 0, 10, 0);
  // LeftDriveSmart.setVelocity(80,percent);
  // RightDriveSmart.setVelocity(80,percent);
  // chassis.drive_distance(46.654);
  // wait(0.5, seconds);
  // chassis.turn_to_angle(270);
  // wait(0.3, seconds);
  // //* INTAKE FROM FIRST LOADER *//
  // // activate matchloader
  // matchloader.set(true);
  // // start intake
  // intakeMotor.spin(forward, 100, percent);
  // chassis.drive_distance(12.223);
  // wait(1.5, seconds);
  // matchloader.set(false);
  // wait(0.5, seconds);
  // chassis.drive_distance(-12.223);
  // intakeMotor.stop();
  // wait(0.5, seconds);
  // // stop intake
  // chassis.turn_to_angle(180);
  // wait(0.5, seconds);
  // chassis.drive_distance(12.989);
  // wait(0.5, seconds);
  // chassis.turn_to_angle(90);
  // wait(0.5, seconds);
  // chassis.drive_distance(89.534);
  // wait(0.5, seconds);
  // chassis.turn_to_angle(0);
  // wait(0.5, seconds);
  // chassis.drive_distance(13.96);
  // wait(0.5, seconds);
  // chassis.turn_to_angle(90);
  // wait(0.5, seconds);
  // LeftDriveSmart.setVelocity(70, percent);
  // RightDriveSmart.setVelocity(70, percent);
  // chassis.drive_distance(-12.366);
  // wait(.5, seconds);
  // //* OUTTAKE FIRST 6 BALLS INTO LONG GOAL *//
  // outtakeMotor.spin(forward, 100, percent);
  // intakeMotor.spin(forward, 100, percent);
  // wait(2, seconds);
  // outtakeMotor.stop();
  // //* INTAKE FROM SECOND LOADER *//
  // matchloader.set(true);
  // LeftDriveSmart.setVelocity(80, percent);
  // RightDriveSmart.setVelocity(80, percent);
  // wait(0.5, seconds);
  // chassis.drive_distance(23);
  // LeftDriveSmart.setVelocity(50, percent);
  // RightDriveSmart.setVelocity(50, percent);
  // wait(0.1, seconds);
  // chassis.drive_distance(4.8);
  // wait(1.5, seconds);
  // intakeMotor.stop();
  // LeftDriveSmart.setVelocity(80, percent);
  // RightDriveSmart.setVelocity(80, percent);
  // chassis.drive_distance(23);
  // LeftDriveSmart.setVelocity(50, percent);
  // RightDriveSmart.setVelocity(50, percent);
  // wait(0.1, seconds);
  // chassis.drive_distance(4.8);
  // wait(0.5, seconds);
  // outtakeMotor.spin(forward, 100, percent);
  // intakeMotor.spin(forward, 100, percent);
  // wait(2, seconds);
  // outtakeMotor.stop();
  // intakeMotor.stop();
  
  // //* END OF FIRST HALF OF SKILLS DRIVE COURSE *//

  chassis.set_drive_constants(9, 1.5, 0, 10, 0);
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);
  chassis.drive_distance(47.26);
  chassis.turn_to_angle(90);
  matchloader.set(true);
  LeftDriveSmart.setVelocity(67,percent);
  RightDriveSmart.setVelocity(67,percent);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(11.187);
  intakeMotor.spin(forward, 100, percent);
  wait(1.5, seconds);
  chassis.drive_distance(-11.187);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  matchloader.set(false);
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);
  chassis.turn_to_angle(0);
  chassis.drive_distance(11.187);
  chassis.turn_to_angle(-90);
  wait(0.2, seconds);
  chassis.drive_distance(85.158);
  chassis.turn_to_angle(180);
  chassis.drive_distance(11.187);
  chassis.turn_to_angle(-90);
  LeftDriveSmart.setVelocity(67,percent);
  RightDriveSmart.setVelocity(67,percent);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(-9.36);
  outtakeMotor.spin(forward, 100, percent);
  intakeMotor.spin(forward, 100, percent);
  wait(2, seconds);
  outtakeMotor.stop();
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);
  chassis.drive_distance(18);
  matchloader.set(true);
  LeftDriveSmart.setVelocity(67,percent);
  RightDriveSmart.setVelocity(67,percent);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(10);
  wait(1.5, seconds);
  intakeMotor.stop();
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);
  chassis.drive_distance(-10);
  matchloader.set(false);
  LeftDriveSmart.setVelocity(67,percent);
  RightDriveSmart.setVelocity(67,percent);
  chassis.drive_distance(-18);
  outtakeMotor.spin(forward, 100, percent);
  intakeMotor.spin(forward, 100, percent);
  wait(2,seconds);
  intakeMotor.stop();
  outtakeMotor.stop():
  LeftDriveSmart.setVelocity(50,percent);
  RightDriveSmart.setVelocity(50,percent)
  chassis.drive_distance(15.515);
  chassis.turn_to_angle(0);
  chassis.drive_distance(45.26);
  
  //* END OF FIRST HALF OF SKILLS DRIVE COURSE *//
  
  chassis.set_drive_constants(9, 1.5, 0, 10, 0);
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);
  chassis.drive_distance(47.26);
  chassis.turn_to_angle(90);
  matchloader.set(true);
  LeftDriveSmart.setVelocity(67,percent);
  RightDriveSmart.setVelocity(67,percent);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(11.287);
  intakeMotor.spin(forward, 100, percent);
  wait(1.5, seconds);
  chassis.drive_distance(-11.287);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  matchloader.set(false);
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);
  chassis.turn_to_angle(0);
  chassis.drive_distance(11.187);
  chassis.turn_to_angle(-90);
  wait(0.2, seconds);
  chassis.drive_distance(85.158);
  chassis.turn_to_angle(180);
  chassis.drive_distance(11.187);
  chassis.turn_to_angle(-90);
  LeftDriveSmart.setVelocity(67,percent);
  RightDriveSmart.setVelocity(67,percent);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(-9.36);
  outtakeMotor.spin(forward, 100, percent);
  intakeMotor.spin(forward, 100, percent);
  wait(2, seconds);
  outtakeMotor.stop();
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);
  chassis.drive_distance(18);
  matchloader.set(true);
  LeftDriveSmart.setVelocity(67,percent);
  RightDriveSmart.setVelocity(67,percent);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(10);
  wait(1.5, seconds);
  intakeMotor.stop();
  LeftDriveSmart.setVelocity(100,percent);
  RightDriveSmart.setVelocity(100,percent);
  chassis.drive_distance(-10);
  matchloader.set(false);
  LeftDriveSmart.setVelocity(67,percent);
  RightDriveSmart.setVelocity(67,percent);
  chassis.drive_distance(-18);
  outtakeMotor.spin(forward, 100, percent);
  intakeMotor.spin(forward, 100, percent);
  wait(2,seconds);
  intakeMotor.stop();
  outtakeMotor.stop():
  chassis.drive_distance(12.815);
  chassis.turn_to_angle(0);
  chassis.drive_distance(46.26);
  chassis.turn_to_angle(-90);
  outtakeMotor.spin(forward, 100, percent);
  intakeMotor.spin(forward, 100, percent);
  chassis.drive_distance(25);
  
}

void autonomous(void) {
 skills();
}

void profiling() {
  // data entries
  chassis.drive_distance(89.534);
  wait(0.5, seconds);
  chassis.turn_to_angle(0);
  wait(0.5, seconds);
  chassis.drive_distance(13.96);
  wait(0.5, seconds);
  chassis.turn_to_angle(90);
  wait(0.5, seconds);
  std::vector<RPMEntryData> leftMotor1EntryData = {};
  std::vector<RPMEntryData> leftMotor2EntryData = {};
  std::vector<RPMEntryData> leftMotor3EntryData = {};
  std::vector<RPMEntryData> rightMotor1EntryData = {};
  std::vector<RPMEntryData> rightMotor2EntryData = {};
  std::vector<RPMEntryData> rightMotor3EntryData = {};
  std::fstream DataFile("data.txt");
  int counter = 0;
  while (true) {
    if (counter == 30) {
      counter = 0;
      // clear data entries
      leftMotor1EntryData.clear();
      leftMotor2EntryData.clear();
      leftMotor3EntryData.clear();
      rightMotor1EntryData.clear();
      rightMotor2EntryData.clear();
      rightMotor3EntryData.clear();
      for (int i = 0; i < leftMotor1EntryData.size(); i++) {
        char* str = "";
        sprintf(
          "(%d, %d),(%d, %d),(%d, %d),(%d, %d),(%d, %d),(%d, %d) ",
          str,
          leftMotor1EntryData.at(i).rpm,
          leftMotor1EntryData.at(i).timestamp,
          leftMotor2EntryData.at(i).rpm,
          leftMotor2EntryData.at(i).timestamp,
          leftMotor3EntryData.at(i).rpm,
          leftMotor3EntryData.at(i).timestamp,
          rightMotor1EntryData.at(i).rpm,
          rightMotor1EntryData.at(i).timestamp,
          rightMotor2EntryData.at(i).rpm,
          rightMotor2EntryData.at(i).timestamp,
          rightMotor3EntryData.at(i).rpm,
          rightMotor3EntryData.at(i).timestamp
        );

        DataFile << str; // write to data file
      }
    }
    RPMEntryData leftMotor1Entry;
    leftMotor1Entry.rpm = leftMotor1.velocity(rpm); 
    leftMotor1Entry.timestamp = Brain.Timer.value();
    leftMotor1EntryData.push_back(leftMotor1Entry);
    
    RPMEntryData leftMotor2Entry;
    leftMotor2Entry.rpm = leftMotor2.velocity(rpm);
    leftMotor2Entry.timestamp = Brain.Timer.value();
    leftMotor2EntryData.push_back(leftMotor2Entry);


    RPMEntryData leftMotor3Entry;
    leftMotor3Entry.rpm = leftMotor3.velocity(rpm);
    leftMotor3Entry.timestamp = Brain.Timer.value();
    leftMotor3EntryData.push_back(leftMotor3Entry);
    
    
    RPMEntryData rightMotor1Entry;
    rightMotor1Entry.rpm = rightMotor1.velocity(rpm);
    rightMotor1Entry.timestamp = Brain.Timer.value();
    rightMotor1EntryData.push_back(rightMotor1Entry);
    

    RPMEntryData rightMotor2Entry;
    rightMotor2Entry.rpm = rightMotor2.velocity(rpm);
    rightMotor2Entry.timestamp = Brain.Timer.value();
    rightMotor2EntryData.push_back(rightMotor2Entry);


    RPMEntryData rightMotor3Entry;
    rightMotor3Entry.rpm = rightMotor3.velocity(rpm);
    rightMotor3Entry.timestamp = Brain.Timer.value();
    rightMotor3EntryData.push_back(rightMotor3Entry);
    counter++;
    wait(1, seconds);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //Replace this line with chassis.control_tank(); for tank drive 
    //or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();
    if (Controller.ButtonL1.pressing()) {
      intakeMotor.spin(reverse, 100, percent);
    } else if (Controller.ButtonL2.pressing()) {
      intakeMotor.spin(forward, 100, percent);
    } else {
      intakeMotor.stop(coast);
    }

    if (Controller.ButtonR1.pressing()) {
      outtakeMotor.spin(reverse, 100, percent);
    } else if (Controller.ButtonR2.pressing()) {
      outtakeMotor.spin(forward, 100, percent);
    } else {
      outtakeMotor.stop(coast);
    } 

    if (Controller.ButtonA.pressing()) {
      matchloader_status = !matchloader_status;
      waitUntil(!Controller.ButtonA.pressing());
    } 
    matchloader.set(!matchloader_status);
    

    if (Controller.ButtonB.pressing()) {
      outputblocker_status = !outputblocker_status;
      waitUntil(!Controller.ButtonB.pressing());
    }
    outputblocker.set(!outputblocker_status);
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
  // thread proflingThread(profiling);
  // start profiling background thread (volatile, mutex may be needed in the future)
  // proflingThread.detach();
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
