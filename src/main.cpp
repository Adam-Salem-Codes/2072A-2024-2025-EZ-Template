#include "main.h"
#include "pros/motors.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {5, -3, -2},  // Left Chassis Ports (negative port will reverse it!)
    {-8, 9, 7},   // Right Chassis Ports (negative port will reverse it!)

    11,    // IMU Port
    2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);  // Wheel RPM

// pros::Motor l_lift(-14);
// pros::Motor r_lift(17);
pros::Motor intake_down(15);
pros::Motor intake_up(10);
pros::Optical color_sensor(6);
pros::Distance front_distance(15);
pros::Rotation lift(12);
ez::Piston mogo('a');
ez::Piston swiper('b');
ez::Piston doinker('f');
ez::Piston intake_lift('h');

int theta = 0;
bool red_pause = false;
bool blue_pause = false;
bool auto_clamp = false;
bool color_sort_blue = false;
bool color_sort_red = false;
int lift_prime = 1;

pros::MotorGroup lady_brown({-14, 17}, pros::MotorGears::rpm_200, pros::MotorEncoderUnits::counts);
// void set_lift(int input) {
//  l_lift.move(input);
//  r_lift.move(input);
//}
ez::PID liftPID{0.45, 0, 0, 0, "Lift"};
pros::Distance clamp_sensor(1);
vector<jas::motors::motordata> motorbar{
     {0, intake_up, "intake 1"},
 };


int auto_clamp_task() {
  while (true) {
    /*
    if (auto_clamp){
      if (clamp_sensor.get_distance() <= 35 && !mogo.get()) {
        pros::delay(50);
        mogo.set(true);
        master.rumble("..");
      }
    }*/

    pros::delay(10);
  }
  return -1;
  
}
int intake_torque_task() {
  while (true) {
    if (intake_up.get_torque() > 1.0 && intake_up.get_power() < 10.0 && pros::competition::is_autonomous()) {  // Check if intake torque exceeds 1.0 Nm and power is below 10 Watts
      intake_up.move(127);                                            // Reverse the intake
      pros::delay(500);                                            // Wait for 0.5 seconds
      intake_up.move(-127);                                           // Stop the intake
    }
    pros::delay(10);  // Delay to prevent task from running too frequently
  }
}
/**
 * @brief Moves the robot to a specified distance from the wall.
 *
 * This function calculates the distance the robot needs to move to achieve
 * the desired distance from the wall and commands the chassis to move accordingly.
 *
 * @param desired_in The desired distance from the wall in inches.
 * @param speed The speed at which the robot should move.
 */
void move_to_distance_from_wall(float desired_in, int speed) {
  chassis.pid_drive_set(-(desired_in - (front_distance.get_distance() / 25.4)), speed, false);  // Set robot to a specific distance from the wall
}

/**
 * @brief Toggles the state of the given piston.
 *
 * This function changes the state of the provided piston to its opposite state.
 * If the piston is currently extended, it will be retracted, and vice versa.
 *
 * @param p The piston to be toggled.
 */
void toggle(ez::Piston p) {
  p.set(!p.get());
}
/**
 * @brief Moves the chassis until a clamping condition is met.
 *
 * This function continuously drives the chassis at the specified speed
 * until the `mogo.get()` condition is true, indicating that the clamping
 * condition has been met. The function checks the condition every 50 milliseconds.
 *
 * @param speed The speed at which to drive the chassis. A negative value
 *              indicates reverse movement.
 */
void move_until_clamped(int speed) {
  while (!mogo.get()) {
    chassis.drive_set(-speed, -speed);
    pros::delay(50);
  }
}
/**
 * @brief Moves the chassis until the clamp is extended.
 *
 * This function drives the chassis at a specified speed for a given distance,
 * then continues to drive at a lower speed until the clamp is extended.
 *
 * @param speed The initial speed to drive the chassis.
 * @param lower_speed The lower speed to drive the chassis after the initial distance.
 * @param distance The distance to drive at the initial speed.
 */
void move_until_clamped(int speed, int lower_speed, int distance) {
  chassis.pid_drive_set(distance, speed, false);  // Drive for a certain distance at higher speed
  chassis.pid_wait();                 // Motion chain into slower speed
  while (!mogo.get()) {
    chassis.drive_set(lower_speed, lower_speed);  // Go at a lower speed until clamp extends.
    pros::delay(10);
  }
}
bool racism_against_blue = false;
int color_sensor_task() {
  int color = 100;
  
  while (true) {
    color = color_sensor.get_hue();
    if (blue_pause){
      if (color >= 200 && color <= 240){
        intake_up.move(50);
        pros::delay(100);
        intake_up.move(0);
        blue_pause = false;
      }
    }
    if (red_pause){
      if (color >= 0 && color <= 15){
        intake_up.move(50);
        pros::delay(100);
        intake_up.move(0);
        red_pause = false;
      }
    }
    if (color_sort_blue){
      if (color >= 200 && color <= 240) {
        pros::delay(100);
        intake_up.move(50);
        pros::delay(100);
        intake_up.move(-127);
      }
    }
    if (color_sort_red){
      if (color >= 0 && color <= 15) {
        pros::delay(100);
        intake_up.move(50);
        pros::delay(100);
        intake_up.move(-127);
      }
    }
    pros::delay(5);
  }
  return -1;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::Task t(auto_clamp_task);
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure
  // l_lift.tare_position();
  // l_lift.set_encoder_units(pros::MotorEncoderUnits::degrees);
  lift.reset();
  lift.reset_position();
  j_auton_selector.jautonpopulate(
      { jas::jasauton([](){



        pros::delay(100000);
        chassis.pid_turn_relative_set(90_deg, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(24, 110, true);
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-135_deg, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(34, 110, true);
        chassis.pid_wait();
        chassis.pid_turn_relative_set(45_deg, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-24, 110, true);
        chassis.pid_wait();
        pros::delay(100000);
        chassis.pid_drive_set(24, 110, true);
        chassis.pid_wait();
        chassis.pid_drive_set(-18, 110, true);
        chassis.pid_wait();
        chassis.pid_drive_set(-6, 110, true);
        chassis.pid_wait();
        chassis.pid_turn_relative_set(90_deg, 120, false);
        chassis.pid_wait();
        chassis.pid_turn_relative_set(45_deg, 120, false);
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-135_deg, 120, false);
        chassis.pid_wait();
        

        pros::delay(100000);

        intake_up.move(-127);
        intake_down.move(127);

        chassis.pid_drive_set(-36, 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait();
        chassis.pid_drive_set(-8.5_in, 35, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        mogo.set(true);
        chassis.pid_turn_relative_set(45, 90, false);
        intake_up.move(-127);
        intake_down.move(127);
        lady_brown.move_absolute((1875 / 3), 150);
        chassis.pid_wait();
        chassis.pid_drive_set(12, 80, false);
        chassis.pid_wait();

        pros::delay(1000);

        pros::delay(5000);
        chassis.pid_drive_set(-12, 127, true);
        chassis.pid_wait();
        pros::delay(10000);
       }, 3, 2, "PID Tuning", "Elims Wall Stake", 1, 1, false),
        
        jas::jasauton([]() {
        theta = 168;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        blue_pause = true;                   
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-90);
        intake_down.move(127);  
        chassis.pid_drive_set(44_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-4_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(360-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-14_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        blue_pause = false;
        pros::delay(200);
        intake_up.move(-127);
        color_sort_red = true;
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(17_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(350-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(20_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        lady_brown.move_absolute(1000, 200);
        pros::delay(500);
        chassis.pid_drive_set(-35_in, 120, true);  // Move away from wall after alliance stake score
        pros::delay(500);
        intake_up.move(0);
        chassis.pid_wait_quick();
      }, 1, 0, "Blue Negative AWP", "Blue 3 rings + Alliance", 3, 0, false),

jas::jasauton([]() {
        
        theta = 192;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        red_pause = true;                   
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-90);
        intake_down.move(127);  
        chassis.pid_drive_set(44_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-4_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(0-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-14_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        red_pause = false;
        pros::delay(200);
        intake_up.move(-127);
        color_sort_blue = true;
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(17_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(10-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(20_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        lady_brown.move_absolute(1000, 200);
        pros::delay(500);
        chassis.pid_drive_set(-35_in, 120, true);  // Move away from wall after alliance stake score
        pros::delay(500);
        intake_up.move(0);
        chassis.pid_wait_quick();
      }, 0, 0, "Red Negative AWP", "Red 3 rings + Alliance", 3, 0, true),

       
        jas::jasauton([]() {
        theta = 168;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        blue_pause = true;                   
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-90);
        intake_down.move(127);  
        chassis.pid_drive_set(42_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-4_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(360-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-14_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        blue_pause = false;
        pros::delay(200);
        intake_up.move(-127);
        color_sort_red = true;
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(350-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(30_in, 60, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-20_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(470-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(40_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(540-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(50_in, 120, true);
        chassis.pid_wait_quick();
        pros::delay(2000);
      }, 1, 0, "Blue Negative Elims", "Blue 4 rings + Alliance", 3, 0, false),

jas::jasauton([]() {
        
        theta = 192;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        red_pause = true;                   
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-90);
        intake_down.move(127);  
        chassis.pid_drive_set(44_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-4_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(0-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-14_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        red_pause = false;
        pros::delay(200);
        intake_up.move(-127);
        color_sort_blue = true;
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(17_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(10-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(30_in, 60, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-20_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(-110-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(40_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(-180-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(50_in, 120, true);
        chassis.pid_wait_quick();
        pros::delay(2000);
      }, 0, 0, "Red Negative Elims", "Red 4 rings + Alliance", 4, 0, true),
        
                jas::jasauton([]() {
        theta = 168;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        pros::delay(1500);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        blue_pause = true;                   
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-90);
        intake_down.move(127);  
        chassis.pid_drive_set(44_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-4_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(360-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-14_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        blue_pause = false;
        pros::delay(200);
        intake_up.move(-127);
        color_sort_red = true;
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(17_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(350-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(20_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        lady_brown.move_absolute(1000, 200);
        pros::delay(500);
        chassis.pid_drive_set(-35_in, 120, true);  // Move away from wall after alliance stake score
        pros::delay(500);
        intake_up.move(0);
        chassis.pid_wait_quick();
      }, 1, 0, "Blue Negative Wait", "Blue Negative Wait AWP ", 3, 0, false),

jas::jasauton([]() {
        
        theta = 192;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        pros::delay(1500);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        red_pause = true;                   
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-90);
        intake_down.move(127);  
        chassis.pid_drive_set(44_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-4_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(0-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-14_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        red_pause = false;
        pros::delay(200);
        intake_up.move(-127);
        color_sort_blue = true;
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(17_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(10-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(20_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        lady_brown.move_absolute(1000, 200);
        pros::delay(500);
        chassis.pid_drive_set(-35_in, 120, true);  // Move away from wall after alliance stake score
        pros::delay(500);
        intake_up.move(0);
        chassis.pid_wait_quick();
      }, 0, 0, "Red Negative Wait", "Red Negative Wait AWP", 3, 0, true),
      
        jas::jasauton([]() {
        
        theta = 192;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 127, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        red_pause = true;                   
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-90);
        intake_down.move(127);  
        chassis.pid_drive_set(44_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-4_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(0-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-14_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        red_pause = false;
        pros::delay(200);
        intake_up.move(-127);
        pros::delay(400);
        mogo.set(false);
        

        chassis.pid_turn_set(-90-theta, 120, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        chassis.pid_drive_set(24_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        red_pause = true;
        chassis.pid_turn_set(-180-theta, 120, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        chassis.pid_drive_set(54_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(-90-theta, 120, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-10_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        red_pause = false;
        pros::delay(100);
        chassis.pid_turn_set(-190-theta, 120, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        intake_up.move(-127);
        chassis.pid_drive_set(26_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        chassis.pid_turn_set(-160-theta, 120, false);  // Turn to ring stack
        chassis.pid_drive_set(-43_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_until(-20);

        intake_up.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        intake_up.move(0);
        chassis.pid_wait_quick();
        
        
      }, 0, 2, "Sig AWP", "2 Ring + 1 Ring + Alliance", 2, 1, true),

              jas::jasauton([]() {
        
        theta = 192;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 127, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        blue_pause = true;                   
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-90);
        intake_down.move(127);  
        chassis.pid_drive_set(44_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-4_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(0-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-14_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        blue_pause = false;
        pros::delay(200);
        intake_up.move(-127);
        pros::delay(400);
        mogo.set(false);
        

        chassis.pid_turn_set(-90-theta, 120, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        chassis.pid_drive_set(24_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        blue_pause = true;
        chassis.pid_turn_set(-180-theta, 120, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        chassis.pid_drive_set(54_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(-90-theta, 120, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-10_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 70, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        blue_pause = false;
        pros::delay(100);
        chassis.pid_turn_set(-190-theta, 120, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        intake_up.move(-127);
        chassis.pid_drive_set(26_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        chassis.pid_turn_set(-160-theta, 120, false);  // Turn to ring stack
        chassis.pid_drive_set(-43_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_until(-20);

        intake_up.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        intake_up.move(0);
        chassis.pid_wait_quick();
        
        
      }, 1, 2, "Sig AWP", "2 Ring + 1 Ring + Alliance", 2, 1, true),
       
      
      
              jas::jasauton([]() {
        
        theta = 60;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        chassis.pid_drive_set(-20_in, 127, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-10_in, 70, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        mogo.set(true);
        pros::delay(200);
        intake_up.move(-127);
        intake_down.move(127);  
        pros::delay(400);               
        chassis.pid_turn_set(0-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        red_pause = true;
        chassis.pid_drive_set(24_in, 127, true);
        chassis.pid_wait_quick();  // Move away from wall after alliance stake score
        chassis.pid_turn_set(70-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        doinker.set(true);
        chassis.pid_drive_set(36_in, 127, true);
        chassis.pid_wait_quick();
        intake_down.move(0);
        lady_brown.move_absolute(325, 200);

        chassis.pid_turn_set(180-theta, 60, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-127);
        
        doinker.set(false);
        chassis.pid_turn_set(280-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(40_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        intake_up.move(0);
        lady_brown.move_absolute(1700, 200);
        lift_prime = 3;
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_turn_set(310-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        
        


        
      }, 0, 0, "Cookage", "Cookage", 2, 0, true),
      
              jas::jasauton([]() {
        
        theta = 192;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);
        pros::delay(1000);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 127, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        blue_pause = true;                   
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        intake_up.move(-100);
        intake_down.move(127);  
        chassis.pid_drive_set(44_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-4_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(0-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-18_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 40, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        blue_pause = false;
        pros::delay(200);

        color_sort_red = true;
        chassis.pid_turn_set(-45-theta, 120, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        intake_up.move(-127);

        chassis.pid_drive_set(48_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        intake_down.move(127);
        pros::delay(500);
        
        intake_lift.set(true);
        pros::delay(200);
        chassis.drive_set(80, 80);
        pros::delay(400);
        intake_lift.set(false);
        pros::delay(400);
        chassis.drive_set(0, 0);
        pros::delay(800);
        chassis.pid_drive_set(-16_in, 40, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_drive_set(8_in, 40, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        chassis.pid_turn_set(-40-theta, 120, false);
        chassis.pid_drive_set(-70_in, 100, true);
        chassis.pid_wait_until(-20);

        intake_up.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        intake_up.move(0);
        chassis.pid_wait_quick();
        
        
      }, 1, 2, "Normal AWP", "Normal AWP", 2, 0, true),


              jas::jasauton([](){
        chassis.pid_drive_set(-(3 * 12), 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait();
        chassis.pid_drive_set(-8.5_in, 35, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        mogo.set(true);
        chassis.pid_turn_relative_set(-45, 90, false);
        intake_up.move(-127);
        lady_brown.move_absolute((1875 / 3), 150);
        chassis.pid_wait();
        chassis.pid_drive_set(12, 80, false);
        chassis.pid_wait();

        pros::delay(1000);
        intake_up.move(0);
        toggle(mogo);
        chassis.pid_turn_set(-110, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_set(25, 90, false);
        chassis.pid_wait();
        intake_lift.set(!intake_lift.get());
        intake_up.move(-127);
        chassis.pid_drive_set(30_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        chassis.pid_drive_set(-8_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        intake_lift.set(!intake_lift.get());
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        intake_up.move(0);
        chassis.pid_turn_relative_set(-35, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(10_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        lady_brown.move_absolute(1875, 200);
        pros::delay(500);
        chassis.pid_drive_set(-12, 127, true);
        chassis.pid_wait();
        pros::delay(10000);
       }, 0, 1, "Goal rush AWP", "Goal rush AWP", 1, 1, true),

       jas::jasauton([](){
        chassis.pid_drive_set(-(3 * 12), 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait();
        chassis.pid_drive_set(-8.5_in, 35, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        mogo.set(true);
        chassis.pid_turn_relative_set(45, 90, false);
        intake_up.move(-127);
        lady_brown.move_absolute((1875 / 3), 150);
        chassis.pid_wait();
        chassis.pid_drive_set(12, 80, false);
        chassis.pid_wait();

        pros::delay(1000);
        intake_up.move(0);
        toggle(mogo);
        chassis.pid_turn_set(110, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_set(-25, 90, false);
        chassis.pid_wait();
        intake_lift.set(!intake_lift.get());
        intake_up.move(-127);
        chassis.pid_drive_set(30_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        chassis.pid_drive_set(-8_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        intake_lift.set(!intake_lift.get());
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        intake_up.move(0);
        chassis.pid_turn_relative_set(35, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(10_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        lady_brown.move_absolute(1875, 200);
        pros::delay(500);
        chassis.pid_drive_set(-12, 127, true);
        chassis.pid_wait();
        pros::delay(10000);
       }, 1, 1, "Goal rush AWP", "Goal rush AWP", 1, 1, true),



       jas::jasauton([](){
        theta = 293;
        
        doinker.set(true);
        intake_down.move(127);
        intake_up.move(-20);
        
        chassis.pid_drive_set(32, 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        doinker.set(false);
        chassis.pid_drive_set(-18, 90, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick();
        doinker.set(true);
        pros::delay(300);
        chassis.pid_turn_set(87-theta, 120, false);
        chassis.pid_wait_quick();
        doinker.set(false);
        
        chassis.pid_drive_set(-12_in, 70, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        pros::delay(100);
        intake_up.move(-110);
        pros::delay(300);
        blue_pause = true;
        pros::delay(100);
        color_sort_red = true;
        mogo.set(false);
        
        chassis.pid_turn_set(180-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-15_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 40, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        blue_pause = false;
        mogo.set(true);

        chassis.pid_turn_set(115-theta, 120, false);
        chassis.pid_wait_quick();
        intake_up.move(-127);
        chassis.pid_drive_set(35_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        blue_pause = true;
        chassis.pid_turn_set(160-theta, 120, false);
        chassis.pid_wait_quick();
        mogo.set(false);
        doinker.set(true);
        pros::delay(100);
        chassis.pid_drive_set(15_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        
        intake_lift.set(false);
        chassis.pid_turn_set(260-theta, 120, false);
        chassis.pid_wait_quick();
        doinker.set(false);
        
        chassis.pid_drive_set(46_in, 100, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        /*
        chassis.pid_turn_set(135-theta, 120, false);
        chassis.pid_wait_quick();
        mogo.set(false);
        red_pause = true;
        chassis.pid_drive_set(48_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        intake_down.move(127);
        pros::delay(500);
        
        intake_lift.set(true);
        pros::delay(200);
        chassis.drive_set(80, 80);
        lady_brown.move_absolute(300, 200);
        
        pros::delay(400);
        intake_lift.set(false);
        pros::delay(400);
        
        intake_lift.set(false);
        chassis.pid_turn_set(265-theta, 120, false);
        chassis.pid_wait_quick();
        
        chassis.pid_drive_set(44_in, 100, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        */

        if (!blue_pause){
          lady_brown.move_absolute(325, 200);
          pros::delay(200);
          intake_up.move(-127);
          chassis.pid_turn_set(240-theta, 60, false);
          chassis.pid_wait_quick();
          pros::delay(200);
          intake_up.move(0);
          lady_brown.move_absolute(1700, 200);
          lift_prime = 3;
          chassis.pid_drive_set(8_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
          chassis.pid_wait_quick();
        }
        else {
          chassis.pid_turn_set(135-theta, 120, false);
          chassis.pid_wait_quick();
          lady_brown.move_absolute(0, 200);
        }

        
       }, 1, 1, "FAST Goal Rush", "FAST Goal Rush", 1, 1, true),

       


       jas::jasauton([](){
        theta = 293;
        
        doinker.set(true);
        intake_down.move(127);
        intake_up.move(-20);
        blue_pause = true;
        
        chassis.pid_drive_set(32, 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        doinker.set(false);
        chassis.pid_drive_set(-12, 90, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick();
        doinker.set(true);
        pros::delay(300);
        chassis.pid_turn_set(97-theta, 120, false);
        chassis.pid_wait_quick();
        doinker.set(false);
        
        chassis.pid_drive_set(-12_in, 70, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        pros::delay(100);
        chassis.pid_turn_set(50-theta, 120, false);
        chassis.pid_wait_quick();
        intake_up.move(-127);

        chassis.pid_drive_set(60_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        pros::delay(200);
        //lady_brown.move_absolute(500, 200);
        chassis.pid_wait_quick();
        //lady_brown.move_absolute(1000, 200);
        pros::delay(300);

        chassis.pid_turn_set(0-theta, 120, false);
        chassis.pid_wait_quick();
        //lady_brown.move_absolute(0, 200);
        mogo.set(false);
        chassis.pid_turn_set(70-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(-24, 120, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();

        chassis.pid_drive_set(-8, 70, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick();
        mogo.set(true);
        pros::delay(100);


        
        chassis.pid_turn_set(120-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(36_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(180-theta, 120, false);
        chassis.pid_wait_quick();
        doinker.set(true);
        chassis.pid_drive_set(12_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(270-theta, 120, false);
        chassis.pid_wait_quick();
        doinker.set(false);
        chassis.pid_turn_set(250-theta, 120, false);
        chassis.pid_wait_quick();
        color_sort_red = true;

        chassis.pid_drive_set(24_in, 70, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        chassis.pid_turn_set(330-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(40_in, 70, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        
       }, 1, 1, "FAST Rush AWP", "FAST Rush AWP", 1, 1, true),



       jas::jasauton([](){
        theta = 113;
        
        doinker.set(true);
        intake_down.move(127);
        intake_up.move(-20);
        
        chassis.pid_drive_set(32, 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        doinker.set(false);
        chassis.pid_drive_set(-12, 90, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick();
        doinker.set(true);
        pros::delay(300);
        chassis.pid_turn_set(285-theta, 120, false);
        chassis.pid_wait_quick();
        doinker.set(false);
        
        chassis.pid_drive_set(-12_in, 40, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        pros::delay(100);
        intake_up.move(-127);
        pros::delay(400);
        red_pause = true;
        mogo.set(false);
        
        chassis.pid_drive_set(6_in, 70, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(180-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-15_in, 120, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8_in, 40, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        red_pause = false;
        mogo.set(true);
        intake_up.move(-127);
        chassis.pid_turn_set(225-theta, 120, false);
        chassis.pid_wait_quick();
        mogo.set(false);
        color_sort_blue = true;
        red_pause = true;
        chassis.pid_drive_set(48_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        intake_down.move(127);
        pros::delay(500);
        
        intake_lift.set(true);
        pros::delay(200);
        chassis.drive_set(80, 80);
        
        pros::delay(400);
        intake_lift.set(false);
        pros::delay(400);
        
        intake_lift.set(false);
        chassis.pid_turn_set(95-theta, 120, false);
        chassis.pid_wait_quick();
        
        chassis.pid_drive_set(44_in, 100, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        lady_brown.move_absolute(300, 200);

        if (!red_pause){
          intake_up.move(-127);
          chassis.pid_turn_set(60-theta, 120, false);
          chassis.pid_wait_quick();
          intake_up.move(0);
          lady_brown.move_absolute(1700, 200);
          lift_prime = 3;
          chassis.pid_drive_set(8_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
          chassis.pid_wait_quick();
        }
        else {
          chassis.pid_turn_set(180-theta, 120, false);
          chassis.pid_wait_quick();
          lady_brown.move_absolute(0, 200);
        }

        
       }, 0, 1, "FAST Goal Rush", "FAST Goal Rush", 1, 1, true),

       jas::jasauton([](){
        theta = 113;
        red_pause = true;
        
        doinker.set(true);
        intake_down.move(127);
        intake_up.move(-127);
        lady_brown.move_absolute(650, 200);
        
        chassis.pid_drive_set(32, 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        doinker.set(false);
        chassis.pid_drive_set(-12, 90, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick();
        doinker.set(true);
        pros::delay(300);
        chassis.pid_turn_set(285-theta, 120, false);
        chassis.pid_wait_quick();
        doinker.set(false);
        intake_up.move(0);
        
        chassis.pid_drive_set(-12_in, 40, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        pros::delay(100);
        chassis.pid_turn_set(307-theta, 120, false);
        chassis.pid_wait();
        intake_up.move(-127);
        pros::delay(500);
        intake_up.move(127);
        pros::delay(100);
        intake_up.move(-127);

        chassis.pid_drive_set(54_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        
        lady_brown.move_absolute(1700, 200);
        pros::delay(600);
        lady_brown.move_absolute(0, 200);

        chassis.pid_turn_set(360-theta, 120, false);
        chassis.pid_wait_quick();
        lady_brown.move_absolute(0, 200);
        mogo.set(false);
        intake_up.move(0);
        chassis.pid_turn_set(290-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(-28, 120, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();

        chassis.pid_drive_set(-12, 40, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick();
        mogo.set(true);
        pros::delay(100);


        
        chassis.pid_turn_set(200-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(34_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        intake_up.move(-127);
        chassis.pid_turn_set(250-theta, 120, false);
        chassis.pid_wait_quick();
        doinker.set(true);
        chassis.pid_drive_set(28_in, 60, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_wait_quick();
        chassis.pid_turn_set(330-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_turn_set(405-theta, 120, false);
        chassis.pid_wait_quick();
        doinker.set(false);
        color_sort_blue = true;

        chassis.pid_drive_set(48_in, 70, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();


        
       }, 0, 1, "FAST Rush AWP", "FAST Rush AWP", 1, 1, true),



       jas::jasauton([](){
        chassis.pid_drive_set(-38, 127, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-9, 30, true);
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_relative_set(45, 120, false);
        chassis.pid_wait();
        intake_up.move(-127);
        pros::delay(300);
        intake_down.move(127);
        chassis.pid_drive_set(12, 120, false);
        intake_up.move(-30);
        chassis.pid_wait();
        
        mogo.set(false);
        chassis.pid_turn_set(110, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        pros::delay(150);
        intake_up.move(-127);
        pros::delay(300);
        chassis.pid_turn_set(-20, 120, false);
        chassis.pid_wait();
        lady_brown.move_absolute(300, 200);
        intake_lift.set(!intake_lift.get());
        chassis.pid_drive_set(26_in, 120, true);
        chassis.pid_wait_quick();

        
        chassis.pid_drive_set(7.5_in, 35, true); // slow down for stack
        chassis.pid_wait();
        intake_lift.set(!intake_lift.get());
        intake_up.move(0);
        pros::delay(250);
        chassis.pid_drive_set(-4_in, 70, true);
        chassis.pid_wait();
        pros::delay(250);
        intake_up.move(-80);
        chassis.pid_turn_relative_set(163, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(60_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(10, 90, false);
        chassis.pid_wait();
        intake_up.move(0);
        intake_down.move(0);
        lady_brown.move_absolute(1700, 200);
        chassis.pid_turn_relative_set(15, 90, false);
        chassis.pid_drive_set(5_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-15, 120, false);
        chassis.pid_wait();
        chassis.pid_turn_relative_set(15, 120, false);
        chassis.pid_wait();
       }, 1, 1, "Goal Rush Elims", "Goal Rush Elims", 1, 1, false),

      jas::jasauton([](){
        chassis.pid_drive_set(-38, 127, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-9, 30, true);
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_relative_set(-45, 120, false);
        chassis.pid_wait();
        intake_up.move(-127);
        pros::delay(300);
        intake_down.move(127);
        chassis.pid_drive_set(12, 120, false);
        intake_up.move(-30);
        chassis.pid_wait();
        
        mogo.set(false);
        chassis.pid_turn_set(-110, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        pros::delay(150);
        intake_up.move(-127);
        pros::delay(300);
        chassis.pid_turn_set(20, 120, false);
        chassis.pid_wait();
        lady_brown.move_absolute(300, 200);
        intake_lift.set(!intake_lift.get());
        chassis.pid_drive_set(26_in, 120, true);
        chassis.pid_wait_quick();

        
        chassis.pid_drive_set(7.5_in, 35, true); // slow down for stack
        chassis.pid_wait();
        intake_lift.set(!intake_lift.get());
        intake_up.move(0);
        pros::delay(250);
        chassis.pid_drive_set(-4_in, 70, true);
        chassis.pid_wait();
        pros::delay(250);
        intake_up.move(-80);
        chassis.pid_turn_relative_set(-163, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(60_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-10, 90, false);
        chassis.pid_wait();
        intake_up.move(0);
        intake_down.move(0);
        lady_brown.move_absolute(1700, 200);
        chassis.pid_turn_relative_set(-15, 90, false);
        chassis.pid_drive_set(5_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(15, 120, false);
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-15, 120, false);
        chassis.pid_wait();
       }, 0, 1, "Goal Rush Elims", "Goal Rush Elims", 1, 1, false),

       
      jas::jasauton([](){
        chassis.pid_drive_set(-36, 127, true);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-10, 60, true);
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_relative_set(-45, 120, false);
        chassis.pid_wait();
        intake_up.move(-127);
        pros::delay(300);
        intake_down.move(127);
        chassis.pid_drive_set(12, 120, false);
        intake_up.move(-30);
        chassis.pid_wait();
        
        mogo.set(false);
        chassis.pid_turn_set(-110, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        intake_up.move(0);
        mogo.set(true);
        pros::delay(150);
        intake_up.move(-127);
        pros::delay(500);
        chassis.pid_turn_relative_set(-150, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(13_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        
       }, 0, 1, "Goal Rush Quals", "Goal Rush Quals", 1, 1, false),

       
      jas::jasauton([](){
        swiper.set(true);
        intake_down.move(127);
        chassis.pid_drive_set(44, 120, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-30, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-24, 120, false);
        chassis.pid_wait();
        mogo.set(true);
        pros::delay(150);
        swiper.set(false);
        
        chassis.pid_turn_relative_set(-40, 120, false);
        chassis.pid_wait();
        
        intake_up.move(-127);
        chassis.pid_drive_set(24, 120, false);
        chassis.pid_wait();
        
        chassis.pid_turn_relative_set(-135, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(30, 120, false);
        chassis.pid_wait();

        chassis.pid_turn_relative_set(-30, 120, false);
        chassis.pid_wait();

        //intake_lift.set(!intake_lift.get());

        chassis.pid_drive_set(10, 60, false);
        chassis.pid_wait();

        intake_down.move(-127);
        pros::delay(15000);

        intake_lift.set(!intake_lift.get());
        
        chassis.pid_turn_relative_set(-180, 120, false);
        chassis.pid_wait();
        intake_down.move(0);

        chassis.pid_drive_set(-60, 120, false);
        chassis.pid_wait();

       }, 0, 0, "Ring Rush Elims", "5 Ring", 5, 0, false),

       
       
      jas::jasauton([](){
        swiper.set(true);
        intake_down.move(127);
        chassis.pid_drive_set(44, 120, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait();
        chassis.pid_turn_relative_set(30, 120, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-24, 120, false);
        chassis.pid_wait();
        mogo.set(true);
        pros::delay(150);
        swiper.set(false);
        
        chassis.pid_turn_relative_set(40, 120, false);
        chassis.pid_wait();
        
        intake_up.move(-127);
        chassis.pid_drive_set(24, 120, false);
        chassis.pid_wait();
        
        chassis.pid_turn_relative_set(135, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(30, 120, false);
        chassis.pid_wait();

        chassis.pid_turn_relative_set(30, 120, false);
        chassis.pid_wait();

        //intake_lift.set(!intake_lift.get());

        chassis.pid_drive_set(10, 60, false);
        chassis.pid_wait();

        intake_down.move(-127);
        pros::delay(15000);

        intake_lift.set(!intake_lift.get());
        
        chassis.pid_turn_relative_set(180, 120, false);
        chassis.pid_wait();
        intake_down.move(0);

        chassis.pid_drive_set(-60, 120, false);
        chassis.pid_wait();

       }, 1, 0, "Ring Rush Elims", "Ring Rush Elims", 5, 0, false),
     


      jas::jasauton([]() {
        intake_up.move(-127);
        pros::delay(500);
        intake_up.move(0);
        intake_down.move(127);
        chassis.pid_drive_set(12_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-90, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_relative_set(90, 90, false);
        chassis.pid_wait();
        intake_up.move(-127);
        chassis.pid_drive_set(26_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(90, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(25_in, 60, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(90, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(24_in, 60, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        pros::delay(1000);
        chassis.pid_drive_set(12_in, 30, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        pros::delay(1000);
        chassis.pid_turn_relative_set(-135, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(24_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-45, 90, false);
        chassis.pid_wait();
        pros::delay(1000);
        intake_up.move(0);
        mogo.set(false);
        chassis.pid_drive_set(-20_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_drive_set(6_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(90, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-84_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_relative_set(-90, 90, false);
        chassis.pid_wait();
        intake_up.move(-127);
        chassis.pid_drive_set(26_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-90, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(25_in, 60, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-90, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(24_in, 60, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        pros::delay(1000);
        chassis.pid_drive_set(12_in, 30, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        pros::delay(1000);
        chassis.pid_turn_relative_set(135, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(24_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(45, 90, false);
        chassis.pid_wait();
        pros::delay(1000);
        intake_up.move(0);
        mogo.set(false);
        chassis.pid_drive_set(-20_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_drive_set(6_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
      }, 2, 2, "Skills", "Skills", 6, 6, true),

      jas::jasauton([](){
        chassis.pid_drive_set(30_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        }, 2, 2, "MOVE", "MOVE", 5, 0, false),

      jas::jasauton([](){
        chassis.pid_drive_set(3_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

       }, 2, 2, "MOVE Small", "MOVE Small", 5, 0, false)});
  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);
  chassis.initialize();
 // ez::as::initialize();
   //pros::Task tempcheckcontroller(tempcheckctrl);
   screeninit();
   tempcheck();
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  intake_up.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //pros::Task t(auto_clamp_task);
  pros::Task t1(color_sensor_task);
  // pros::Task t2(intake_torque_task);
  // ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
  jautonrun();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  chassis.drive_brake_set(driver_preference_brake);
  intake_up.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  swiper.set(false);
  lady_brown.move_absolute(0, 200);
  red_pause = false;
  blue_pause = false;
  auto_clamp = false;
  color_sort_blue = false;
  color_sort_red = false;
  int reset = false;
  
  // l_lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  // r_lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (true) {
    master.print(0, 0, "%f", chassis.drive_imu_get());
    color_sensor.set_led_pwm(100);
            if(lv_obj_get_parent(pageswitch) == motortemps) {
            for(int m = 0; m < motorbar.size(); m++) {
                lv_event_send(motorboxes[m], LV_EVENT_REFRESH, NULL);
            }
        }
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      // set_lift(liftPID.compute((lift.get_angle() / 100.0)));

      // Trigger the selected autonomous routine
      if (master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_A)) {
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }
    if (master.get_digital_new_press(DIGITAL_X))
        chassis.pid_tuner_toggle();
      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        if (lift_prime == 1) {
          lift_prime = 2;
          lady_brown.tare_position_all();
          lady_brown.set_brake_mode_all(MOTOR_BRAKE_HOLD);
          lady_brown.move_absolute(300, 200);
        } else if (lift_prime == 2) {
          lift_prime = 3;
          lady_brown.move_absolute(1700, 200);
        } else {
          lift_prime = 1;
          lady_brown.set_brake_mode_all(MOTOR_BRAKE_COAST);
          lady_brown.move_absolute(0, 200);
        }
      }
      if (master.get_digital_new_press(DIGITAL_R1))
        mogo.set(!mogo.get());
      if (master.get_digital_new_press(DIGITAL_UP))
        swiper.set(!swiper.get());
      if (master.get_digital_new_press(DIGITAL_B))
        doinker.set(!doinker.get());
      if (master.get_digital_new_press(DIGITAL_Y))
        intake_lift.set(!intake_lift.get());
      if (master.get_digital_new_press(DIGITAL_A))
      {
        if (!reset) {
        reset = true;
        lady_brown.move(-127);
        } else {
        lady_brown.move(0);
        lady_brown.tare_position_all();
        lift_prime = 1;
        reset = false;
        }
      }
      if (master.get_digital(DIGITAL_LEFT)) {
        intake_down.move(-127);
      }
      if (master.get_digital(DIGITAL_DOWN)) {
        intake_up.move(127);
        intake_down.move(-127);
      } else if (master.get_digital(DIGITAL_R2)) {
        intake_down.move(127);
        intake_up.move(-127);
      } else {
        intake_up.move(0);
        intake_down.move(0);
      }
    // if (lv_obj_get_parent(pageswitch) == motortemps) {
    //   for (int m = 0; m < motorbar.size(); m++) {
    //     lv_event_send(motorboxes[m], LV_EVENT_REFRESH, NULL);
    //   }
    // }
    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}