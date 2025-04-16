#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-7, -8, 9},
    {5, 18, -17},  // Left Chassis Ports (negative port will reverse it!)
       // Right Chassis Ports (negative port will reverse it!)

    20,    // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);  // Wheel RPM

// pros::Motor l_lady_brown(-14);
// pros::Motor r_lady_brown(17);
pros::Motor lower_intake(-11);
pros::Motor upper_intake(-10);
pros::Optical color_sensor(2);
pros::Distance ring_check(3);
pros::Motor lady_brown(1);

pros::adi::DigitalIn lady_brown_reset('g');

ez::Piston mogo('a');
ez::Piston swiper('b');

ez::Piston left_doinker('g');
ez::Piston right_doinker('g');
ez::Piston PTO('e');

ez::Piston winch_lock('f');
ez::Piston hang_reset('g');

int theta;

bool red_pause = false;
bool blue_pause = false;

bool color_sort_blue = false;
bool color_sort_red = false;

bool sort_blue_driver = false;
bool sort_red_driver = false;

bool auton1 = false;

bool auto_clamp = false;

int lady_brown_pos = 1;
bool intake_lady_brown = false;

bool red_ring = false;
bool blue_ring = false;
bool ring_top = false;

double color = 0;
// void set_lady_brown(int input) {
//  l_lady_brown.move(input);
//  r_lady_brown.move(input);
//}


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

/**
 * @brief Moves the robot to a specified distance from the wall.
 *
 * This function calculates the distance the robot needs to move to achieve
 * the desired distance from the wall and commands the chassis to move accordingly.
 *
 * @param desired_in The desired distance from the wall in inches.
 * @param speed The speed at which the robot should move.
 */


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

int color_sensor_task() { 
  while (true) {
    if (blue_pause){
      if (blue_ring && ring_top){
        upper_intake.move(0);
        blue_pause = false;
      }
    }
    if (red_pause){
      if (red_ring && ring_top){
        upper_intake.move(0);
        red_pause = false;
      }
    }
    if (color_sort_blue){
      if (blue_ring && ring_top) {
        upper_intake.move(-127);
        pros::delay(90);
        upper_intake.move(50);
        pros::delay(100);
        upper_intake.move(-100);
        blue_ring = false;
      }
    }
    if (color_sort_red){
      if (red_ring && ring_top) {
        pros::delay(50);
        upper_intake.move(50);
        pros::delay(50);
        upper_intake.move(-100);
        red_ring = false;
      }
    }
    if (!(pros::competition::is_autonomous() || auton1)){
      if (!(lady_brown_pos == 2)){
        if (master.get_digital(DIGITAL_DOWN)) {
          upper_intake.move(110);
        } else if (master.get_digital(DIGITAL_R2)) {
          upper_intake.move(-110);
        } else {
          upper_intake.move(0);
        }
      } else {
        if (master.get_digital(DIGITAL_DOWN)) {
          upper_intake.move(70);
        } else if (master.get_digital(DIGITAL_R2)) {
          upper_intake.move(-110);
        } else {
          upper_intake.move(0);
        }
      }
    }
    pros::delay(5);
  }
  return -1;
}

int color_sense() {
  while (true) {
    color = color_sensor.get_hue();
    if (color >= 0 && color <= 15) red_ring = true;
    if (color >= 200 && color <= 240) blue_ring = true;
    pros::delay(5);
  }
  return -1;
}

int ring_sense() {
  while (true) {
    if (ring_check.get_distance() < 50) ring_top = true;
    else ring_top = false;
    pros::delay(5);
  }
  return -1;
}

int lady_brown_control(){
  while (true){
    if ((!lady_brown_reset.get_value()) && (lady_brown_pos != 1)){
      lady_brown.move(0);
      lady_brown.set_brake_mode_all(MOTOR_BRAKE_COAST);
      lady_brown_pos = 1;
      lady_brown.tare_position_all();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      if (lady_brown_pos == 1) {
        lady_brown_pos = 2;
        lady_brown.set_brake_mode_all(MOTOR_BRAKE_HOLD);
        lady_brown.move_absolute(-275, 100);
      } else if (lady_brown_pos == 2) {
        lady_brown_pos = 3;
        lady_brown.move_absolute(-700, 70);
      } else if (lady_brown_pos == 3) {
        lady_brown_pos = 4;
        lady_brown.move_absolute(-1100, 130);
      } else {
        lady_brown.move(120);
      }
    }

    if (master.get_digital_new_press(DIGITAL_A)){
      lady_brown_pos = 4;
      lady_brown.move(120);
    }
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

  pros::delay(1000);  // Stop the user from doing anything while legacy ports configure

  pros::Task t(lady_brown_control);
  pros::Task t1(color_sensor_task);
  pros::Task t2(color_sense);
  pros::Task t3(ring_sense);
  

  color_sensor.set_led_pwm(100);
  j_auton_selector.jautonpopulate(
      { 

       
        jas::jasauton([](){
          auton1 = true;
  
  

          chassis.pid_turn_relative_set(90_deg, 120, false);
          chassis.pid_wait();
          pros::delay(2000);
          chassis.pid_turn_relative_set(-90_deg, 120, false);
          chassis.pid_wait();
          pros::delay(2000);
          chassis.pid_drive_set(24, 110, true);
          chassis.pid_wait();
          pros::delay(2000);
          chassis.pid_drive_set(-24, 110, true);
          chassis.pid_wait();
          pros::delay(2000);
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
  
          upper_intake.move(-127);
          lower_intake.move(127);
  
          chassis.pid_drive_set(-36, 127, false);  // Move the majority of the distance to the mogo
          chassis.pid_wait();
          chassis.pid_drive_set(-8.5_in, 35, true);  // Slow down before reaching the mobile goal to clamp correctly.
          chassis.pid_wait();
  
          mogo.set(true);
          chassis.pid_turn_relative_set(45, 90, false);
          upper_intake.move(-127);
          lower_intake.move(127);
          lady_brown.move_absolute((1875 / 3), 150);
          chassis.pid_wait();
          chassis.pid_drive_set(12, 80, false);
          chassis.pid_wait();
  
          pros::delay(1000);
  
          pros::delay(5000);
          chassis.pid_drive_set(-12, 127, true);
          chassis.pid_wait();
          pros::delay(10000);
         }, 2, 2, "PID Tuning", "PID Tuning", 1, 1, false),
       
      jas::jasauton([](){
        auton1 = true;
        theta = -30;
        color_sort_red = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1100, 50);
        pros::delay(500);

        chassis.pid_drive_set(-14, 110, false);
        chassis.pid_wait();

        lady_brown.move_absolute(-450, 200);

        chassis.pid_turn_set(-90-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        mogo.set(true);

        chassis.pid_turn_set(-220-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(24, 110, false);
        chassis.pid_wait();

        chassis.pid_turn_set(-110-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(36, 110, true);
        chassis.pid_wait();

        chassis.pid_turn_set(0-theta, 100, false);
        chassis.pid_wait();

        chassis.pid_drive_set(40, 110, false);
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(35, 30, false);
        chassis.pid_wait();
        pros::delay(250);

        chassis.pid_turn_set(45-theta, 110, false);
        chassis.pid_wait_quick();
        upper_intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        
        chassis.pid_drive_set(36, 100, false);
        pros::delay(250);
        mogo.set(false);
        pros::delay(100);
        upper_intake.move(-85);
        blue_pause = true;
        chassis.pid_wait_quick();

        chassis.pid_drive_set(6, 50, false);
        chassis.pid_wait();
        
        chassis.pid_turn_set(12-theta, 110, false);
        chassis.pid_wait();
        upper_intake.move(0);
        blue_pause = false;
        upper_intake.move(0);
        

        chassis.pid_drive_set(-32, 60, false);
        chassis.pid_wait();

        mogo.set(true);
        pros::delay(100);
        upper_intake.move(-100);
        pros::delay(250);

        chassis.pid_turn_set(-45-theta, 110, false);
        chassis.pid_wait();
        upper_intake.move(0);

        chassis.pid_drive_set(-12, 110, false);
        chassis.pid_wait_quick();

        lady_brown.move_absolute(0, 50);
        pros::delay(100);
        lower_intake.move(0);

       }, 1, 2, "Blue SAWP", "Blue SAWP", 3, 1, true),

       jas::jasauton([](){
        auton1 = true;
        theta = 30;
        color_sort_blue = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1100, 50);
        pros::delay(600);

        chassis.pid_drive_set(-16, 100, false);
        chassis.pid_wait();

        lady_brown.move_absolute(-450, 200);

        chassis.pid_turn_set(90-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        mogo.set(true);

        chassis.pid_turn_set(210-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(16, 110, false);
        chassis.pid_wait();
        pros::delay(200);

        upper_intake.move(0);
        chassis.pid_turn_set(115-theta, 120, false);
        chassis.pid_wait_quick();
        color_sort_blue = false;
        blue_pause = true;
        upper_intake.move(-100);

        chassis.pid_drive_set(40, 100, true);
        chassis.pid_wait();

        chassis.pid_turn_set(0-theta, 100, false);
        chassis.pid_wait();

        chassis.pid_drive_set(30, 110, false);
        pros::delay(150);
        blue_pause = false;
        color_sort_blue = true;
        chassis.pid_wait_quick_chain();
        
        chassis.pid_drive_set(50, 50, false);
        chassis.pid_wait();
        pros::delay(250);

        chassis.pid_turn_set(-45-theta, 110, false);
        chassis.pid_wait_quick();
        upper_intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        
        chassis.pid_drive_set(36, 100, false);
        pros::delay(250);
        mogo.set(false);
        pros::delay(100);
        upper_intake.move(-85);
        red_pause = true;
        chassis.pid_wait_quick();

        chassis.pid_drive_set(6, 50, false);
        chassis.pid_wait();
        
        chassis.pid_turn_set(-20-theta, 110, false);
        chassis.pid_wait();
        

        chassis.pid_drive_set(-32, 60, false);
        pros::delay(150);
        upper_intake.move(0);
        red_pause = false;
        chassis.pid_wait();

        mogo.set(true);
        pros::delay(100);
        upper_intake.move(-100);
        pros::delay(250);

        chassis.pid_turn_set(45-theta, 110, false);
        chassis.pid_wait();
        upper_intake.move(20);

        chassis.pid_drive_set(-15, 110, false);
        chassis.pid_wait_quick();

        lady_brown.move_absolute(0, 50);
        pros::delay(100);
        lower_intake.move(0);

       }, 0, 2, "Red SAWP", "Red SAWP", 3, 1, true),

       jas::jasauton([](){
        auton1 = true;
        theta = -30;
        color_sort_red = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1100, 50);
        pros::delay(500);

        chassis.pid_drive_set(-14, 110, false);
        chassis.pid_wait();

        lady_brown.move_absolute(-450, 200);

        chassis.pid_turn_set(-90-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        mogo.set(true);

        chassis.pid_turn_set(-220-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(24, 110, false);
        chassis.pid_wait();

        pros::delay(1000);

        chassis.pid_turn_set(-110-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(24, 110, true);
        chassis.pid_wait();
        pros::delay(500);

        chassis.pid_turn_set(35-theta, 100, false);
        chassis.pid_wait();

        chassis.pid_drive_set(32, 50, true);
        pros::delay(200);
        lady_brown.move_absolute(-1000, 200);
        chassis.pid_wait();


       }, 1, 0, "Blue Neg Quals", "Blue Negative Quals", 2, 0, true),

       jas::jasauton([](){
        auton1 = true;
        theta = 30;
        color_sort_blue = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1100, 50);
        pros::delay(500);

        chassis.pid_drive_set(-16, 100, false);
        chassis.pid_wait();

        lady_brown.move_absolute(-450, 200);

        chassis.pid_turn_set(90-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        mogo.set(true);

        chassis.pid_turn_set(220-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(18, 80, false);
        chassis.pid_wait();
        pros::delay(1000);

        upper_intake.move(0);
        chassis.pid_turn_set(115-theta, 120, false);
        chassis.pid_wait_quick();
        color_sort_blue = false;
        blue_pause = true;
        upper_intake.move(-100);

        chassis.pid_drive_set(28, 110, true);
        chassis.pid_wait();
        pros::delay(500);

        chassis.pid_turn_set(-35-theta, 100, false);
        chassis.pid_wait();

        chassis.pid_drive_set(34, 50, true);
        pros::delay(200);
        lady_brown.move_absolute(-1000, 200);
        chassis.pid_wait();

       }, 0, 0, "Red Neg Quals", "Red Negative Quals", 2, 0, true),

       jas::jasauton([](){
        auton1 = true;
        theta = -30;
        color_sort_red = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1100, 50);
        pros::delay(500);

        chassis.pid_drive_set(-14, 110, false);
        chassis.pid_wait();

        lady_brown.move_absolute(-450, 200);

        chassis.pid_turn_set(-90-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        mogo.set(true);

        chassis.pid_turn_set(-220-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(24, 110, false);
        chassis.pid_wait();

        

        chassis.pid_turn_set(-190-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(12, 30, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-12, 80, false);
        chassis.pid_wait();

        chassis.pid_turn_set(-110-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(36, 110, true);
        chassis.pid_wait();

        chassis.pid_turn_set(0-theta, 100, false);
        chassis.pid_wait();

        chassis.pid_drive_set(40, 110, false);
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(35, 50, false);
        chassis.pid_wait();
        

       }, 1, 0, "Blue Neg Elims", "Blue Negative Elims", 3, 0, true),

       jas::jasauton([](){
        auton1 = true;
        theta = 30;
        color_sort_blue = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1100, 50);
        pros::delay(500);

        chassis.pid_drive_set(-18, 80, false);
        chassis.pid_wait();

        lady_brown.move_absolute(-450, 200);

        chassis.pid_turn_set(90-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        mogo.set(true);

        chassis.pid_turn_set(220-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(16, 110, false);
        chassis.pid_wait();

        chassis.pid_turn_set(180-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(14, 30, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-14, 110, false);
        chassis.pid_wait();
        
        chassis.pid_turn_set(190-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(0);
        chassis.pid_turn_set(115-theta, 120, false);
        chassis.pid_wait_quick();
        color_sort_blue = false;
        blue_pause = true;
        upper_intake.move(-100);

        chassis.pid_drive_set(40, 100, true);
        chassis.pid_wait();

        chassis.pid_turn_set(0-theta, 100, false);
        chassis.pid_wait();

        chassis.pid_drive_set(30, 110, false);
        pros::delay(150);
        blue_pause = false;
        color_sort_blue = true;
        upper_intake.move(-100);
        chassis.pid_wait_quick_chain();
        
        chassis.pid_drive_set(50, 50, false);
        chassis.pid_wait();
        pros::delay(250);

       }, 0, 0, "Red Neg Elims", "Red Negative Elims", 3, 1, true),

       jas::jasauton([](){
        auton1 = true;
        theta = 30;
        color_sort_red = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1300, 50);
        pros::delay(500);

        chassis.pid_drive_set(-14, 110, false);
        chassis.pid_wait();

        lady_brown.move_absolute(150, 200);

        chassis.pid_turn_set(0-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(10, 60, false);
        chassis.pid_wait();

        left_doinker.set(true);
        pros::delay(200);

        chassis.pid_drive_set(-10, 40, false);
        chassis.pid_wait();

        chassis.pid_turn_set(90-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        left_doinker.set(false);
        mogo.set(true);

        chassis.pid_turn_set(50-theta, 120, false);
        chassis.pid_wait();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(16, 60, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-16, 60, false);
        chassis.pid_wait();

        chassis.pid_turn_set(-45-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(15);

        chassis.pid_drive_set(16, 60, false);
        chassis.pid_wait();

        left_doinker.set(true);
        pros::delay(200);

        lower_intake.move(-120);

        chassis.pid_turn_set(-60-theta, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-34, 60, false);
        chassis.pid_wait();

        left_doinker.set(false);

        chassis.pid_turn_set(-90-theta, 120, false);
        chassis.pid_wait();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(14, 60, false);
        chassis.pid_wait();
        pros::delay(500);

        chassis.pid_turn_set(-180-theta, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(14, 60, false);
        chassis.pid_wait();

        

        chassis.pid_turn_set(-200-theta, 120, false);
        chassis.pid_wait();

        pros::delay(500);

        chassis.pid_drive_set(-34, 60, false);
        pros::delay(400);
        upper_intake.move(15);
        lower_intake.move(0);
        lady_brown.move_absolute(-200, 200);
        chassis.pid_wait();

       }, 1, 1, "Blue Pos Quals", "Blue Positive Quals", 3, 0, true),
       
       jas::jasauton([](){
        auton1 = true;
        theta = 30;
        color_sort_red = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1100, 50);
        pros::delay(500);

        chassis.pid_drive_set(-14, 110, false);
        chassis.pid_wait();

        lady_brown.move_absolute(150, 200);

        chassis.pid_turn_set(0-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(10, 60, false);
        chassis.pid_wait();

        left_doinker.set(true);
        pros::delay(200);

        chassis.pid_drive_set(-10, 40, false);
        chassis.pid_wait();

        chassis.pid_turn_set(90-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        left_doinker.set(false);
        mogo.set(true);

        chassis.pid_turn_set(50-theta, 120, false);
        chassis.pid_wait();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(16, 60, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-16, 60, false);
        chassis.pid_wait();

        chassis.pid_turn_set(-45-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(15);

        chassis.pid_drive_set(16, 60, false);
        chassis.pid_wait();

        left_doinker.set(true);
        pros::delay(200);

        lower_intake.move(-120);

        chassis.pid_turn_set(-60-theta, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-34, 60, false);
        chassis.pid_wait();

        left_doinker.set(false);
        pros::delay(200);

        chassis.pid_turn_set(-100-theta, 120, false);
        chassis.pid_wait();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(14, 60, false);
        chassis.pid_wait();
        pros::delay(500);

        chassis.pid_turn_set(-180-theta, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(14, 110, false);
        chassis.pid_wait();

        chassis.pid_turn_set(-245-theta, 120, false);
        chassis.pid_wait();

        right_doinker.set(true);

        chassis.pid_drive_set(30, 110, false);
        chassis.pid_wait_quick();

        chassis.pid_turn_set(-360-theta, 110, false);
        chassis.pid_wait_quick_chain();

        right_doinker.set(false);

        chassis.pid_turn_set(-270-theta, 110, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(8, 110, false);
        chassis.pid_wait_quick();

        pros::delay(200);
        
        chassis.pid_drive_set(-30, 60, false);
        chassis.pid_wait_quick_chain();

        chassis.pid_turn_set(-360-theta, 110, false);
        chassis.pid_wait_quick();

       }, 1, 1, "Blue Pos Elims", "Blue Positive Elims", 4, 0, true),

       jas::jasauton([](){
        auton1 = true;
        theta = -30;
        color_sort_blue = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1100, 50);
        pros::delay(500);

        chassis.pid_drive_set(-14, 110, false);
        chassis.pid_wait();

        lady_brown.move_absolute(150, 200);

        chassis.pid_turn_set(0-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(12, 60, false);
        chassis.pid_wait();

        right_doinker.set(true);
        pros::delay(200);

        chassis.pid_drive_set(-12, 40, false);
        chassis.pid_wait();

        chassis.pid_turn_set(-90-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        right_doinker.set(false);
        mogo.set(true);

        chassis.pid_turn_set(-50-theta, 120, false);
        chassis.pid_wait();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(16, 60, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-16, 60, false);
        chassis.pid_wait();

        chassis.pid_turn_set(40-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(15);

        chassis.pid_drive_set(16, 60, false);
        chassis.pid_wait();

        right_doinker.set(true);
        pros::delay(200);

        lower_intake.move(-120);

        chassis.pid_turn_set(60-theta, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-34, 60, false);
        chassis.pid_wait();

        right_doinker.set(false);

        chassis.pid_turn_set(90-theta, 120, false);
        chassis.pid_wait();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(14, 60, false);
        chassis.pid_wait();

        chassis.pid_turn_set(180-theta, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(14, 60, false);
        chassis.pid_wait();

        

        chassis.pid_turn_set(200-theta, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-34, 60, false);
        pros::delay(400);
        upper_intake.move(15);
        lower_intake.move(0);
        lady_brown.move_absolute(-200, 200);
        chassis.pid_wait();

       }, 0, 1, "Red Pos Quals", "Red Positive Quals", 3, 0, true),
       
       jas::jasauton([](){
        auton1 = true;
        theta = -30;
        color_sort_blue = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(-1100, 50);
        pros::delay(500);

        chassis.pid_drive_set(-14, 110, false);
        chassis.pid_wait();

        lady_brown.move_absolute(150, 200);

        chassis.pid_turn_set(5-theta, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(12, 60, false);
        chassis.pid_wait();

        right_doinker.set(true);
        pros::delay(200);

        chassis.pid_drive_set(-12, 60, false);
        chassis.pid_wait();

        chassis.pid_turn_set(-90-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(-28, 60, false);
        chassis.pid_wait();

        right_doinker.set(false);
        mogo.set(true);

        chassis.pid_turn_set(-50-theta, 120, false);
        chassis.pid_wait();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(16, 60, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-16, 60, false);
        chassis.pid_wait();

        chassis.pid_turn_set(40-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(0);

        chassis.pid_drive_set(16, 60, false);
        chassis.pid_wait();

        right_doinker.set(true);
        pros::delay(200);

        lower_intake.move(-120);

        chassis.pid_turn_set(60-theta, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-34, 60, false);
        chassis.pid_wait_quick();

        upper_intake.move(15);

        right_doinker.set(false);

        chassis.pid_turn_set(90-theta, 120, false);
        chassis.pid_wait();

        upper_intake.move(-100);
        lower_intake.move(120);

        chassis.pid_drive_set(16, 60, false);
        chassis.pid_wait();

        chassis.pid_turn_set(180-theta, 120, false);
        chassis.pid_wait();

        chassis.pid_drive_set(14, 110, false);
        chassis.pid_wait();

        chassis.pid_turn_set(245-theta, 120, false);
        chassis.pid_wait();

        left_doinker.set(true);

        chassis.pid_drive_set(28, 110, false);
        chassis.pid_wait_quick_chain();

        chassis.pid_turn_set(360-theta, 110, false);
        chassis.pid_wait_quick_chain();

        left_doinker.set(false);

        chassis.pid_turn_set(270-theta, 110, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(8, 110, false);

        pros::delay(500);
        
        chassis.pid_drive_set(-30, 110, false);
        chassis.pid_wait_quick_chain();

        chassis.pid_turn_set(360-theta, 110, false);
        chassis.pid_wait_quick();

       }, 0, 1, "Red Pos Elims", "Red Positive Elims", 4, 0, true),

       jas::jasauton([]() {
        auton1 = true;
        upper_intake.move(-127);
        pros::delay(500);
        upper_intake.move(0);
        lower_intake.move(127);
        chassis.pid_drive_set(13_in, 100, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(-90, 100, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-24_in, 40, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        chassis.pid_turn_relative_set(90, 100, false);
        chassis.pid_wait_quick();
        upper_intake.move(-127);
        chassis.pid_drive_set(24_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(90, 100, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(24_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(90, 100, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(24_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_drive_set(16_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        pros::delay(500);
        chassis.pid_turn_relative_set(-140, 100, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(24_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(-40, 100, false);
        chassis.pid_wait_quick();
        pros::delay(500);
        upper_intake.move(0);
        mogo.set(false);
        chassis.pid_drive_set(-20_in, 100, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_drive_set(6_in, 100, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(89, 80, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-70_in, 100, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-14_in, 40, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        chassis.pid_turn_relative_set(-90, 120, false);
        chassis.pid_wait_quick();
        upper_intake.move(-127);
        chassis.pid_drive_set(26_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(-90, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(25_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(-90, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(24_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_drive_set(14_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        pros::delay(1000);
        chassis.pid_turn_relative_set(140, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(24_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(40, 120, false);
        chassis.pid_wait_quick();
        pros::delay(500);
        upper_intake.move(0);
        mogo.set(false);
        chassis.pid_drive_set(-20_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        upper_intake.move(-127);
        lady_brown.move_absolute(325, 200);
        chassis.pid_drive_set(53, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(-90, 120, false);
        chassis.pid_wait_quick();
        pros::delay(500);
        upper_intake.move(0);
        lady_brown.move_absolute(1700, 200);
        pros::delay(700);
        chassis.pid_turn_relative_set(-10, 120, false);
        chassis.pid_wait_quick_chain();
        chassis.pid_turn_relative_set(10, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-12_in, 110, true);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(6_in, 110, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        upper_intake.move(-80);
        lady_brown.move_absolute(0, 200);
        red_pause = true;
        chassis.pid_turn_relative_set(110, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(30_in, 110, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(-135, 120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-35_in, 110, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-14_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        mogo.set(true);
        color_sort_blue = true;
        chassis.pid_turn_relative_set(-25,120, false);
        chassis.pid_wait_quick();
        upper_intake.move(-127);
        chassis.pid_drive_set(30_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        upper_intake.move(0);
        chassis.pid_turn_relative_set(90,120, false);
        chassis.pid_wait_quick();
        upper_intake.move(-127);
        chassis.pid_drive_set(36_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        chassis.pid_turn_relative_set(-45,120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(16_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_relative_set(135,120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(24_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        chassis.pid_turn_relative_set(75,120, false);
        chassis.pid_wait_quick();
        pros::delay(500);
        upper_intake.move(0);
        mogo.set(false);
        chassis.pid_drive_set(2_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();;
        lower_intake.move(0);
        
        chassis.pid_drive_set(-20_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_drive_set(10_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(85,120, false);
        chassis.pid_wait_quick();
        chassis.pid_drive_set(110_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-10_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

      }, 2, 2, "Skills", "Skills", 6, 6, true),

        /*
        jas::jasauton([](){
        auton1 = true;


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

        upper_intake.move(-127);
        lower_intake.move(127);

        chassis.pid_drive_set(-36, 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait();
        chassis.pid_drive_set(-8.5_in, 35, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        mogo.set(true);
        chassis.pid_turn_relative_set(45, 90, false);
        upper_intake.move(-127);
        lower_intake.move(127);
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
        auton1 = true;
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
        upper_intake.move(-90);
        lower_intake.move(127);  
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
        upper_intake.move(-127);
        color_sort_red = true;
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(355-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(20_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        lady_brown.move_absolute(800, 200);
        pros::delay(500);
        chassis.pid_drive_set(-50_in, 120, true);  // Move away from wall after alliance stake score
        pros::delay(700);
        upper_intake.move(0);
        lady_brown.move_absolute(0, 10);
        lady_brown_pos = 3;
        lady_brown.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        chassis.pid_wait_quick();
        color_sort_red = true;
      }, 1, 0, "Blue Negative AWP", "Blue 3 rings + Alliance", 3, 0, false),

jas::jasauton([]() {
        
        theta = 192;
        auton1 = true;
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
        upper_intake.move(-90);
        lower_intake.move(127);  
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
        upper_intake.move(-127);
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
        chassis.pid_drive_set(-50_in, 120, true);  // Move away from wall after alliance stake score
        pros::delay(700);
        upper_intake.move(0);
        lady_brown.move_absolute(0, 10);
        lady_brown_pos = 3;
        lady_brown.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        chassis.pid_wait_quick();
        sort_blue_driver = true;
      }, 0, 0, "Red Negative AWP", "Red 3 rings + Alliance", 3, 0, true),

       
        jas::jasauton([]() {
        theta = 168;
        auton1 = true;
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
        upper_intake.move(-90);
        lower_intake.move(127);  
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
        upper_intake.move(-127);
        color_sort_red = true;
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(355-theta, 80, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(30_in, 40, true);  // Move away from wall after alliance stake score
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
        color_sort_red = true;
      }, 1, 0, "Blue Negative Elims", "Blue 4 rings + Alliance", 4, 0, false),

jas::jasauton([]() {
        
        theta = 192;
        auton1 = true;
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
        upper_intake.move(-90);
        lower_intake.move(127);  
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
        upper_intake.move(-127);
        color_sort_blue = true;
        chassis.pid_turn_set(60-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(17_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(10-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(30_in, 40, true);  // Move away from wall after alliance stake score
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
        color_sort_blue = true;
      }, 0, 0, "Red Negative Elims", "Red 4 rings + Alliance", 4, 0, true),
        
      jas::jasauton([]() {
        theta = 168;
        auton1 = true;
        
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);
        pros::delay(1300);

        lady_brown.move_absolute(1900, 200);
        pros::delay(750);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        blue_pause = true;                   
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        upper_intake.move(-90);
        lower_intake.move(127);  
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
        upper_intake.move(-127);
        color_sort_red = true;
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(15_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        chassis.pid_turn_set(355-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(20_in, 120, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        lady_brown.move_absolute(800, 200);
        pros::delay(500);
        chassis.pid_drive_set(-50_in, 120, true);  // Move away from wall after alliance stake score
        pros::delay(700);
        upper_intake.move(0);
        lady_brown.move_absolute(0, 10);
        lady_brown_pos = 3;
        lady_brown.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        chassis.pid_wait_quick();
        color_sort_red = true;
      }, 1, 0, "Blue Negative Wait", "Blue Negative Wait AWP ", 3, 0, false),

jas::jasauton([]() {
        
        theta = 192;
        auton1 = true;
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
        upper_intake.move(-90);
        lower_intake.move(127);  
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
        upper_intake.move(-127);
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
        chassis.pid_drive_set(-50_in, 120, true);  // Move away from wall after alliance stake score
        pros::delay(700);
        upper_intake.move(0);
        lady_brown.move_absolute(0, 10);
        lady_brown_pos = 3;
        lady_brown.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        chassis.pid_wait_quick();
        color_sort_blue = true;
      }, 0, 0, "Red Negative Wait", "Red Negative Wait AWP", 3, 0, true),
      
        jas::jasauton([]() {
        auton1 = true;
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
        upper_intake.move(-90);
        lower_intake.move(127);  
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
        upper_intake.move(-127);
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
        upper_intake.move(-127);
        color_sort_blue = true;
        chassis.pid_drive_set(26_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        chassis.pid_turn_set(-160-theta, 120, false);  // Turn to ring stack
        chassis.pid_drive_set(-43_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_until(-20);

        upper_intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        upper_intake.move(0);
        chassis.pid_wait_quick();
        
        
        
      }, 0, 2, "Sig AWP", "2 Ring + 1 Ring + Alliance", 2, 1, true),

              jas::jasauton([]() {
        
        theta = 192;
        auton1 = true;
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
        upper_intake.move(-90);
        lower_intake.move(127);  
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
        upper_intake.move(-127);
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
        upper_intake.move(-127);
        chassis.pid_drive_set(26_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        color_sort_red = true;

        chassis.pid_turn_set(-160-theta, 120, false);  // Turn to ring stack
        chassis.pid_drive_set(-43_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_until(-20);

        upper_intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        upper_intake.move(0);
        chassis.pid_wait_quick();
        
        
      }, 1, 2, "Sig AWP", "2 Ring + 1 Ring + Alliance", 2, 1, true),
       
      
      
              jas::jasauton([]() {
        
        theta = 60;
        auton1 = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        chassis.pid_drive_set(-20_in, 127, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-10_in, 70, true);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();
        mogo.set(true);
        pros::delay(200);
        upper_intake.move(-127);
        lower_intake.move(127);  
        pros::delay(400);               
        chassis.pid_turn_set(0-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        red_pause = true;
        chassis.pid_drive_set(24_in, 127, true);
        chassis.pid_wait_quick();  // Move away from wall after alliance stake score
        chassis.pid_turn_set(70-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        left_doinker.set(true);
        chassis.pid_drive_set(36_in, 127, true);
        chassis.pid_wait_quick();
        lower_intake.move(0);
        lady_brown.move_absolute(325, 200);

        chassis.pid_turn_set(180-theta, 60, false);      // Turn to mogo
        chassis.pid_wait_quick();
        upper_intake.move(-127);
        
        left_doinker.set(false);
        color_sort_blue = true;
        chassis.pid_turn_set(280-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(40_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        upper_intake.move(0);
        lady_brown.move_absolute(1700, 200);
        lady_brown_pos = 3;
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_turn_set(310-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_turn_set(300-theta, 120, false);      // Turn to mogo
        chassis.pid_wait_quick();
        
        


        
      }, 0, 0, "Cookage", "Cookage", 2, 0, true),
      
              jas::jasauton([]() {
        
        theta = 192;
        auton1 = true;
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
        upper_intake.move(-100);
        lower_intake.move(127);  
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
        upper_intake.move(-127);

        chassis.pid_drive_set(48_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        lower_intake.move(127);
        pros::delay(500);

        pros::delay(200);
        chassis.drive_set(80, 80);
        pros::delay(400);

        pros::delay(400);
        chassis.drive_set(0, 0);
        pros::delay(800);
        chassis.pid_drive_set(-16_in, 40, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_drive_set(8_in, 40, false);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        color_sort_red = true;

        chassis.pid_turn_set(-40-theta, 120, false);
        chassis.pid_drive_set(-70_in, 100, true);
        chassis.pid_wait_until(-20);

        upper_intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        upper_intake.move(0);
        chassis.pid_wait_quick();
        
        
      }, 1, 2, "Normal AWP", "Normal AWP", 2, 0, true),


              jas::jasauton([](){
        chassis.pid_drive_set(-(3 * 12), 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait();
        chassis.pid_drive_set(-8.5_in, 35, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        auton1 = true;
        mogo.set(true);
        chassis.pid_turn_relative_set(-45, 90, false);
        upper_intake.move(-127);
        lady_brown.move_absolute((1875 / 3), 150);
        chassis.pid_wait();
        chassis.pid_drive_set(12, 80, false);
        chassis.pid_wait();

        pros::delay(1000);
        upper_intake.move(0);
        toggle(mogo);
        chassis.pid_turn_set(-110, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_set(25, 90, false);
        chassis.pid_wait();
        upper_intake.move(-127);
        chassis.pid_drive_set(30_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        chassis.pid_drive_set(-8_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        upper_intake.move(0);
        chassis.pid_turn_relative_set(-35, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(10_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        color_sort_blue = true;
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
        upper_intake.move(-127);
        lady_brown.move_absolute((1875 / 3), 150);
        chassis.pid_wait();
        chassis.pid_drive_set(12, 80, false);
        chassis.pid_wait();

        pros::delay(1000);
        upper_intake.move(0);
        toggle(mogo);
        chassis.pid_turn_set(110, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_set(-25, 90, false);
        chassis.pid_wait();
        upper_intake.move(-127);
        chassis.pid_drive_set(30_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        chassis.pid_drive_set(-8_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        upper_intake.move(0);
        chassis.pid_turn_relative_set(35, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(10_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        color_sort_red = true;
        lady_brown.move_absolute(1875, 200);
        pros::delay(500);
        chassis.pid_drive_set(-12, 127, true);
        chassis.pid_wait();
        pros::delay(10000);
       }, 1, 1, "Goal rush AWP", "Goal rush AWP", 1, 1, true),

       jas::jasauton([](){
        chassis.pid_drive_set(30_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        }, 2, 2, "PSU_soloAwpSafe", "PSU_soloAwpSafe", 0, 0, false),

      jas::jasauton([](){
        chassis.pid_drive_set(30_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

        }, 2, 2, "MOVE", "MOVE", 5, 0, false),

      jas::jasauton([](){
        chassis.pid_drive_set(3_in, 120, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();

       }, 2, 2, "MOVE Small", "MOVE Small", 5, 0, false)*/
       });
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
  upper_intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //pros::Task t(auto_clamp_task);
  
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
  upper_intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  swiper.set(false);
  lady_brown.move_absolute(0, 200);
  auto_clamp = false;
  blue_pause = false;
  red_pause = false;

  int reset = false;
  auton1 = false;

  
  
  // l_lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  // r_lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (true) {
    
    //master.print(0, 6, "%f", ring_top);
    
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      // set_lady_brown(lady_brownPID.compute((lady_brown.get_angle() / 100.0)));

      // Trigger the selected autonomous routine
      if (master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_A)) {
        auton1 = true;
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }
      if (master.get_digital_new_press(DIGITAL_X))
          chassis.pid_tuner_toggle();
        chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
      }

      if (master.get_digital_new_press(DIGITAL_R1))
        mogo.set(!mogo.get());
      if (master.get_digital_new_press(DIGITAL_B))
        right_doinker.set(!right_doinker.get());
      if (master.get_digital_new_press(DIGITAL_UP))
        color_sort_blue = !color_sort_blue;
      if (master.get_digital_new_press(DIGITAL_LEFT))
        sort_blue_driver = !sort_blue_driver;


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