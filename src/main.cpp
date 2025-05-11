#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

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


pros::Motor lower_intake(-15);
pros::Motor upper_intake(-10);
pros::Optical color_sensor(2);
pros::Distance ring_check(3);
pros::Motor lady_brown(11);

pros::adi::DigitalIn lady_brown_reset('h');

ez::Piston mogo('a');
ez::Piston left_doinker('g');
ez::Piston right_doinker('f');
ez::Piston yeet('e');

ez::Piston winch_lock('f');
ez::Piston swiper('b');
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
        upper_intake.move(-105);
        blue_ring = false;
      }
    }
    if (color_sort_red){
      if (red_ring && ring_top) {
        upper_intake.move(-127);
        pros::delay(90);
        upper_intake.move(50);
        pros::delay(100);
        upper_intake.move(-105);
        red_ring = false;
      }
    }
    if ((abs(upper_intake.get_voltage()) >= 1000) && (upper_intake.get_efficiency() < 1) && (lady_brown.get_position() < -300) && (lady_brown.get_position() > -600) && auton1){
      upper_intake.move(100);
      pros::delay(200);
      upper_intake.move(-100);
    }
    if (!(pros::competition::is_autonomous() || auton1)){
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        upper_intake.move(70);
      } else if (master.get_digital(DIGITAL_DOWN)) {
        upper_intake.move(100);
      } else if (master.get_digital(DIGITAL_R2)) {
        upper_intake.move(-111);
      } else {
        upper_intake.move(0);
      }
    }
    color = color_sensor.get_hue();
    if (color >= 0 && color <= 15) {red_ring = true; blue_ring = false;}
    if (color >= 200 && color <= 240) {red_ring = false; blue_ring = true;}
    pros::delay(ez::util::DELAY_TIME);
  }
  return -1;
}

int color_sense() {
  return -1;
}

int ring_sense() {
  while (true) {
    if (ring_check.get_distance() < 50) ring_top = true;
    else ring_top = false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return -1;
}

int lady_brown_control() {
  while (true){
    if (!(pros::competition::is_autonomous() || auton1)){
      if ((lady_brown_reset.get_value()) && (lady_brown_pos != 1)){
        pros::delay(100);
        lady_brown.tare_position_all();
        lady_brown.move(0);
        lady_brown_pos = 1;
        pros::delay(50);
        lady_brown.move_absolute(-200, 200);
        pros::delay(100);
      }

      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        if (lady_brown_pos == 1) {
          lady_brown_pos = 2;
          lady_brown.move_absolute(-515, 200);
        } else if (lady_brown_pos == 2) {
          lady_brown_pos = 3;
          pros::delay(50);
          lady_brown.move_absolute(-1000, 200);
        } else if (lady_brown_pos == 3) {
          lady_brown_pos = 4;
          lady_brown.move_absolute(-1425, 200);
        } else if (lady_brown_pos == 4) {
          lady_brown_pos = 5;
          lady_brown.move_absolute(-2000, 200);
        } else {
          lady_brown.move(200);
        }
      }

      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        if (lady_brown_pos == 5){
          lady_brown.move(0);
        } else {
        lady_brown_pos = 5;
        lady_brown.move(200);
        }
      }

      if (master.get_digital_new_press(DIGITAL_X)){
        lady_brown_pos = 5;
        lady_brown.move_absolute(-2000, 200);
      }
    }
    pros::delay(ez::util::DELAY_TIME);
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
  pros::Task t3(ring_sense);
  lady_brown.set_brake_mode_all(MOTOR_BRAKE_HOLD);

  color_sensor.set_led_pwm(100);
  j_auton_selector.jautonpopulate(
      { 
        jas::jasauton([](){
        
          auton1 = true;
          theta = -58;
          color_sort_red = true;
          chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  
          lady_brown.move_absolute(-1800, 200);
  
          chassis.pid_drive_set(4.5, 110, false);
          
          pros::delay(700);
  
          lady_brown.move_absolute(300, 200);
  
          chassis.pid_drive_set(-30, 60, false);
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
          theta = 58;
          color_sort_blue = true;
          chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  
          lady_brown.move_absolute(-1800, 200);
  
          chassis.pid_drive_set(4.5, 110, false);
          
          pros::delay(700);
  
          lady_brown.move_absolute(300, 200);
  
          chassis.pid_drive_set(-30, 60, false);
          chassis.pid_wait();
  
          mogo.set(true);

  
          chassis.pid_turn_set(180-theta, 120, false);
          chassis.pid_wait_quick();
  
          upper_intake.move(-100);
          lower_intake.move(120);
  
          chassis.pid_drive_set(16, 110, false);
          chassis.pid_wait();
          pros::delay(200);
  
          upper_intake.move(0);
          chassis.pid_turn_set(30-theta, 120, false);
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
  
          lady_brown.move_absolute(0, 200);
          pros::delay(100);
          lower_intake.move(0);
  
         }, 0, 2, "Red SAWP", "Red SAWP", 3, 1, true),

  
         jas::jasauton([](){
          
          auton1 = true;
          theta = -58;
          color_sort_red = true;
          chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  
          lady_brown.move_absolute(-1600, 152);
  
          chassis.pid_drive_set(4.5, 110, false);
          
          pros::delay(900);
  
          chassis.pid_drive_set(-31, 50, false);
          chassis.pid_wait();
  
          lady_brown.move_absolute(300, 200);
  
          mogo.set(true);
  
          chassis.pid_turn_set(-220-theta, 120, false);
          chassis.pid_wait_quick();
  
          upper_intake.move(-100);
  
          chassis.pid_drive_set(22, 90, false);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-185-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(15, 80, true);
          chassis.pid_wait();
  
          chassis.pid_drive_set(-10, 110, true);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-100-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(45, 80, true);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-135-theta, 100, false);
          chassis.pid_wait();
  
          chassis.pid_drive_set(30, 50, true);
          pros::delay(1300);
  
          chassis.pid_drive_set(-15, 50, true);
          chassis.pid_wait();
  
          pros::delay(400);
  
          chassis.pid_drive_set(15, 50, true);
          chassis.pid_wait();
  
          chassis.pid_drive_set(-25, 50, true);
          chassis.pid_wait_quick_chain();
  
          upper_intake.move(0);
  
          chassis.pid_turn_set(55-theta, 120, false);
          chassis.pid_wait_quick_chain();
  
          chassis.pid_drive_set(15, 100, true);
          chassis.pid_wait_quick_chain();
  
          lady_brown.move_absolute(-1500, 200);
          
          lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
          upper_intake.move(-115);
          pros::delay(800);
          lady_brown.move(10);
          pros::delay(1000);
          lady_brown.move(0);
  
         }, 1, 0, "Blue Neg Quals", "Blue Negative Quals", 5, 0, true),

         jas::jasauton([](){
          
          auton1 = true;
          theta = -58;
          color_sort_red = true;
          chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  
          chassis.pid_drive_set(-26, 50, false);
          chassis.pid_wait();
  
          mogo.set(true);
  
          chassis.pid_turn_set(-220-theta, 120, false);
          chassis.pid_wait_quick();
  
          upper_intake.move(-100);
  
          chassis.pid_drive_set(22, 90, false);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-185-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(15, 80, true);
          chassis.pid_wait();
  
          chassis.pid_drive_set(-10, 110, true);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-100-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(45, 80, true);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-135-theta, 100, false);
          chassis.pid_wait();
  
          chassis.pid_drive_set(30, 50, true);
          pros::delay(1300);
  
          chassis.pid_drive_set(-15, 50, true);
          chassis.pid_wait();
  
          pros::delay(400);
  
          chassis.pid_drive_set(15, 50, true);
          chassis.pid_wait();
  
          chassis.pid_drive_set(-25, 50, true);
          chassis.pid_wait_quick_chain();
  
          upper_intake.move(0);
  
          chassis.pid_turn_set(55-theta, 120, false);
          chassis.pid_wait_quick_chain();
  
          chassis.pid_drive_set(15, 100, true);
          chassis.pid_wait_quick_chain();
  
          lady_brown.move_absolute(-1500, 200);
          
          lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
          upper_intake.move(-115);
          pros::delay(800);
          lady_brown.move(10);
          pros::delay(1000);
          lady_brown.move(0);
  
         }, 1, 0, "Blue Neg Quals No WS", "Blue Negative Quals No Wallstake", 6, 0, true),
  
         jas::jasauton([](){
          
          auton1 = true;
          theta = -58;
          color_sort_red = true;
          chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  
          lady_brown.move_absolute(-1600, 152);
  
          chassis.pid_drive_set(4.5, 110, false);
          
          pros::delay(900);
  
          chassis.pid_drive_set(-31, 50, false);
          chassis.pid_wait();
  
          lady_brown.move_absolute(300, 200);
  
          mogo.set(true);
  
          chassis.pid_turn_set(-220-theta, 120, false);
          chassis.pid_wait_quick();
  
          upper_intake.move(-100);
  
          chassis.pid_drive_set(22, 90, false);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-185-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(15, 80, true);
          chassis.pid_wait();
  
          chassis.pid_drive_set(-10, 110, true);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-100-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(45, 80, true);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-135-theta, 100, false);
          chassis.pid_wait();
  
          chassis.pid_drive_set(30, 50, true);
          pros::delay(1300);
  
          chassis.pid_drive_set(-15, 50, true);
          chassis.pid_wait();
  
          pros::delay(400);
  
          chassis.pid_drive_set(15, 50, true);
          chassis.pid_wait();
  
          chassis.pid_drive_set(-18, 50, true);
          chassis.pid_wait_quick();
  
          chassis.pid_turn_set(5-theta, 100, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(30, 100, true);
          chassis.pid_wait_quick_chain();
  
          chassis.pid_drive_set(30, 60, true);
          chassis.pid_wait_quick_chain();

          chassis.pid_drive_set(20, 80, true);
          chassis.pid_wait();
  
          pros::delay(2000);
  
  
         }, 1, 0, "Blue Neg Elims", "Blue Negative Elims", 6, 0, true),
  
         jas::jasauton([](){
          
          auton1 = true;
          theta = -58;
          color_sort_blue = true;
          chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  
          lady_brown.move_absolute(-1600, 152);
  
          chassis.pid_drive_set(4.5, 110, false);
          
          pros::delay(900);        
  
          chassis.pid_drive_set(-32, 50, false);
          chassis.pid_wait();
  
          lady_brown.move_absolute(300, 200);
  
          mogo.set(true);
  
          chassis.pid_turn_set(-180-theta, 120, false);
          chassis.pid_wait_quick();
  
          upper_intake.move(-100);
  
          chassis.pid_drive_set(22, 90, false);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-90-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(26, 80, true);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-135-theta, 100, false);
          chassis.pid_wait();
  
          chassis.pid_drive_set(30, 50, true);
          pros::delay(1300);
  
          chassis.pid_drive_set(-20, 70, true);
          chassis.pid_wait();
  
          pros::delay(400);
  
          chassis.pid_drive_set(20, 70, true);
          chassis.pid_wait();
  
          chassis.pid_drive_set(-25, 50, true);
          chassis.pid_wait_quick_chain();
  
          upper_intake.move(0);
  
          chassis.pid_turn_set(55-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(15, 100, true);
          chassis.pid_wait_quick_chain();
  
          lady_brown.move_absolute(-1500, 200);
          
          lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
          upper_intake.move(-100);
          pros::delay(800);
          lady_brown.move(10);
          pros::delay(1000);
          lady_brown.move(0);
  
  
         }, 0, 1, "Red Pos Quals", "Red Positive Quals", 3, 0, true),

         jas::jasauton([](){
          
          auton1 = true;
          theta = -58;
          color_sort_blue = true;
          chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); 
  
          chassis.pid_drive_set(-28, 50, false);
          chassis.pid_wait();
  
          mogo.set(true);
  
          chassis.pid_turn_set(-180-theta, 120, false);
          chassis.pid_wait_quick();
  
          upper_intake.move(-100);
  
          chassis.pid_drive_set(22, 90, false);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-90-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(26, 80, true);
          chassis.pid_wait();
  
          chassis.pid_turn_set(-135-theta, 100, false);
          chassis.pid_wait();
  
          chassis.pid_drive_set(30, 50, true);
          pros::delay(1300);
  
          chassis.pid_drive_set(-20, 70, true);
          chassis.pid_wait();
  
          pros::delay(400);
  
          chassis.pid_drive_set(20, 70, true);
          chassis.pid_wait();
  
          chassis.pid_drive_set(-25, 50, true);
          chassis.pid_wait_quick_chain();
  
          upper_intake.move(0);
  
          chassis.pid_turn_set(55-theta, 120, false);
          chassis.pid_wait_quick();
  
          chassis.pid_drive_set(15, 100, true);
          chassis.pid_wait_quick_chain();
  
          lady_brown.move_absolute(-1500, 200);
          
          lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
          upper_intake.move(-100);
          pros::delay(800);
          lady_brown.move(10);
          pros::delay(1000);
          lady_brown.move(0);
  
  
         }, 0, 1, "Red Pos Quals No WS", "Red Positive Quals No Wallstake", 3, 0, true),
  
        jas::jasauton([](){
         
         auton1 = true;
         theta = -58;
         color_sort_blue = true;
         chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  
         lady_brown.move_absolute(-1600, 152);
  
         chassis.pid_drive_set(4.5, 110, false);
         
         pros::delay(800);        
  
         chassis.pid_drive_set(-32, 60, false);
         chassis.pid_wait();
  
         lady_brown.move_absolute(300, 200);
  
         mogo.set(true);
  
         chassis.pid_turn_set(-180-theta, 110, true);
         chassis.pid_wait_quick();
  
         upper_intake.move(-70);
  
         chassis.pid_drive_set(22, 90, false);
         chassis.pid_wait();
  
         lady_brown.move(100);
  
         while (!lady_brown_reset.get_value()) pros::delay(10);
  
         lady_brown.move(0);
         lady_brown_pos = 1;
         pros::delay(150);
         lady_brown.tare_position_all();
         lady_brown.set_brake_mode_all(MOTOR_BRAKE_HOLD);
  
         pros::delay(150);
  
         upper_intake.move(-100);
  
         chassis.pid_turn_set(-90-theta, 120, false);
         chassis.pid_wait_quick();
  
         chassis.pid_drive_set(24, 90, true);
         chassis.pid_wait();
  
         chassis.pid_turn_set(-135-theta, 100, false);
         chassis.pid_wait();
  
         chassis.pid_drive_set(10, 100, true);
         chassis.pid_wait_quick();
  
         chassis.pid_drive_set(40, 30, false);
         pros::delay(1200);
  
         chassis.pid_drive_set(-20, 70, true);
         chassis.pid_wait();
  
         //right_doinker.set(true);
  
         pros::delay(400);
  
         lady_brown.tare_position_all();
  
         lady_brown.move_absolute(-300, 100);
  
         chassis.pid_drive_set(20, 70, true);
         chassis.pid_wait_quick();
  
         blue_pause = true;
  
         lady_brown.move_absolute(-300, 100);
  
         chassis.pid_drive_set(-5, 70, true);
         chassis.pid_wait_quick();
  
         chassis.pid_turn_set(-250-theta, 100, false);
         chassis.pid_wait();
  
         //right_doinker.set(false);
  
         upper_intake.move(-127);
         blue_pause = true;
  
         chassis.pid_drive_set(36, 100, true);
         chassis.pid_wait();
  
         upper_intake.move(100);
         pros::delay(100);
         upper_intake.move(0);
  
         chassis.pid_turn_set(-220-theta, 100, false);
         chassis.pid_wait();
  
         
         chassis.pid_drive_set(7, 70, true);
         chassis.pid_wait();
  
         if (blue_pause) lady_brown.move_absolute(-1100, 150);
  
  
        }, 0, 1, "Red Pos Elims", "Red Positive Elims", 2, 1, true),

       jas::jasauton([](){
        
        auton1 = true;
        theta = 58;
        color_sort_blue = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

        lady_brown.move_absolute(-1600, 152);

        chassis.pid_drive_set(4.5, 110, false);
        
        pros::delay(900);        

        chassis.pid_drive_set(-30, 50, false);
        chassis.pid_wait();

        lady_brown.move_absolute(200, 200);

        mogo.set(true);

        chassis.pid_turn_set(220-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);

        chassis.pid_drive_set(22, 110, false);
        chassis.pid_wait();

        chassis.pid_turn_set(188-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(15, 80, true);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(-10, 110, true);
        chassis.pid_wait_quick();

        chassis.pid_turn_set(100-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(45, 80, true);
        chassis.pid_wait();

        chassis.pid_turn_set(135-theta, 100, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(30, 60, true);
        pros::delay(1000);

        chassis.pid_drive_set(-15, 50, true);
        chassis.pid_wait_quick();

        pros::delay(400);

        chassis.pid_drive_set(15, 50, true);
        chassis.pid_wait_quick();

        pros::delay(300);

        chassis.pid_drive_set(-25, 40, true);
        chassis.pid_wait_quick();

        upper_intake.move(0);

        chassis.pid_turn_set(-55-theta, 120, false);
        chassis.pid_wait_quick();


        chassis.pid_drive_set(30, 120, true);
        chassis.pid_wait_quick();


        lady_brown.move_absolute(-1500, 200);
        
        lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        upper_intake.move(-115);
        pros::delay(800);
        lady_brown.move(20);
        pros::delay(1000);
        lady_brown.move(0);

       }, 0, 0, "Red Neg Quals", "Red Negative Quals", 5, 0, true),

       jas::jasauton([](){
        
        auton1 = true;
        theta = 58;
        color_sort_blue = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    

        chassis.pid_drive_set(-25, 50, false);
        chassis.pid_wait();

        mogo.set(true);

        chassis.pid_turn_set(220-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);

        chassis.pid_drive_set(22, 110, false);
        chassis.pid_wait();

        chassis.pid_turn_set(188-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(15, 80, true);
        chassis.pid_wait();

        chassis.pid_drive_set(-10, 110, true);
        chassis.pid_wait();

        chassis.pid_turn_set(100-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(45, 80, true);
        chassis.pid_wait_quick();

        chassis.pid_turn_set(135-theta, 100, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(30, 60, true);
        pros::delay(1000);

        chassis.pid_drive_set(-15, 50, true);
        chassis.pid_wait_quick();

        pros::delay(400);

        chassis.pid_drive_set(15, 50, true);
        chassis.pid_wait_quick();

        pros::delay(300);

        chassis.pid_drive_set(-25, 40, true);
        chassis.pid_wait_quick();

        upper_intake.move(0);

        chassis.pid_turn_set(-55-theta, 120, false);
        chassis.pid_wait_quick();


        chassis.pid_drive_set(30, 120, true);
        chassis.pid_wait_quick();


        lady_brown.move_absolute(-1500, 200);
        
        lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        upper_intake.move(-115);
        pros::delay(800);
        lady_brown.move(20);
        pros::delay(1000);
        lady_brown.move(0);

       }, 0, 0, "Red Neg Quals No WS", "Red Negative Quals No Wallstake", 5, 0, true),

       jas::jasauton([](){
        
        auton1 = true;
        theta = 58;
        color_sort_blue = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

        lady_brown.move_absolute(-1600, 175);

        chassis.pid_drive_set(4, 110, false);
        
        pros::delay(600);         

        chassis.pid_drive_set(-20, 80, false);
        chassis.pid_wait();

        chassis.pid_drive_set(-10, 40, false);
        chassis.pid_wait();

        lady_brown.move_absolute(200, 125);

        mogo.set(true);

        chassis.pid_turn_set(220-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-110);

        chassis.pid_drive_set(20, 110, false);
        chassis.pid_wait();

        chassis.pid_turn_set(192-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(15, 60, true);
        chassis.pid_wait();

        chassis.pid_drive_set(-12, 110, true);
        chassis.pid_wait();

        upper_intake.move(-110);

        chassis.pid_turn_set(98-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(45, 80, true);
        chassis.pid_wait();

        chassis.pid_turn_set(135-theta, 100, false);
        chassis.pid_wait_quick_chain();

        chassis.pid_drive_set(30, 50, true);
        pros::delay(800);

        chassis.pid_drive_set(-15, 70, true);
        chassis.pid_wait_quick();

        pros::delay(300);

        chassis.pid_drive_set(13, 70, true);
        chassis.pid_wait_quick();

        pros::delay(300);

        chassis.pid_drive_set(-18, 100, true);
        chassis.pid_wait_quick();

        chassis.pid_turn_set(-0-theta, 100, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(65, 120, true);
        pros::delay(200);
        upper_intake.move(110);
        pros::delay(200);
        upper_intake.move(-110);
        chassis.pid_wait_quick_chain();

        chassis.pid_drive_set(15, 50, true);
        chassis.pid_wait_quick();

        pros::delay(2000);


       }, 0, 0, "Red Neg Elims", "Red Negative Elims", 6, 0, true),

       jas::jasauton([](){
        
        auton1 = true;
        theta = 58;
        color_sort_red = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

        lady_brown.move_absolute(-1600, 152);

        chassis.pid_drive_set(4.5, 110, false);
        
        pros::delay(900);        

        chassis.pid_drive_set(-32, 60, false);
        chassis.pid_wait();

        lady_brown.move_absolute(200, 200);

        mogo.set(true);

        chassis.pid_turn_set(180-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);

        chassis.pid_drive_set(22, 90, false);
        chassis.pid_wait();

        chassis.pid_turn_set(90-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(28, 80, true);
        chassis.pid_wait();

        chassis.pid_turn_set(135-theta, 100, false);
        chassis.pid_wait();

        chassis.pid_drive_set(30, 50, true);
        pros::delay(1400);

        chassis.pid_drive_set(-20, 70, true);
        chassis.pid_wait();

        pros::delay(400);

        chassis.pid_drive_set(20, 70, true);
        chassis.pid_wait();

        chassis.pid_drive_set(-25, 50, true);
        chassis.pid_wait_quick_chain();

        upper_intake.move(0);

        chassis.pid_turn_set(-55-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(25, 100, true);
        chassis.pid_wait_quick_chain();

        lady_brown.move_absolute(-1500, 200);
        
        lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        upper_intake.move(-115);
        pros::delay(800);
        lady_brown.move(10);
        pros::delay(1000);
        lady_brown.move(0);


       }, 1, 1, "Blue Pos Quals", "Blue Positive Quals", 3, 0, true),

       jas::jasauton([](){
        
        auton1 = true;
        theta = 58;
        color_sort_red = true;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

        chassis.pid_drive_set(-27, 60, false);
        chassis.pid_wait();

        mogo.set(true);

        chassis.pid_turn_set(180-theta, 120, false);
        chassis.pid_wait_quick();

        upper_intake.move(-100);

        chassis.pid_drive_set(22, 90, false);
        chassis.pid_wait();

        chassis.pid_turn_set(90-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(28, 80, true);
        chassis.pid_wait();

        chassis.pid_turn_set(135-theta, 100, false);
        chassis.pid_wait();

        chassis.pid_drive_set(30, 50, true);
        pros::delay(1400);

        chassis.pid_drive_set(-20, 70, true);
        chassis.pid_wait();

        pros::delay(400);

        chassis.pid_drive_set(20, 70, true);
        chassis.pid_wait();

        chassis.pid_drive_set(-25, 50, true);
        chassis.pid_wait_quick_chain();

        upper_intake.move(0);

        chassis.pid_turn_set(-55-theta, 120, false);
        chassis.pid_wait_quick();

        chassis.pid_drive_set(25, 100, true);
        chassis.pid_wait_quick_chain();

        lady_brown.move_absolute(-1500, 200);
        
        lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        upper_intake.move(-115);
        pros::delay(800);
        lady_brown.move(10);
        pros::delay(1000);
        lady_brown.move(0);


       }, 1, 1, "Blue Pos Quals No WS", "Blue Positive Quals No Wallstake", 3, 0, true),

      jas::jasauton([](){
       
       auton1 = true;
       theta = 58;
       color_sort_red = true;
       chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

       lady_brown.move_absolute(-1600, 152);

       chassis.pid_drive_set(4.5, 110, false);
       
       pros::delay(800);        

       chassis.pid_drive_set(-32, 60, false);
       chassis.pid_wait();

       lady_brown.move_absolute(200, 200);

       mogo.set(true);

       chassis.pid_turn_set(180-theta, 110, true);
       chassis.pid_wait_quick();

       upper_intake.move(-70);

       chassis.pid_drive_set(22, 90, false);
       chassis.pid_wait();

       lady_brown.move(100);

       while (!lady_brown_reset.get_value()) pros::delay(10);

       lady_brown.move(0);
       lady_brown_pos = 1;
       pros::delay(150);
       lady_brown.tare_position_all();
       lady_brown.set_brake_mode_all(MOTOR_BRAKE_HOLD);

       pros::delay(150);

       upper_intake.move(-100);

       chassis.pid_turn_set(90-theta, 120, false);
       chassis.pid_wait_quick();

       chassis.pid_drive_set(24, 90, true);
       chassis.pid_wait();

       chassis.pid_turn_set(135-theta, 100, false);
       chassis.pid_wait();

       chassis.pid_drive_set(10, 100, true);
       chassis.pid_wait_quick();

       chassis.pid_drive_set(40, 30, false);
       pros::delay(1200);

       chassis.pid_drive_set(-20, 70, true);
       chassis.pid_wait();

       //right_doinker.set(true);

       pros::delay(400);

       lady_brown.tare_position_all();

       lady_brown.move_absolute(-200, 60);

       chassis.pid_drive_set(20, 70, true);
       chassis.pid_wait_quick();

       blue_pause = true;

       lady_brown.move_absolute(-200, 60);

       chassis.pid_drive_set(-5, 70, true);
       chassis.pid_wait_quick();

       chassis.pid_turn_set(250-theta, 100, false);
       chassis.pid_wait();

       //right_doinker.set(false);

       upper_intake.move(-127);
       blue_pause = true;

       chassis.pid_drive_set(36, 100, true);
       chassis.pid_wait();

       upper_intake.move(100);
       pros::delay(100);
       upper_intake.move(0);

       chassis.pid_turn_set(220-theta, 100, false);
       chassis.pid_wait();

       
       chassis.pid_drive_set(7, 70, true);
       chassis.pid_wait();

       if (blue_pause) lady_brown.move_absolute(-1100, 150);

      }, 1, 1, "Blue Pos Elims", "Blue Positive Elims", 2, 1, true),
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
  auton1 = false;
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
    
    master.print(0, 0, "%f", chassis.drive_imu_get());
    
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
      if (master.get_digital_new_press(DIGITAL_UP)){
        color_sort_blue = false;
        color_sort_red = false;
      }
      if (master.get_digital_new_press(DIGITAL_LEFT))
        yeet.set(!yeet.get());

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