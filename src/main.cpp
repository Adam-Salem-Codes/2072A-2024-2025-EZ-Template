#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {4, -3, -2},  // Left Chassis Ports (negative port will reverse it!)
    {-8, 9, 7},   // Right Chassis Ports (negative port will reverse it!)

    11,    // IMU Port
    2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);  // Wheel RPM

// pros::Motor l_lift(-14);
// pros::Motor r_lift(17);
pros::Motor intake(10);
pros::Optical color_sensor(5);
pros::Distance front_distance(6);
pros::Rotation lift(12);
ez::Piston mogo('a');
ez::Piston swiper('b');
ez::Piston intake_lift('h');

pros::MotorGroup lady_brown({-14, 17}, pros::MotorGears::rpm_200, pros::MotorEncoderUnits::counts);
// void set_lift(int input) {
//  l_lift.move(input);
//  r_lift.move(input);
//}
ez::PID liftPID{0.45, 0, 0, 0, "Lift"};
pros::Distance clamp_sensor(1);
// vector<jas::motors::motordata> motorbar{
//     {0, intake, "intake 1"},
// };

int auto_clamp_task() {
  while (true) {
    if (clamp_sensor.get_distance() <= 25) {
      pros::delay(250);
      // toggle clamp
      mogo.set(true);
      pros::delay(2500);
    }
    pros::delay(10);
  }

  master.rumble(".");
}
int intake_torque_task() {
  while (true) {
    if (intake.get_torque() > 1.0 && intake.get_power() < 10.0) {  // Check if intake torque exceeds 1.0 Nm and power is below 10 Watts
      intake.move(127);                                            // Reverse the intake
      pros::delay(500);                                            // Wait for 0.5 seconds
      intake.move(-127);                                           // Stop the intake
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
  chassis.pid_wait_quick_chain();                 // Motion chain into slower speed
  while (!mogo.get()) {
    chassis.drive_set(lower_speed, lower_speed);  // Go at a lower speed until clamp extends.
    pros::delay(10);
  }
}
bool racism_against_blue = false;
int color_sensor_task() {
  bool lastReverse = false;
  while (true) {
    if (
        (racism_against_blue && (color_sensor.get_hue() >= 180 && color_sensor.get_hue() <= 270)) ||
        (!racism_against_blue && (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 25))) {
      pros::delay(100);
      intake.move(50);
      pros::delay(300);
      intake.move(-127);
    }
  }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure
  // l_lift.tare_position();
  // l_lift.set_encoder_units(pros::MotorEncoderUnits::degrees);
  lift.reset();
  lift.reset_position();
  /*j_auton_selector.jautonpopulate(
      {jas::jasauton(drive_example, 1, 1, "Blue 50% WP", "Blue 2 ring in positive corner", 2, 0, false),
       jas::jasauton(drive_example, 1, 2, "Blue test auton", "Testing for blue autons", 0, 0, false),

       jas::jasauton(drive_example, 0, 1, "Red 50% WP", "Red 2 ring in positive corner", 2, 0, false),
       jas::jasauton(drive_example, 0, 2, "Red test auton", "Testing for red autons", 0, 0, false),

       jas::jasauton(drive_example, 1, 0, "Blue 4 ring no WP", "Blue 4 ring in negative corner", 4, 0, false),
       jas::jasauton(drive_example, 1, 0, "Blue 4 ring WP", "Blue 3 ring in negative corner + 1 ring on alliance", 3, 0, true),
       jas::jasauton(drive_example, 1, 0, "Blue 6 ring no WP", "Blue 6 ring in negative corner", 6, 0, false),

       jas::jasauton(drive_example, 0, 0, "Red 4 ring no WP", "Red 4 ring in negative corner", 4, 0, false),
       jas::jasauton(drive_example, 0, 0, "Red 4 ring WP", "Red 3 ring in negative corner + 1 ring on alliance", 3, 0, true),
       jas::jasauton(drive_example, 0, 0, "Red 6 ring no WP", "Red 6 ring in negative corner", 6, 0, false),
       jas::jasauton(drive_example, 0, 2, "Old skills auton", "Unfinished, inconsistent skills auton", 6, 6, true),
       jas::jasauton(drive_example, 0, 2, "New skills auton", "Potential 50 pt skills auton- does not work yet, takes too long", 5, 6, true),

       jas::jasauton(drive_example, 2, 2, "Move forward", "Drive straight forward", 0, 0, false)});*/
  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      Auton("Blue Negative \n Gets 1 blue on alliance stake \n Gets 3 on mogo", []() {
        // ZACH: If we want the auton faster we can turn off slew (accelerations)
        racism_against_blue = false;
        chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        lady_brown.set_zero_position_all(0);

        lady_brown.move_absolute(1900, 200);
        pros::delay(850);
        lady_brown.move_absolute(50, 200);
        chassis.pid_drive_set(-12_in, 127, false);  // Move away from wall after alliance stake score
        chassis.pid_wait_quick();                   // Quick chain into the turn
        chassis.pid_turn_set(-80, 127, false);      // Turn to mogo
        chassis.pid_wait_quick();
        chassis.pid_drive_set(-30_in, 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        // TODO: CURRENTLY THIS IS PRESET BUT IN THE FUTURE MAKE IT GO BACK UNTIL THE AUTO CLAMP ACTIVATES.
        chassis.pid_drive_set(-6_in, 90, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();
        chassis.pid_turn_set(-170, 110, false);  // Turn to ring stack
        chassis.pid_wait_quick();
        intake.move(-127);                         // Start intake before running into ring stack
        chassis.pid_drive_set(23_in, 127, false);  // Move to ring stack
        chassis.pid_wait_quick_chain();
        pros::delay(325);                              // WAIT FOR RING TO GET INTO INTAKE
        intake.move(0);                                // Stop intake to avoid intaking red one when  going towards the wall to line up
        move_to_distance_from_wall(14, 90);            // Move 14" away from the wall to align for two rings
        chassis.pid_wait_quick();                      // Wait for move_to_distance_from_wall to finish
        chassis.pid_turn_relative_set(-85, 90, true);  // Turn to ring stack
        chassis.pid_wait();
        intake.move(-127);                       // Move intake for the rest of the auton
        toggle(swiper);                          // Swiper down
        pros::delay(200);                        // Wait for swiper to settle down
        chassis.pid_drive_set(14_in, 40, true);  // Get rings
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-15_in, 80, true);  // Drag them back
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-90, 40, true);  // Turn them away from the left behind red ring and the bar
        chassis.pid_wait();
        toggle(swiper);  // Swiper up to intake them
        chassis.pid_turn_relative_set(15, 90, true);
        chassis.pid_wait_quick_chain();
        // Move around to intake rings
        chassis.pid_drive_set(10_in, 60, true);
        chassis.pid_wait_quick_chain();
        chassis.pid_turn_relative_set(15, 60, false);
        chassis.pid_wait_quick_chain();
        // Move around to intake rings
        chassis.pid_drive_set(19_in, 127, true);  // Goto bar
        chassis.pid_wait();
        pros::delay(350);  // Make sure the intake finishes scoring.
        intake.move(0);
        // swiper.set(!swiper.get());
      }),
      Auton("Goal Rush AWP", []() {
        chassis.pid_drive_set(-(3 * 12), 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8.5_in, 35, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        mogo.set(true);
        chassis.pid_turn_relative_set(45, 90, false);
        intake.move(-127);
        lady_brown.move_absolute((1875 / 3), 150);
        chassis.pid_wait();
        chassis.pid_drive_set(12, 80, false);
        chassis.pid_wait();

        pros::delay(1000);
        intake.move(0);
        toggle(mogo);
        chassis.pid_turn_set(110, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_set(-25, 90, false);
        chassis.pid_wait_quick();
        intake_lift.set(!intake_lift.get());
        intake.move(-127);
        chassis.pid_drive_set(30_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        chassis.pid_drive_set(-8_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        intake_lift.set(!intake_lift.get());
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        intake.move(0);
        chassis.pid_turn_relative_set(35, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(10_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        lady_brown.move_absolute(1875, 200);
        pros::delay(500);
        chassis.pid_drive_set(-12, 127, true);
        chassis.pid_wait();
        pros::delay(10000);
      }),
      Auton("ELIMS GOAL RUSH ", []() {
        chassis.pid_drive_set(-(3 * 12), 127, false);  // Move the majority of the distance to the mogo
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(-8.5_in, 35, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        mogo.set(true);
        chassis.pid_turn_relative_set(45, 90, false);
        intake.move(-127);
        lady_brown.move_absolute((1875 / 3), 150);
        chassis.pid_wait();
        chassis.pid_drive_set(12, 80, false);
        chassis.pid_wait();

        pros::delay(1000);
        intake.move(0);
        toggle(mogo);
        chassis.pid_turn_set(110, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 50, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_set(-25, 90, false);
        chassis.pid_wait_quick();
        intake_lift.set(!intake_lift.get());
        intake.move(-127);
        chassis.pid_drive_set(30_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick();

        chassis.pid_drive_set(-8_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        intake_lift.set(!intake_lift.get());
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(6_in, 25, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait_quick_chain();
        intake.move(0);
        chassis.pid_turn_relative_set(165, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(65_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        lady_brown.move_absolute(1775, 200);
        chassis.pid_turn_relative_set(20, 90, false);
        chassis.pid_drive_set(5_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-15, 90, false);
        chassis.pid_wait();
        chassis.pid_turn_relative_set(15, 90, false);
        chassis.pid_wait();

        pros::delay(5000);
        chassis.pid_drive_set(-12, 127, true);
        chassis.pid_wait();
        pros::delay(10000);
      }),

      Auton("Skeeeeells", []() {
        intake.move(-127);
        pros::delay(500);
        intake.move(0);
        chassis.pid_drive_set(12_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        chassis.pid_turn_relative_set(-90, 90, false);
        chassis.pid_wait();
        chassis.pid_drive_set(-20_in, 80, true);  // Slow down before reaching the mobile goal to clamp correctly.
        chassis.pid_wait();
        mogo.set(true);
        chassis.pid_turn_relative_set(90, 90, false);
        chassis.pid_wait();
        intake.move(-127);
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
        intake.move(0);
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
      }),
      Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
      Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
      Auton("Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  // pros::Task tempcheckcontroller(tempcheckctrl);
  // screeninit();
  // tempcheck();
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
  pros::Task t(auto_clamp_task);
  pros::Task t1(color_sensor_task);
  // pros::Task t2(intake_torque_task);
  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
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
  int lift_prime = 1;
  // l_lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  // r_lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (true) {
    color_sensor.set_led_pwm(100);
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X))
        chassis.pid_tuner_toggle();
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        if (lift_prime == 1) {
          lift_prime = 2;
          lady_brown.set_brake_mode_all(MOTOR_BRAKE_HOLD);
          lady_brown.move_absolute(350, 200);
        } else if (lift_prime == 2) {
          lift_prime = 3;
          lady_brown.move_absolute(1875, 200);
        } else {
          lift_prime = 1;
          lady_brown.set_brake_mode_all(MOTOR_BRAKE_COAST);
          lady_brown.move_absolute(0, 200);
        }
      }
      if (master.get_digital(DIGITAL_DOWN)) {
        intake.move(127);
      } else if (master.get_digital(DIGITAL_R2)) {
        if (
            (racism_against_blue && (color_sensor.get_hue() >= 200 && color_sensor.get_hue() <= 220)) ||
            (!racism_against_blue && (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20))) {
          intake.move(50);
          pros::delay(300);
          intake.move(-127);
        } else
          intake.move(-127);

      } else
        intake.move(0);

      if (master.get_digital_new_press(DIGITAL_R1))
        mogo.set(!mogo.get());
      if (master.get_digital_new_press(DIGITAL_UP))
        swiper.set(!swiper.get());
      if (master.get_digital_new_press(DIGITAL_B))
        intake_lift.set(!intake_lift.get());
      // set_lift(liftPID.compute((lift.get_angle() / 100.0)));

      // Trigger the selected autonomous routine
      if (master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_A)) {
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }

      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
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