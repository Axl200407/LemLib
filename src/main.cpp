#include "main.h"
#include "lemlib/api.hpp"
#include "pros/llemu.hpp"

//https://lemlib.github.io/LemLib/html/md_docs_tutorials_3_tuning_and_moving.html

//https://lemlib.github.io/Path-Gen/

pros::Controller master (pros::E_CONTROLLER_MASTER);

pros::Motor left_front_motor(6, pros::E_MOTOR_GEARSET_06, false); // port 6, blue gearbox, reversed
pros::Motor left_middle_motor(7, pros::E_MOTOR_GEARSET_06, true); // port 7, blue gearbox, not reversed
pros::Motor left_back_motor(8, pros::E_MOTOR_GEARSET_06, false); // port 8, blue gearbox, reversed
pros::Motor right_front_motor(2, pros::E_MOTOR_GEARSET_06, true); // port 2, blue gearbox, not reversed
pros::Motor right_middle_motor(3, pros::E_MOTOR_GEARSET_06, false); // port 3, blue gearbox, reversed
pros::Motor right_back_motor(4, pros::E_MOTOR_GEARSET_06, true); // port 4, blue gearbox, not reversed


pros::MotorGroup left_side_motors({left_front_motor, left_middle_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_middle_motor, right_back_motor});

pros::Rotation rotation_right(1, true); // port 1, not reversed
pros::Rotation rotation_left(5, true); // port 5, not reversed
pros::Rotation rotation_back(9, false); // port 9, not reversed

pros::Imu inertial_sensor(10); // port 9

lemlib::TrackingWheel left_tracking_wheel(&rotation_left, 2.75, -9.125); //
lemlib::TrackingWheel right_tracking_wheel(&rotation_right, 2.75, 9.125); //
lemlib::TrackingWheel back_tracking_wheel(&rotation_back, 2.75, 3.5); //

lemlib::Drivetrain_t drivetrain {
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    12.5, // track width
    3.25, // wheel diameter
    465 // wheel rpm
};

lemlib::OdomSensors_t sensors {
    &left_tracking_wheel, // vertical tracking wheel 1
    &right_tracking_wheel, // vertical tracking wheel 2
    &back_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &inertial_sensor // inertial sensor
};

//driving PID
lemlib::ChassisController_t lateralController {
    30, // kP
    10, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController {
    2, // kP
    10, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void screen() {
    // loop forever
    while (true) {
        //chassis.setPose(0, 0, 0);
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}
 
void initialize() {
    chassis.calibrate(); // calibrate the chassis
    pros::Task screenTask(screen); // create a task to print the position to the screen
    chassis.setPose(0, 0, 0);
    printf("Calibrated");
    pros::lcd::initialize();
    //chassis.moveTo(3,3,5000,100);
}

/**
 * Disabled runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
    //printf("Begining autonomous");
    chassis.setPose(0, 0, 0);
    chassis.moveTo(0,10,5000,100);
    
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
    chassis.setPose(0, 0, 0);
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
        right_front_motor.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X));
        right_middle_motor.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X));
        right_back_motor.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X));
        left_front_motor.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X));
        left_middle_motor.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X));
        left_back_motor.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X));

		pros::delay(10);
	}
}
