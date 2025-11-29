#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"

pros::MotorGroup left_motors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({4, 5, 6}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&left_motors, 
								&right_motors, 
								11.75, 
								Omniwheel::NEW_325, 
								450, 
								2);

pros::Imu imu (20);
pros::Rotation horizontal_tracker (-9);
pros::Rotation vertical_tracker (-10);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracker, Omniwheel::NEW_325, -2.5);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracker, Omniwheel::NEW_325, 0);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, 
							nullptr, 
							&horizontal_tracking_wheel, 
							nullptr, 
							&imu);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, 
						lateral_controller, 
						angular_controller, 
						sensors);
pros::adi::Pneumatics trapdoor('C', false);
pros::adi::Pneumatics match_load('G', false);
pros::adi::Pneumatics wing_descore('E', false);
pros::Motor intake1(7, pros::MotorGearset::blue);
pros::Motor intake2(8, pros::MotorGearset::blue);


// --- HELPER FUNCTIONS --- //

// void trapdoor_move(bool open_close){
//         trapdoor.set_value(!open_close);
// }
// void match_load_move(bool down_up){
//         match_load.set_value(down_up);
// }
// void wing_descore_move(bool up_down){
//         wing_descore.set_value(up_down);
// }
// void intake1_move(bool intake){
//         if (intake){
//                 stage1_intake_motor.move_velocity(600);
//         }
//         else{
//                 stage1_intake_motor.move_velocity(-600);
//         }
// }
// void intake2_move(bool cycle){
//         if (cycle){
//                 stage2_intake_motor.move_velocity(600);
//         }
//         else{
//                 stage2_intake_motor.move_velocity(-600);
//         }
// }

// void intake1_stop(){
//         stage1_intake_motor.move_velocity(0);
// }
// void intake2_stop(){
//         stage2_intake_motor.move_velocity(0);
// }
// void intake1_move_velocity_percent(int velocity){
//         stage1_intake_motor.move_velocity(velocity*6);
// }
// void intake2_move_velocity_percent(int velocity){
//         stage2_intake_motor.move_velocity(velocity*6);
// }


// 

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
        lcd::initialize();
        chassis.calibrate(); // calibrate sensors
    
        // print position to brain screen
        pros::Task screen_task([&]() {
                while (true) {
                        // print robot location to the brain screen
                        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
                        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
                        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
                        // delay to save resources
                        pros::delay(20);
                }
        });
        // autonomous();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
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
// void opcontrol() {
	// while (true) {
        // get left y and right x positions
        // int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        //                 intake_stg2_move(true);
        //         }
        //         else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        //                 intake_stg2_move(false);
        //         }
        //         else{
        //                 intake_stg2_stop();
        //         }
                
        //         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        //                 intake_stg1_move(true);
        //         }
        //         else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        //                 intake_stg1_move(false);
        //         }
        //         else{
        //                 intake_stg1_stop();
                // }


//                 // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
//                 //         if (!trapdoor_O_F){
//                 //                 trapdoor_move(true);
//                 //                 trapdoor_O_F = true;
//                 //         }
//                 //         else{
//                 //                 trapdoor_move(false);
//                 //                 trapdoor_O_F = false;
//                 //         }
//                 // }
//                 // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
//                 //         if (!loader_O_F){
//                 //                 match_load_move(true);
//                 //                 loader_O_F = true;
//                 //         }
//                 //         else{
//                 //                 match_load_move(false);
//                 //                 loader_O_F = false;
//                 //         }
//                 // }
//                 // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
//                 //         if(!wing_descore_O_F){
//                 //                 wing_descore_move(true);
//                 //                 wing_descore_O_F = true;
//                 //         }
//                 //         else{
//                 //                 wing_descore_move(false);
//                 //                 wing_descore_O_F = false;
//                 //         }
//                 }
		
// 		// move the robot
//         // chassis.curvature(leftY, rightX);

//         // delay to save resources
//         pros::delay(25);
//     }
// }
/*
Ports + intended controllers
123 - left Drive motors
456 - right drive motors
7 - stage 1, 8 - stage 2

G - match load
E - wing
C - Middle goal

9, 10, horizontal and vertical tracking
*/