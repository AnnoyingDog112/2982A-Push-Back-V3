#include "main.h"



ASSET(A2982A_Skills_Auto_V3_txt);

MotorGroup left_motors({-1, -2, -3}, MotorGearset::blue); // left motors on ports 1, 2, 3, but reversed
MotorGroup right_motors({4, 5, 6}, MotorGearset::blue); // right motors on ports 4, 5, 6

Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.75, // 10 inch track width
                              Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
Imu imu(20);

// Tracking wheels //

// Sensors for tracking wheels
Rotation horizontal_tracking_sensor(-9);

Rotation vertical_tracking_sensor(-10);


// Setup tracking wheels
TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, 
						Omniwheel::NEW_325, 
						-2.5
);

TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor, 
        Omniwheel::NEW_325, 
             0
);

// --- ODOMETRY --- //

// Setup odom
OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// --- PID --- //

// lateral PID controller
ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              9, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              00, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              00, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0.0, // integral gain (kI)
                                              14, // derivative gain (kD)
                                              1.5, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
ExpoDriveCurve throttle_curve(8, // joystick deadband out of 127
                                     13, // minimum output where drivetrain will move out of 127
                                     1.02 // expo curve gain
);

// input curve for steer input during driver control
ExpoDriveCurve steer_curve(8, // joystick deadband out of 127
                                  18, // minimum output where drivetrain will move out of 127
                                  1.02 // expo curve gain
);

Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&throttle_curve, // throttle input curve
						&steer_curve // steer input curve
);

Controller controller(pros::E_CONTROLLER_MASTER);

Motor intake1(7, MotorGearset::blue); // stage 1 intake motor on port 7
Motor intake2(8, MotorGearset::blue); // stage 2 intake motor on port 8

pros::adi::Pneumatics trapdoor('C', true);
pros::adi::Pneumatics match_load('G', true);
pros::adi::Pneumatics wing_descore('E', true);

// --- HELPER FUNCTIONS --- //

void trapdoor_move(bool open_close){
        trapdoor.set_value(!open_close);
}
void match_load_move(bool down_up){
        match_load.set_value(down_up);
}
void wing_descore_move(bool up_down){
        wing_descore.set_value(up_down);
}
void intake1_move(bool intake){
        if (intake){
                intake1.move_velocity(600);
        }
        else{
                intake1.move_velocity(-600);
        }
}
void intake2_move(bool cycle){
        if (cycle){
                intake2.move_velocity(600);
        }
        else{
                intake2.move_velocity(-600);
        }
}
void intake1_stop(){
        intake1.move_velocity(0);
}
void intake2_stop(){
        intake2.move_velocity(0);
}
void intake1_move_velocity_percent(int velocity){
        intake1.move_velocity(velocity*6);
}
void intake2_move_velocity_percent(int velocity){
        intake2.move_velocity(velocity*6);
}

int loop_delay_ms = 20;
bool intake_stg3_O_F = false;
bool trapdoor_O_F = false;
bool loader_O_F = false;
bool intake_stg2_O_F = false;
bool wing_descore_O_F = false;


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
        autonomous();
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
        chassis.setPose(0, 0, 0); // set starting pose
        pros::delay(100); // small delay to ensure pose is set
        chassis.moveToPoint(0, 24, 10000); // move to (0, 24) inches
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
	while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
                        intake2_move(true);
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
                        intake2_move(false);
                }
                else{
                        intake2_stop();
                }
                
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
                        intake1_move(true);
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
                        intake1_move(false);
                }
                else{
                        intake1_stop();
                }


                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
                        if (!trapdoor_O_F){
                                trapdoor_move(true);
                                trapdoor_O_F = true;
                        }
                        else{
                                trapdoor_move(false);
                                trapdoor_O_F = false;
                        }
                }
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
                        if (!loader_O_F){
                                match_load_move(true);
                                loader_O_F = true;
                        }
                        else{
                                match_load_move(false);
                                loader_O_F = false;
                        }
                }
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
                        if(!wing_descore_O_F){
                                wing_descore_move(true);
                                wing_descore_O_F = true;
                        }
                        else{
                                wing_descore_move(false);
                                wing_descore_O_F = false;
                        }
                }
		
		// move the robot
        chassis.curvature(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}
/*
Ports + intended controllers
123 - left Drive motors
456 - right drive motors
7 - stage 1, 8 - stage 2

G - match load
E - wing
C - Middle goal

Maybe port A as horizontal tracker
9, 10, horizontal and vertical tracking
*/