#include "main.h" 

void initialize() {
    led led1(LED_1_PORT, LED_1_LENGTH); 
    chassis.calibrate(); // calibrate the chassis
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    pros::lcd::initialize(); // initialize brain screen
    pros::Task screen([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose(); // get chassis position
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);
            pros::delay(50);
        }
    });
    pros::delay(1000);
    //lemlib::Pose testpose = chassis.getPose();
    //if (testpose.theta != testpose.theta) {VIBRATE;};
	printf("init");
}

//pros::Task lights([&] { led1.rotate(); pros::delay(100); });

void disabled() {}
void competition_initialize() {}

void autonomous() {
    //pros::Task logger_thread(loginator);

    dt_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    dt_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    
	//auton_red_non_rush();
    auton_blue_non_rush();
}

void opcontrol() {
    dt_left.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    dt_right.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	pros::Task drive_thread(drive);
	pros::Task intake_thread(intake);
    pros::Task clamp_thread(clamp);
}
