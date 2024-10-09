/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "setup.h"

void check_device_plugged_in(int port, std::string deviceName)
{
	if (pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::none ||
		pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::undefined)
	{
		master.rumble("---");
		console.println((deviceName + " not plugged in!").c_str());
	}
}

void initialize() {
    console.println("Initializing robot...");
    led led1(LED_1_PORT, LED_1_LENGTH); 
    chassis.calibrate(); // calibrate the chassis
    //.reset_position();

    console.println("Checking device status...");
	check_device_plugged_in(inertial_sensor.get_port(), "IMU");
	check_device_plugged_in(dt_left.get_port(0), "Left Motor Group 1");
	check_device_plugged_in(dt_left.get_port(1), "Left Motor Group 2");
	check_device_plugged_in(dt_left.get_port(2), "Left Motor Group 3");
	check_device_plugged_in(dt_right.get_port(0), "Right Motor Group 1");
	check_device_plugged_in(dt_right.get_port(1), "Right Motor Group 2");
	check_device_plugged_in(dt_right.get_port(2), "Right Motor Group 3");
    check_device_plugged_in(intake_motor.get_port(), "Intake Motor");
	check_device_plugged_in(arm_motor.get_port(), "Arm Motor");
	//check_device_plugged_in(horizontal_encoder.get_port(), "Horizontal Encoder");

    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    //pros::lcd::initialize(); // initialize brain screen

    /*
    pros::Task screen([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose(); // get chassis position
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);
            pros::delay(50);
        }
    });
    */
    pros::delay(1000);

    lemlib::Pose testpose = chassis.getPose();
    if (testpose.theta != testpose.theta) {
        console.println("Initializing failed...");
        rd_view_alert(getSelectorView(selector), "Initializing failed...");
    } else {
        console.println("Initializing successful...");
    }

    pros::Task update_ui_thread(update_ui);
    //pros::Task lights([&] { led1.rotate(); pros::delay(100); });

}

void disabled() {}
void competition_initialize() {selector.focus();}

void autonomous() {
    console.println("Running auton...");

    //pros::Task logger_thread(loginator);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    selector.run_auton();
}

void opcontrol() {
    img.focus();

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	pros::Task drive_thread(drive);
	pros::Task intake_thread(intake);
    
}
