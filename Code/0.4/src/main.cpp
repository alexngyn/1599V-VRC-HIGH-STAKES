/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "gui.h"
#include "pros/motors.h"
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
    partner.print(0, 0, "current status: init start"); // 0-2 0-14
    led led1(LED_1_PORT, LED_1_LENGTH); 
    chassis.calibrate(); // calibrate the chassis
    arm_rotational_sensor.reset(); // reset the arm sensor
    //arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //.reset_position();

    //console.println("Checking device status...");
	// check_device_plugged_in(inertial_sensor.get_port(), "IMU");
	// check_device_plugged_in(dt_left.get_port(0), "Left Motor Group 1");
	// check_device_plugged_in(dt_left.get_port(1), "Left Motor Group 2");
	// check_device_plugged_in(dt_left.get_port(2), "Left Motor Group 3");
	// check_device_plugged_in(dt_right.get_port(0), "Right Motor Group 1");
	// check_device_plugged_in(dt_right.get_port(1), "Right Motor Group 2");
	// check_device_plugged_in(dt_right.get_port(2), "Right Motor Group 3");
    // check_device_plugged_in(intake_motor.get_port(), "Intake Motor");
	// check_device_plugged_in(arm_motor.get_port(), "Arm Motor");
	//check_device_plugged_in(horizontal_encoder.get_port(), "Horizontal Encoder");

    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    //pros::lcd::initialize(); // initialize brain screen

    
    pros::Task positionprint([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose(); // get chassis position
            partner.print(2, 0, "X: %f Y: %f Theta: %f", pose.x, pose.y, pose.theta); // 0-2 0-14
            pros::delay(50);
        }
    });

    pros::delay(1000);

    lemlib::Pose testpose = chassis.getPose();
    // if (testpose.theta != testpose.theta) {
    //     console.println("Initializing failed...");
    //     rd_view_alert(getSelectorView(selector), "Initializing failed...");
    // } else {
    //     console.println("Initializing successful...");
    // }

    //pros::Task update_ui_thread(update_ui);
    //pros::Task lights([&] { led1.rotate(); pros::delay(100); });
    //imageTest();

    partner.print(0, 0, "current status: init done"); // 0-2 0-14
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    console.println("Running auton...");
    partner.print(0, 0, "current status: auton start"); // 0-2 0-14

    //pros::Task logger_thread(loginator);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    auton_blue_non_rush();
    //selector.run_auton();
    partner.print(0, 0, "current status: auton done"); // 0-2 0-14
}

void opcontrol() {
    console.println("Running op...");
    partner.print(0, 0, "current status: op start"); // 0-2 0-14

    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	pros::Task drive_thread(drive);
	pros::Task intake_thread(intake);
    pros::Task clamp_thread(clamp);
    pros::Task topmech_thread(topmech);
}
