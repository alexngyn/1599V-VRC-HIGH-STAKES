/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "autonomous.h"
#include "pros/motors.h"
#include "setup.h"

void check_device_plugged_in(int port, std::string deviceName)
{
	if (pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::none ||
		pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::undefined)
	{
		master.rumble("---");
	}
}

void initialize() {
    // partner.clear();
    //partner.print(0, 0, "init start"); // 0-2 0-14
    chassis.calibrate(); // calibrate the chassis
    arm_rotational_sensor.reset(); // reset the arm sensor

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
    pros::lcd::initialize(); // initialize brain screen
    
    pros::Task positionprint([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose(); // get chassis position
            partner.print(0, 0, "%.1f %.1f %.1f", pose.x, pose.y, pose.theta); // 0-2 0-14
            pros::delay(50);
            partner.print(1, 0, "%.1f %s", arm_controller.getAngle(), arm_controller.isInPosition() ? "stopped" : " moving"); // 0-2 0-14
            pros::delay(50);
            partner.print(2, 0, "%s %s ej:%s", sideColor == red ? "red" : "blue", ejectEnabled ? "on" : "off"); // 0-2 0-14
            pros::delay(50);
            pros::lcd::print(3,"opotical distance: %i", optical_sensor.get_proximity());
            pros::delay(500);
        }
    });
    
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

    pros::Task selection([&]() {
        while (pros::competition::is_disabled()) {
            if (selector.get_value() < 100) {
                sideColor = blue;
                indicator.set_value(true);
            } else {
                sideColor = red;
                indicator.set_value(false);
            }
            pros::delay(100);
        }
    });

    pros::lcd::print(0, "%s %s auton", sideColor == red ? "red" : "blue"); // 0-2 0-14

    pros::delay(500);

    //lemlib::Pose testpose = chassis.getPose();
    // if (testpose.theta != testpose.theta) {
    //     console.println("Initializing failed...");
    //     rd_view_alert(getSelectorView(selector), "Initializing failed...");
    // } else {
    //     console.println("Initializing successful...");
    // }

    //pros::Task update_ui_thread(update_ui);
    //pros::Task lights([&] { led1.rotate(); pros::delay(100); });
    //imageTest();

    //partner.print(0, 0, "init done"); // 0-2 0-14
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    
    // partner.print(0, 0, "auton start"); // 0-2 0-14
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    //skills();

    if (sideColor == red){
        //soloAWP_right_red();
        //soloAWP_left_red();
        //elims_red();
    } else {
        //soloAWP_right_blue();
        //soloAWP_left_blue();
        //elims_blue();
    }

}

void printDistance(){
    while (true){
        pros::lcd::print(0,"distance: %i", optical_sensor.get_proximity());
        pros::delay(500);
    }
}

void opcontrol() {
    master.clear();
    // console.println("Running op...");
    partner.print(0, 0, "op start"); // 0-2 0-14
    arm_controller.moveToAngle(16);
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	pros::Task drive_thread(drive);
	pros::Task intake_thread(intake);
    pros::Task clamp_thread(clamp);
    pros::Task topmech_thread(topmech);
    pros::Task doinker_thread(doinker);
    ledsetup();
}