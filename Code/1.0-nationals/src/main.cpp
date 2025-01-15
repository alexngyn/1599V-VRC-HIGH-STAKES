/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "autonomous.h"
#include "setup.h"

void printTelemetry() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(TEXT_MEDIUM, 1, "x: %.1f", pose.x); // prints the x position
        pros::screen::print(TEXT_MEDIUM, 2, "y: %.1f", pose.y); // prints the y position
        pros::screen::print(TEXT_MEDIUM, 3, "theta: %.1f", pose.theta); // prints the heading
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Arm position: %d", arm_controller.getAngle()); // prints the arm position

        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "left temp: %.1f %.1f %.1f", dt_left.get_temperature(0),
                            dt_left.get_temperature(1), dt_left.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "right temp: %.1f %.1f %.1f", dt_right.get_temperature(0),
                            dt_right.get_temperature(1), dt_right.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "intake temp: %.1f", intake_motor.get_temperature());

        std::cout << pose.x << " " << pose.y << " " << inertial_sensor.get_rotation() << pose.theta << std::endl;
        switch (intake_controller.getState()) {
            case Intake::SortState::BLUE: master.print(1, 1, "%s", "BLUE"); break;
            case Intake::SortState::RED: master.print(1, 1, "%s", "RED"); break;
            case Intake::SortState::OFF: master.print(1, 1, "%s", "OFF"); break;
            default: break;
        }

        pros::delay(200);
    }
}

void initialize() {

    // partner.clear();
    //partner.print(0, 0, "init start"); // 0-2 0-14
    chassis.calibrate(); // calibrate the chassis
    arm_rotational_sensor.reset(); // reset the arm sensor

    for (int port : {dt_left.get_port(0), dt_left.get_port(1), dt_left.get_port(2), 
                     dt_right.get_port(0), dt_right.get_port(1), dt_right.get_port(2), 
                     intake_motor.get_port(), arm_motors.get_port(0), arm_motors.get_port(1)}) {
        if (pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::none || pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::undefined) { 
            master.rumble("---"); 
        }
    }
    
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    pros::lcd::initialize(); // initialize brain screen
    
    pros::Task positionprint(printTelemetry);
    pros::lcd::initialize(); // initialize brain screen

    pros::Task selection([&]() {
        while (pros::competition::is_disabled()) {
            if (selector.get_value() < 100) {
                sideColor = color::blue;
                indicator.set_value(true);
            } else {
                sideColor = color::red;
                indicator.set_value(false);
            }
            pros::delay(100);
        }
        switch (sideColor) {
            case color::blue: intake_controller.setState(Intake::SortState::BLUE); break;
            case color::red: intake_controller.setState(Intake::SortState::RED); break;
            default: intake_controller.setState(Intake::SortState::OFF); break;
        }
    });

    pros::lcd::print(0, "%s %s auton", sideColor == color::red ? "red" : "blue"); // 0-2 0-14

    optical_sensor.set_integration_time(20);
    optical_sensor.set_led_pwm(100);

    pros::delay(500);
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    
    // partner.print(0, 0, "auton start"); // 0-2 0-14
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    //arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // pidtune();

    // skills();

    // quali
    // if (sideColor == red){
    //     // soloAWP_right_pos(); //pos
    //     soloAWP_left_pos(); //neg
    // } else if (sideColor == blue){
    //     // soloAWP_left_pos(); // pos
    //     soloAWP_right_pos(); // neg
    // }

    // elims
    //     // soloAWP_right_pos(); //pos
    //     soloAWP_left_pos(); //neg
    // } else if (sideColor == blue){
    //     // soloAWP_left_pos(); // pos
    //     soloAWP_right_pos(); // neg
    // }
}

void opcontrol() {
    master.clear();
    // console.println("Running op...");
    //partner.print(0, 0, "op start"); // 0-2 0-14
    //arm_controller.moveToAngle(16);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	pros::Task drive_thread(drive);
	pros::Task intake_thread(intake);
    pros::Task topmech_thread(topmech);
    pros::Task piston_thread(piston);
    ledsetup();
}