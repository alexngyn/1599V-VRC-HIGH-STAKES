#include "main.h" 
#include "robodash/views/image.hpp"
#include "robodash/views/selector.hpp"

rd::Console console;
rd::Image img("logo");
rd::Selector selector({
   {"Auton 0", &auton_red_non_rush},
   {"Auton 1", &auton_blue_non_rush}
});

void initialize() {
    console.println("Initializing robot...");
    led led1(LED_1_PORT, LED_1_LENGTH); 
    chassis.calibrate(); // calibrate the chassis
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
        rd_view_alert(selector, "Initializing failed...");
    } else {
        console.println("Initializing successful...");
    }
}

//pros::Task lights([&] { led1.rotate(); pros::delay(100); });

void disabled() {}
void competition_initialize() {selector.focus();}

void autonomous() {
    console.println("Running auton...");

    //pros::Task logger_thread(loginator);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    selector.run_auton();

	//auton_red_non_rush();
    //auton_blue_non_rush();
}

void opcontrol() {
    // rd_view_t *view = rd_view_create("custom view"); // create view
    // lv_obj_t *label = lv_label_create(rd_view_obj(view));
    // lv_label_set_text(label, "example");
    // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    img.focus();

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	pros::Task drive_thread(drive);
	pros::Task intake_thread(intake);
    pros::Task clamp_thread(clamp);
}
