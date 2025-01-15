#include "autonomous.h"
#include "main.h"
#include "arm.h"
#include "pros/rtos.hpp"
#include "setup.h" 
#include "opcontrol.h"

//======================= pid tuning =======================

void pidtune() {
    // disable all other settings other than kd and kp
    // set position to x:0, y:0, heading:0
    chassis.setPose(-48, 0, 90);
    // turn to face heading 90 with a very long timeout
    //chassis.swingToHeading(45, DriveSide::LEFT, 100000);
    //chassis.turnToHeading(45, 100000);
    // increase kp until the robot oscillates then add kd until it stops
    //chassis.moveToPoint(0, 24, 10000);
    // chassis.moveToPoint(-48, 0, 10000);
    // chassis.turnToHeading(270, 100000);
    // chassis.moveToPoint(-48, 0, 10000);

    pros::delay(5000);

    //chassisPrintPose();
    //chassis.turnToHeading(95, 100000);
    //robot::chassisPrintPose();
}

//======================= quali neg 10p auton =======================

lemlib::Pose qual_neg_0 = {-52.5,-8.5, 300};
lemlib::Pose qual_neg_1 = {-60, -4.5, 300};
lemlib::Pose qual_neg_2 = {-26, -20, 300};
lemlib::Pose qual_neg_3 = {-30, -38, 160};
lemlib::Pose qual_neg_4 = {-42, -5, 330};
lemlib::Pose qual_neg_5 = {-28, -2, 90};

//======================= quali pos autons =======================

lemlib::Pose qual_pos_blue_0 = {-52.5,-8.5, 300};
lemlib::Pose qual_pos_blue_1 = {-60, -4.5, 300};
lemlib::Pose qual_pos_blue_2 = {-26, -20, 300};
lemlib::Pose qual_pos_blue_3 = {-30, -38, 160};
lemlib::Pose qual_pos_blue_4 = {-42, -5, 330};
lemlib::Pose qual_pos_blue_5 = {-28, -2, 90};

lemlib::Pose qual_pos_red_0 = {-52.5,-8.5, 300};
lemlib::Pose qual_pos_red_1 = {-60, -4.5, 300};
lemlib::Pose qual_pos_red_2 = {-26, -20, 300};
lemlib::Pose qual_pos_red_3 = qual_pos_blue_3;
lemlib::Pose qual_pos_red_4 = qual_pos_blue_4;
lemlib::Pose qual_pos_red_5 = qual_pos_blue_5;

//======================= elims autons =======================

lemlib::Pose elims_pos_blue_0 = qual_pos_blue_0;
lemlib::Pose elims_pos_blue_1 = qual_pos_blue_1;
lemlib::Pose elims_pos_blue_2 = qual_pos_blue_2;
lemlib::Pose elims_pos_blue_3 = qual_pos_blue_3;
lemlib::Pose elims_pos_blue_4 = qual_pos_blue_4;
lemlib::Pose elims_pos_blue_5 = qual_pos_blue_5;
lemlib::Pose elims_pos_blue_6 = {-26, -20, 300};

lemlib::Pose elims_pos_red_0 = qual_pos_red_0;
lemlib::Pose elims_pos_red_1 = qual_pos_red_1;
lemlib::Pose elims_pos_red_2 = qual_pos_red_2;
lemlib::Pose elims_pos_red_3 = qual_pos_red_3;
lemlib::Pose elims_pos_red_4 = qual_pos_red_4;
lemlib::Pose elims_pos_red_5 = qual_pos_red_5;
lemlib::Pose elims_pos_red_6 = elims_pos_blue_6;

//======================= skills autons =======================

lemlib::Pose awp_0 = {-52.5,-8.5, 300};
lemlib::Pose awp_1 = {-60, -4.5, 300};
lemlib::Pose awp_2 = {-26, -20, 300};
lemlib::Pose awp_3 = {-30, -38, 160};
lemlib::Pose awp_4 = {-42, -5, 330};
lemlib::Pose awp_5 = {-28, -2, 90};

/*======================= old autons =======================

double meter_to_in (double meter) { return meter * 39.37008; }
void autonIntake() { colorSortVision(); pros::delay(50); }

lemlib::Pose awp_0 = {-52.5,-8.5, 300};
lemlib::Pose awp_1 = {-60, -4.5, 300};
lemlib::Pose awp_2 = {-26, -20, 300};
lemlib::Pose awp_3 = {-30, -38, 160};
lemlib::Pose awp_4 = {-42, -5, 330};
lemlib::Pose awp_5 = {-28, -2, 90};

void soloAWP_right_pos(){ // red
    pros::Task autonIntakeThread(autonIntake);
    autonIntakeThread.suspend();

    chassis.setPose(awp_0.x, awp_0.y, awp_0.theta);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(awp_1.x, awp_1.y, 1000, {}, false);
    chassis.waitUntilDone();

    arm_controller.moveToAngle(16);
    pros::delay(500);
    //while (!arm_controller.isInPosition()) {pros::delay(10);};

    //set position, open clamp
    clamp_solenoid.toggle();
    chassis.moveToPoint(awp_0.x, awp_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
    //move to mogo
    chassis.moveToPose(awp_2.x, awp_2.y, awp_2.theta, 2000, {.forwards = false, .maxSpeed = 100}, false);

    // // //grab mogo
    pros::delay(50);
    clamp_solenoid.toggle();
    
    pros::delay(200);

    autonIntakeThread.resume();

    chassis.moveToPose(awp_3.x, awp_3.y, awp_3.theta, 2000, {.forwards = true, .maxSpeed = 100}, false);
    // pros::delay(200); //changed 1000 -> 800

    arm_controller.moveToAngle(18);
    intake_solenoid.toggle();

    // //move to donut pile
    chassis.moveToPose(awp_4.x, awp_4.y, awp_4.theta, 3000, {.forwards = true, .maxSpeed = 64}, false);

    // //eat second donut
    pros::delay(500);

    // //move to ladder
    chassis.moveToPose(awp_5.x, awp_5.y, awp_5.theta, 1000, {.forwards = true, .maxSpeed = 127}, true);
    //chassis.waitUntil(2);
    intake_solenoid.toggle();
    pros::delay(10000);
}

void soloAWP_left_pos(){ // blue
    pros::Task autonIntakeThread(autonIntake);
    autonIntakeThread.suspend();

    chassis.setPose(awp_0.x, -awp_0.y, -awp_0.theta - 180);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(awp_1.x, -awp_1.y, 1000);
    chassis.waitUntilDone();

    arm_controller.moveToAngle(16);
    pros::delay(500);

    //set position, open clamp
    clamp_solenoid.toggle();
    chassis.moveToPoint(awp_0.x, -awp_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
    //move to mogo
    chassis.moveToPose(awp_2.x, -awp_2.y, -awp_2.theta - 180, 2000, {.forwards = false, .maxSpeed = 100}, false);

    //grab mogo
    pros::delay(50);
    clamp_solenoid.toggle();

    pros::delay(200);
    
    autonIntakeThread.resume();
    //pros::delay(200);

    chassis.moveToPose(awp_3.x, -awp_3.y, -awp_3.theta-180, 2000, {.forwards = true, .maxSpeed = 100}, false);
    //pros::delay(200); //changed 1000 -> 800

    arm_controller.moveToAngle(18);
    intake_solenoid.toggle();
    //autonIntakeThread.resume();

    //move to donut pile
    chassis.moveToPose(awp_4.x, -awp_4.y, -awp_4.theta - 180, 3000, {.forwards = true, .maxSpeed = 100}, false);

    //eat second donut
    pros::delay(500);

    //move to ladder
    chassis.moveToPose(awp_5.x, -awp_5.y, -awp_5.theta - 180, 5000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.waitUntil(5);
    intake_solenoid.toggle();
    pros::delay(10000);
}

lemlib::Pose awp_neg_0 = {awp_0.x, -awp_0.y, -awp_0.theta - 180};
lemlib::Pose awp_neg_1 = {awp_1.x, -awp_1.y, -awp_1.theta - 180};
lemlib::Pose awp_neg_2 = {awp_2.x, -awp_2.y, -awp_2.theta - 180};
lemlib::Pose awp_neg_3 = {-7, 35, 210};
lemlib::Pose awp_neg_4 = {-76, 60, 0};
lemlib::Pose awp_neg_e = {-10,30, 30};
lemlib::Pose awp_neg_5 = {-25, 45, 210};
lemlib::Pose awp_neg_6 = {awp_4.x, -awp_4.y, -awp_4.theta - 180};
lemlib::Pose awp_neg_7 = {awp_5.x, -awp_5.y, -awp_5.theta - 180};

void soloAWP_left_neg(){ // red
    pros::Task autonIntakeThread(autonIntake);
    autonIntakeThread.suspend();

    chassis.setPose(awp_neg_0.x, awp_neg_0.y, awp_neg_0.theta);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(awp_neg_1.x, awp_neg_1.y, 1000);
    chassis.waitUntilDone();

    arm_controller.moveToAngle(16);
    pros::delay(500);

    //set position, open clamp
    clamp_solenoid.toggle();
    chassis.moveToPoint(awp_neg_0.x, awp_neg_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
    //move to mogo
    chassis.moveToPose(awp_neg_2.x, awp_neg_2.y, awp_neg_2.theta, 2000, {.forwards = false, .maxSpeed = 100}, false);

    //grab mogo
    pros::delay(50);
    clamp_solenoid.toggle();
    
    pros::delay(200);
    autonIntakeThread.resume();

    chassis.turnToHeading(-awp_neg_2.theta-180, 1000, {}, false);

    autonIntakeThread.resume();

    chassis.moveToPose(awp_neg_3.x, awp_neg_3.y, awp_neg_3.theta, 1000, {.forwards = true, .maxSpeed = 50, .minSpeed = 20, .earlyExitRange = 3}, false);
    //chassis.moveToPoint(awp_neg_3.x, awp_neg_3.y, 2000, {.forwards = true, .maxSpeed = 50, .minSpeed = 20, .earlyExitRange = 3}, false);
    chassis.moveToPoint(awp_neg_4.x, awp_neg_4.y, 1000, {.forwards = true, .maxSpeed = 50}, false);
    chassis.moveToPoint(awp_neg_e.x, awp_neg_e.y, 1000, {.forwards = false, .maxSpeed = 80}, false);
    chassis.moveToPoint(awp_neg_5.x, awp_neg_5.y, 2000, {.forwards = true, .maxSpeed = 127}, false);

    intake_solenoid.toggle();

    //move to donut pile
    chassis.moveToPose(awp_neg_6.x, awp_neg_6.y, awp_neg_6.theta, 3000, {.forwards = true, .maxSpeed = 127}, false);

    //eat second donut
    pros::delay(500);

    //move to ladder
    chassis.moveToPose(awp_neg_7.x, awp_neg_7.y, awp_neg_7.theta, 1000, {.forwards = true, .maxSpeed = 127}, true);
    chassis.waitUntil(5);
    intake_solenoid.toggle();
}

void soloAWP_right_neg() { // blue
    pros::Task autonIntakeThread(autonIntake);
    autonIntakeThread.suspend();

    chassis.setPose(awp_neg_0.x, -awp_neg_0.y, -awp_neg_0.theta - 180);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};
    
    chassis.moveToPoint(awp_neg_1.x, -awp_neg_1.y, 1000);
    chassis.waitUntilDone();

    arm_controller.moveToAngle(16);
    pros::delay(500);

    //set position, open clamp
    clamp_solenoid.toggle();
    chassis.moveToPoint(awp_neg_0.x, -awp_neg_0.y, 1000, {.forwards = false, .minSpeed = 60});

    //move to mogo
    chassis.moveToPose(awp_neg_2.x, -awp_neg_2.y, -awp_neg_2.theta - 180, 2000, {.forwards = false, .maxSpeed = 125}, false);

    //grab mogo
    pros::delay(50);
    clamp_solenoid.toggle();

    pros::delay(200);
    autonIntakeThread.resume();

    chassis.swingToHeading(-awp_neg_2.theta - 180, DriveSide::LEFT, 1000, {.minSpeed = 60});

    chassis.moveToPoint(awp_neg_3.x, -awp_neg_3.y, 2000, {.forwards = true, .maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 3}, false);
    chassis.moveToPoint(awp_neg_4.x, -awp_neg_4.y, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.moveToPoint(awp_neg_e.x, -awp_neg_e.y, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 20}, false);
    chassis.moveToPose(awp_neg_5.x, -awp_neg_5.y, -awp_neg_5.theta - 180, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 20}, false);

    intake_solenoid.toggle();

    //move to donut pile
    chassis.moveToPose(awp_neg_6.x, -awp_neg_6.y, -awp_neg_6.theta - 180, 3000, {.forwards = true, .maxSpeed = 127}, false);

    //eat second donut
    pros::delay(500);

    //move to ladder
    chassis.moveToPose(awp_neg_7.x, -awp_neg_7.y, -awp_neg_7.theta - 180, 1000, {.forwards = true, .maxSpeed = 127}, true);
    chassis.waitUntil(5);
    intake_solenoid.toggle();

}

//======================= elims =======================

lemlib::Pose mid_mogo = {-6, -47.5, 90};

//middle donuts
lemlib::Pose elims_1 = {-6.718, -42.645, 0};
lemlib::Pose elims_2 = {-7.159, 52.352, 0};
lemlib::Pose elims_3 = {-22.631, 47.184, 240};

void elims_right () { //red
    pros::Task autonIntakeThread(autonIntake);
    autonIntakeThread.suspend();

    chassis.setPose(awp_0.x, awp_0.y, awp_0.theta);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(awp_1.x, awp_1.y, 1000, {}, false);
    chassis.waitUntilDone();

    arm_controller.moveToAngle(16);
    pros::delay(500);
    //while (!arm_controller.isInPosition()) {pros::delay(10);};

    //set position, open clamp
    clamp_solenoid.toggle();
    chassis.moveToPoint(awp_0.x, awp_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
    //move to mogo
    chassis.moveToPose(awp_2.x, awp_2.y, awp_2.theta, 2000, {.forwards = false, .maxSpeed = 100}, false);

    // // //grab mogo
    pros::delay(50);
    clamp_solenoid.toggle();
    
    pros::delay(200);

    autonIntakeThread.resume();

    chassis.moveToPose(awp_3.x, awp_3.y, awp_3.theta, 2000, {.forwards = true, .maxSpeed = 100}, false);

    arm_controller.moveToAngle(18);
    intake_solenoid.toggle();

    // //move to donut pile
    chassis.moveToPose(awp_4.x, awp_4.y, awp_4.theta, 3000, {.forwards = true, .maxSpeed = 64}, false);

    // //eat second donut
    pros::delay(250);
    autonIntakeThread.suspend();

    //turn 180, drop mogo, turn back
    chassis.turnToHeading(awp_4.theta + 180, 3000,{.maxSpeed = 64});
    chassis.turnToHeading(awp_4.theta, 3000, {.maxSpeed = 64});
    clamp_solenoid.toggle();
    // //move to 3rd mogo
    chassis.moveToPose(mid_mogo.x, mid_mogo.y, mid_mogo.theta, 1000, {.forwards = false, .maxSpeed = 127}, true);
    // detectdonutthread.suspend();
    intake_solenoid.toggle();
    pros::delay(500);
    autonIntakeThread.resume();
}

void elims_left() { // blue
    pros::Task autonIntakeThread(autonIntake);
    autonIntakeThread.suspend();

    chassis.setPose(awp_0.x, awp_0.y, awp_0.theta);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(awp_1.x, awp_1.y, 1000, {}, false);
    chassis.waitUntilDone();

    arm_controller.moveToAngle(16);
    pros::delay(500);
    //while (!arm_controller.isInPosition()) {pros::delay(10);};

    //set position, open clamp
    clamp_solenoid.toggle();
    chassis.moveToPoint(awp_0.x, awp_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
    //move to mogo
    chassis.moveToPose(awp_2.x, awp_2.y, awp_2.theta, 2000, {.forwards = false, .maxSpeed = 100}, false);

    // // //grab mogo
    pros::delay(50);
    clamp_solenoid.toggle();
    
    pros::delay(200);

    autonIntakeThread.resume();

    chassis.moveToPose(awp_3.x, awp_3.y, awp_3.theta, 2000, {.forwards = true, .maxSpeed = 100}, false);
    // pros::delay(200); //changed 1000 -> 800

    arm_controller.moveToAngle(18);
    intake_solenoid.toggle();

    // //move to donut pile
    chassis.moveToPose(awp_4.x, awp_4.y, awp_4.theta, 3000, {.forwards = true, .maxSpeed = 64}, false);

    // //eat second donut
    pros::delay(500);

    //turn 180, drop mogo, turn back
    chassis.turnToHeading(awp_4.theta + 180, 3000);
    clamp_solenoid.toggle();
    chassis.turnToHeading(awp_4.theta, 3000);

    // //move to 3rd mogo
    chassis.moveToPose(mid_mogo.x, mid_mogo.y, mid_mogo.theta, 1000, {.forwards = false, .maxSpeed = 127}, true);
    intake_solenoid.toggle();
    pros::delay(500);
}


//======================= skills =======================

void skills () {
    pros::Task autonIntakeThread(autonIntake);
    autonIntakeThread.suspend();
    
    //setup
    clamp_solenoid.toggle();
    dt_left.move_relative(-0.35, 150);
    dt_right.move_relative(-0.35, 150);
    pros::delay(2000);
    chassis.setPose(-55, 0 ,270);

    //alliance stake
    arm_controller.moveToAngle(90);
    pros::delay(500);
    dt_left.move_relative(1, 150);
    dt_right.move_relative(1, 150);
    pros::delay(2000);
    arm_controller.moveToAngle(16);
    pros::delay(500);
    dt_left.move(0);
    dt_right.move(0);
    dt_left.move_relative(-1, 300);
    dt_right.move_relative(-1, 300);

    chassis.moveToPoint(-53, -20, 2000, {.forwards = false, .maxSpeed = 75}, false);
    pros::delay(1000);
    clamp_solenoid.toggle();
    pros::delay(1000);

    chassis.turnToPoint(-65, -46, 2000);
    autonIntakeThread.resume();
    chassis.moveToPoint(-65, -46, 2000, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(500);
    chassis.moveToPoint(-46, -46, 2000, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(500);
    chassis.moveToPoint(-25.3, -25, 2000, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(500);
    chassis.moveToPoint(-46, -46, 2000, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(500);
    chassis.moveToPoint(-24.5, -47.2, 2000, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(500);
    chassis.moveToPoint(-46, -46, 2000, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(500);
    chassis.moveToPoint(-47, -59, 2000, {.forwards = true, .maxSpeed = 100}, false);
    chassis.turnToPoint(-64, -64, 2000, {.forwards = true});
    chassis.moveToPoint(-64, -64, 2000, {.forwards = false, .maxSpeed = 75}, true);
    pros::delay(2000);
    autonIntakeThread.suspend();
    clamp_solenoid.toggle();
    pros::delay(500);
    chassis.moveToPoint(-55, -55, 2000, {.forwards = false, .maxSpeed = 100}, true);

    chassis.moveToPoint(-47, 17, 3000, {.forwards = false, .maxSpeed = 100}, false);
    chassis.waitUntilDone();
    clamp_solenoid.toggle();
    pros::delay(500);
    autonIntakeThread.resume();
    chassis.turnToPoint(-26, 20, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-26, 20, 2000, {.forwards = true, .maxSpeed = 100}, false);
    chassis.moveToPoint(-22, 45, 2000, {.forwards = true, .maxSpeed = 100}, false);
    chassis.turnToPoint(-44, 47, 2000, {.forwards = true});
    chassis.moveToPoint(-44, 47, 2000, {.forwards = true, .maxSpeed = 100}, false);
    chassis.moveToPoint(-60, 47, 2000, {.forwards = true, .maxSpeed = 100}, false);
    chassis.turnToPoint(-48, 57, 2000, {.forwards = true, .maxSpeed = 100}, false);
    chassis.moveToPoint(-48, 57, 2000, {.forwards = true, .maxSpeed = 100}, false);
    chassis.turnToPoint(-66, 66, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.waitUntilDone();
    clamp_solenoid.toggle();
    pros::delay(500);
    chassis.moveToPoint(-44, 44, 2000, {.forwards = true, .maxSpeed = 127}, false);
    arm_controller.moveToAngle(150);
    chassis.moveToPoint(-10, 10, 10000, {.forwards = true, .maxSpeed = 100}, false);
    chassis.waitUntilDone();
    arm_controller.moveToAngle(10);
}
*/