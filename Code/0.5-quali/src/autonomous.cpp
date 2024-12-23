#include "autonomous.h"
#include "arm.h"
#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"
#include "setup.h" 
#include "opcontrol.h"
#include <sys/_intsup.h>

double meter_to_in (double meter) { return meter * 39.37008; }

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

//======================= awp autons =======================

lemlib::Pose awp_0 = {-52.5,-8.5, 300};
lemlib::Pose awp_1 = {-60, -4.5, 300};
lemlib::Pose awp_2 = {-30, -20, 300};
lemlib::Pose awp_3 = {-30, -38, 160};
lemlib::Pose awp_4 = {-44, -5, 330};
lemlib::Pose awp_5 = {-28, -2, 90};

void soloAWP_right_pos(){ // red
    chassis.setPose(awp_0.x, awp_0.y, awp_0.theta);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(awp_1.x, awp_1.y, 1000, {}, false);

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

    intake_motor.move_velocity(600);

    chassis.moveToPose(awp_3.x, awp_3.y, awp_3.theta, 2000, {.forwards = true, .maxSpeed = 100}, false);
    // pros::delay(200); //changed 1000 -> 800

    arm_controller.moveToAngle(18);
    intake_solenoid.toggle();

    // //move to donut pile
    chassis.moveToPose(awp_4.x, awp_4.y, awp_4.theta, 3000, {.forwards = true, .maxSpeed = 100}, false);

    // //eat second donut
    pros::delay(500);

    // //move to ladder
    chassis.moveToPose(awp_5.x, awp_5.y, awp_5.theta, 1000, {.forwards = true, .maxSpeed = 127}, true);
    chassis.waitUntil(5);
    intake_solenoid.toggle();
}

void soloAWP_left_pos(){ // blue
    chassis.setPose(awp_0.x, -awp_0.y, -awp_0.theta - 180);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(awp_1.x, -awp_1.y, 1000);

    arm_controller.moveToAngle(16);
    pros::delay(500);

    //set position, open clamp
    clamp_solenoid.toggle();
    chassis.moveToPoint(awp_0.x, -awp_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
    //move to mogo
    chassis.moveToPose(awp_2.x, -awp_2.y, -awp_2.theta - 180, 2000, {.forwards = false, .maxSpeed = 125}, false);

    //grab mogo
    pros::delay(50);
    clamp_solenoid.toggle();

     pros::delay(200);
    
    intake_motor.move_velocity(600);
    //pros::delay(200);

    chassis.moveToPose(awp_3.x, -awp_3.y, -awp_3.theta-180, 2000, {.forwards = true, .maxSpeed = 100}, false);
    //pros::delay(200); //changed 1000 -> 800

    arm_controller.moveToAngle(18);
    intake_solenoid.toggle();
    //intake_motor.move_velocity(600);

    //move to donut pile
    chassis.moveToPose(awp_4.x, -awp_4.y, -awp_4.theta - 180, 3000, {.forwards = true, .maxSpeed = 127}, false);

    //eat second donut
    pros::delay(500);

    //move to ladder
    chassis.moveToPose(awp_5.x, -awp_5.y, -awp_5.theta - 180, 5000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.waitUntil(5);
    intake_solenoid.toggle();
}

lemlib::Pose awp_neg_0 = {awp_0.x, -awp_0.y, -awp_0.theta - 180};
lemlib::Pose awp_neg_1 = {awp_1.x, -awp_1.y, -awp_1.theta - 180};
lemlib::Pose awp_neg_2 = {awp_2.x, -awp_2.y, -awp_2.theta - 180};
lemlib::Pose awp_neg_3 = {-5, 43, 0};
lemlib::Pose awp_neg_4 = {-5,50, 0};
lemlib::Pose awp_neg_e = {-12,30, 30};
lemlib::Pose awp_neg_5 = {-20, 45, 300};
lemlib::Pose awp_neg_6 = {awp_4.x, -awp_4.y, -awp_4.theta - 180};
lemlib::Pose awp_neg_7 = {awp_5.x, -awp_5.y, -awp_5.theta - 180};

void soloAWP_left_neg(){ // red
    chassis.setPose(awp_neg_0.x, awp_neg_0.y, awp_neg_0.theta);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(awp_neg_1.x, awp_neg_1.y, 1000);

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
    intake_motor.move_velocity(600);

    chassis.swingToHeading(-awp_neg_2.theta-180, DriveSide::RIGHT, 1000, {.minSpeed = 60});

    chassis.moveToPoint(awp_neg_3.x, awp_neg_3.y, 2000, {.forwards = true, .maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 3}, false);
    chassis.moveToPoint(awp_neg_4.x, awp_neg_4.y, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.moveToPoint(awp_neg_e.x, awp_neg_e.y, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 20}, false);
    chassis.moveToPose(awp_neg_5.x, awp_neg_5.y, awp_5.theta, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 20}, false);

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
    chassis.setPose(awp_neg_0.x, -awp_neg_0.y, -awp_neg_0.theta - 180);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};
    
    chassis.moveToPoint(awp_neg_1.x, -awp_neg_1.y, 1000);

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
    intake_motor.move_velocity(600);

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


lemlib::Pose elims_0 = {awp_0.x, awp_0.y, awp_0.theta};
lemlib::Pose elims_1 = {awp_1.x, awp_1.y, awp_1.theta};
lemlib::Pose elims_2 = {-50,-16, 30};
lemlib::Pose elims_3 = {-50,-6, 20};
lemlib::Pose elims_4 = {awp_2.x, awp_2.y, awp_2.theta};
lemlib::Pose elims_5 = {awp_3.x, awp_3.y, awp_3.theta};
lemlib::Pose elims_6 = {-56, -60, 255};
lemlib::Pose elims_7 = {-36, -60, 255};
lemlib::Pose elims_8 = {-5, -50, 60};
lemlib::Pose elims_9 = {-15, -56, 60};

void elims_right () { // red
    chassis.setPose(elims_0.x, elims_0.y, elims_0.theta);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(elims_1.x, elims_1.y, 1000, {}, false);

    arm_controller.moveToAngle(16);
    pros::delay(500);

    //set position, open clamp
    clamp_solenoid.toggle();
    chassis.moveToPoint(elims_2.x, elims_2.y, 1000, {.forwards = false, .maxSpeed = 60});
    intake_solenoid.toggle();
    intake_motor.move_velocity(600);
    //intake_motor.move_relative(1000, 600);
    chassis.swingToHeading(elims_2.theta, DriveSide::RIGHT, 1000, {}, true);

    pros::delay(1500);
    intake_motor.move_velocity(0);

    //move to mogo
    chassis.moveToPose(elims_4.x, elims_4.y, elims_4.theta, 2000, {.forwards = false, .maxSpeed = 100}, false);

    //grab mogo
    pros::delay(50);
    intake_solenoid.toggle();
    clamp_solenoid.toggle();
    arm_controller.moveToAngle(90);
    
    pros::delay(200);

    intake_motor.move_velocity(600);
    chassis.moveToPoint(elims_5.x, elims_5.y, 2000, {.forwards = true, .maxSpeed = 100}, false);

    intake_motor.move_velocity(0);

    // // donink

    chassis.moveToPose(elims_6.x, elims_6.y, elims_6.theta, 2000, {.forwards = true, .maxSpeed = 127}, true);
    doinker_solenoid.toggle();

    //chassis.moveToPoint(-55, -59, 1000);

    chassis.turnToHeading(elims_6.theta+45, 1000);
    pros::delay(500);
    doinker_solenoid.toggle();
    clamp_solenoid.toggle();   
    chassis.turnToHeading(elims_6.theta, 1000);
    
    //intake_motor.move_velocity(600);
    //chassis.moveToPoint(-59, -63, 1000);

    chassis.moveToPoint(elims_7.x, elims_7.y, 2000, {.forwards = false, .maxSpeed = 127}, true);
    arm_controller.moveToAngle(16);
    chassis.turnToHeading(elims_7.theta+180, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});

    chassis.moveToPose(elims_8.x, elims_8.y, elims_8.theta, 2000, {.forwards = true, .maxSpeed = 127}, false);
    doinker_solenoid.toggle();
    chassis.moveToPoint(elims_9.x, elims_9.y, 2000, {.forwards = false, .maxSpeed = 127});
}

void elims_left() { // blue
    chassis.setPose(elims_0.x, -elims_0.y, -elims_0.theta - 180);
    arm_controller.moveToAngle(90);
    while (!arm_controller.isInPosition()) {pros::delay(10);};

    chassis.moveToPoint(elims_1.x, -elims_1.y, 1000, {}, false);

    arm_controller.moveToAngle(16);
     pros::delay(500);

    //set position, open clamp
    clamp_solenoid.toggle();
    chassis.moveToPoint(elims_2.x, -elims_2.y, 1000, {.forwards = false, .minSpeed = 60});
    intake_solenoid.toggle();
    intake_motor.move_velocity(600);
    chassis.swingToHeading(-elims_2.theta - 180, DriveSide::LEFT, 1000, {}, true);

    pros::delay(1500);
    intake_motor.move_velocity(0);
    
    //move to mogo
    chassis.moveToPose(elims_4.x, -elims_4.y, -elims_4.theta - 180, 2000, {.forwards = false, .maxSpeed = 125}, false);

    //grab mogo
    pros::delay(50);
    clamp_solenoid.toggle();
    intake_solenoid.toggle();
    arm_controller.moveToAngle(90);
    
    intake_motor.move_velocity(600);
    //pros::delay(200);
    chassis.moveToPoint(elims_5.x, -elims_5.y, 2000, {.forwards = true, .maxSpeed = 100}, false);
    intake_motor.move_velocity(0);

    // donink

    chassis.moveToPose(elims_6.x, -elims_6.y, -elims_6.theta - 180, 2000, {.forwards = true, .maxSpeed = 127}, true);
    doinker_solenoid.toggle();

    chassis.turnToHeading((-elims_6.theta - 180) - 80, 1000);
    pros::delay(500);
    doinker_solenoid.toggle();
    clamp_solenoid.toggle();
    chassis.turnToHeading(-elims_6.theta - 180, 1000);

    chassis.moveToPose(elims_7.x, -elims_7.y, -elims_7.theta - 180, 2000, {.forwards = true, .maxSpeed = 127}, true);
    arm_controller.moveToAngle(16);
    chassis.turnToHeading(-elims_7.theta, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
    
    chassis.moveToPose(elims_8.x, -elims_8.y, -elims_8.theta - 180, 2000, {.forwards = true, .maxSpeed = 127}, false);
    doinker_solenoid.toggle();
    chassis.moveToPose(elims_9.x, -elims_9.y, -elims_9.theta - 180, 2000, {.forwards = false, .maxSpeed = 127}, false);
}


//======================= skills =======================

void skills () {
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
    intake_motor.move_velocity(600);
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
    intake_motor.move_velocity(0);
    clamp_solenoid.toggle();
    pros::delay(500);
    dt_left.move_relative(1, 300);
    dt_right.move_relative(1, 300);
}