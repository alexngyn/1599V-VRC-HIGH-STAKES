#include "arm.h"
#include "pros/rtos.hpp"
#include "setup.h" 
#include "opcontrol.h"

double meter_to_in (double meter) { return meter * 39.37008; }

void pidtune() {
    // disable all other settings other than kd and kp
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    //chassis.swingToHeading(45, DriveSide::LEFT, 100000);
    //chassis.turnToHeading(45, 100000);
    // increase kp until the robot oscillates then add kd until it stops
    //chassis.moveToPoint(0, 16, 10000);
    chassis.moveToPose(8, 24,0, 10000);
    //chassisPrintPose();
    chassis.turnToHeading(90, 100000);
    //robot::chassisPrintPose();
}

void spitFirstDonut(){
    intake_motor.move_velocity(600);
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while (optical_sensor.get_proximity() < 240){pros::delay(10);}
    pros::delay(150); //changed 100 -> 110
    intake_motor.brake();
    intake_motor.move_velocity(0);
    pros::delay(500);
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake_motor.move_velocity(600);
}

//======================= awp autons =======================

lemlib::Pose awp0 = {-53,-6, 300};
lemlib::Pose awp1 = {-20, -23 , 300};
lemlib::Pose awpe = {-23, -42, 180};
lemlib::Pose awp2 = {-38, 0, 20};
lemlib::Pose awp3 = {-23, 0, 90};

void soloAWP_R(){
    // pray we score alliance stake
    arm_controller.moveToAngle(90);
    pros::delay(500);
    dt_left.move_relative(1.5, 150);
    dt_right.move_relative(1.5, 150);
    pros::delay(2000);
    arm_controller.moveToAngle(15);
    pros::delay(500);
    dt_left.move(0);
    dt_right.move(0);
    dt_left.move_relative(-1.5, 300);
    dt_right.move_relative(-1.5, 300);

    //set position, open clamp
    chassis.setPose(awp0.x, awp0.y, awp0.theta);
    clamp_solenoid.toggle();
    
    //move to mogo
    chassis.moveToPose(awp1.x, awp1.y, awp1.theta, 2000, {.forwards = false, .maxSpeed = 125}, false);

    //grab mogo
    pros::delay(500);
    clamp_solenoid.toggle();
    
    intake_motor.move_velocity(600);
    pros::delay(200);

    chassis.moveToPoint(awpe.x, awpe.y, 2000, {.forwards = true, .maxSpeed = 125}, false);
    pros::delay(1200); //changed 1000 -> 800

    pros::Task spitFirstDonutTask(spitFirstDonut);

    //move to donut pile
    chassis.moveToPose(awp2.x, awp2.y, awp2.theta, 3000, {.forwards = true, .maxSpeed = 125}, false);

    //eat second donut
    pros::delay(1500);

    intake_motor.move_velocity(600);
    arm_controller.moveToAngle(50);

    //move to ladder
    chassis.moveToPose(awp3.x, awp3.y, awp3.theta, 5000, {.forwards = true, .maxSpeed = 127}, false);
    
}

void soloAWP_L(){
    // pray we score alliance stake
    arm_controller.moveToAngle(90);
    pros::delay(500);
    dt_left.move_relative(1.5, 150);
    dt_right.move_relative(1.5, 150);
    pros::delay(2000);
    arm_controller.moveToAngle(15);
    pros::delay(500);
    dt_left.move(0);
    dt_right.move(0);
    dt_left.move_relative(-1.5, 300);
    dt_right.move_relative(-1.5, 300);

    //set position, open clamp
    chassis.setPose(-1 * awp0.x, awp0.y, -1 * awp0.theta);
    clamp_solenoid.toggle();
    
    //move to mogo
    chassis.moveToPose(-1 * awp1.x, awp1.y, -1 * awp1.theta, 2000, {.forwards = false, .maxSpeed = 125}, false);

    //grab mogo
    pros::delay(500);
    clamp_solenoid.toggle();
    
    intake_motor.move_velocity(600);
    pros::delay(200);

    chassis.moveToPoint(-1 * awpe.x, awpe.y, 2000, {.forwards = true, .maxSpeed = 125}, false);
    pros::delay(1200); //changed 1000 -> 800

    pros::Task spitFirstDonutTask(spitFirstDonut);

    //move to donut pile
    chassis.moveToPose(-1 * awp2.x +10, awp2.y + 5, -1 * awp2.theta, 3000, {.forwards = true, .maxSpeed = 125}, false);

    //eat second donut
    pros::delay(1500);

    intake_motor.move_velocity(600);
    arm_controller.moveToAngle(50);

    //move to ladder
    chassis.moveToPose(-1 * awp3.x, awp3.y, -1 * awp3.theta, 5000, {.forwards = true, .maxSpeed = 127}, false);
    
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
    arm_controller.moveToAngle(12);
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


//======================= old autons =======================

lemlib::Pose old0 = {-48,36, 270};
lemlib::Pose old1 = {-26, 28 , 300};
lemlib::Pose old2 = {-25, 42, 45};
lemlib::Pose old3 = {-6, 30, 135};

void old_R() {
    chassis.setPose(old0.x, old0.y, old0.theta);
    clamp_solenoid.toggle();

    chassis.moveToPose(old1.x, old1.y, old1.theta, 2000, {.forwards = false, .maxSpeed = 200}, false);
    pros::delay(500);
    clamp_solenoid.toggle();
    pros::delay(2000);

    dt_left.move_relative(1, 600);
    dt_right.move_relative(1, 600);
    pros::delay(2000);
    dt_left.move(0);
    dt_right.move(0);


    dt_left.move_relative(-1, 600);
    dt_right.move_relative(-1, 600);

    pros::delay(500);
    dt_left.move(0);
    dt_right.move(0);
    pros::delay(500);


    chassis.moveToPose(old2.x, old2.y, old2.theta, 5000);
    chassis.waitUntil(12);
    intake_motor.move_velocity(600);
    pros::delay(5000);
    intake_motor.move_velocity(0);
}

void old_L() {
    chassis.setPose(-old0.x, old0.y, -old0.theta);
    clamp_solenoid.toggle();

    chassis.moveToPose(-old1.x, old1.y, -old1.theta, 2000, {.forwards = false, .maxSpeed = 200}, false);
    pros::delay(500);
    clamp_solenoid.toggle();
    pros::delay(2000);

    dt_left.move_relative(1, 600);
    dt_right.move_relative(1, 600);
    pros::delay(2000);
    dt_left.move(0);
    dt_right.move(0);


    dt_left.move_relative(-1, 600);
    dt_right.move_relative(-1, 600);

    pros::delay(500);
    dt_left.move(0);
    dt_right.move(0);
    pros::delay(500);


    chassis.moveToPose(-old2.x, old2.y, -old2.theta, 5000);
    chassis.waitUntil(12);
    intake_motor.move_velocity(600);
    pros::delay(5000);
    intake_motor.move_velocity(0);
}