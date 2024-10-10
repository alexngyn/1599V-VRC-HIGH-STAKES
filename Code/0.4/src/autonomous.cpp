#include "setup.h" 

// ASSET(path_txt);

/*
float slew(float current, float target, float rate) {
  if (target - current > rate) {
    if (current < target) {
      current += rate;
    } else if (current > target) {
      current -= rate;
    }
    return current;
  } else {
    return target;
  }
}
*/

double meter_to_in (double meter) { return meter * 39.37008; }

/*
void loginator() {
    while (true) {
        //lemlib::telemetrySink()->debug("{},{},{}", 0, (dt_motor_lb.get_actual_velocity() + dt_motor_rb.get_actual_velocity()) / 2, (dt_motor_lb.get_target_velocity() + dt_motor_rb.get_target_velocity()) / 2);
        lemlib::telemetrySink()->debug("{},{}", 1, (dt_left.get_position() + dt_right.get_position()) / 2);
        lemlib::Pose poseTel = chassis.getPose();
        lemlib::telemetrySink()->debug("{},{}", 2, poseTel.theta);
        pros::delay(20 ? CONTROLLER_MODE == bluetooth : 100); 
    }
};
*/

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

void auton_red_non_rush() {
    
}

lemlib::Pose non_rush_start_pose = {-48,36, 270};
lemlib::Pose non_rush_pose_1 = {-26, 28 , 300};
lemlib::Pose non_rush_pose_2 = {-25, 42, 45};
lemlib::Pose non_rush_pose_3 = {-6, 30, 135};

void auton_blue_non_rush() {
    chassis.setPose(non_rush_start_pose.x, non_rush_start_pose.y, non_rush_start_pose.theta);
    clamp_solenoid.toggle();

    chassis.moveToPose(non_rush_pose_1.x, non_rush_pose_1.y, non_rush_pose_1.theta, 2000, {.forwards = false, .maxSpeed = 200}, false);
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


    chassis.moveToPose(non_rush_pose_2.x, non_rush_pose_2.y, non_rush_pose_2.theta, 5000);
    chassis.waitUntil(12);
    intake_motor.move_velocity(600);
    pros::delay(5000);
    intake_motor.move_velocity(0);
    //chassis.swingToPoint(-non_rush_pose_3.x, non_rush_pose_3.y, DriveSide::RIGHT, 5000, {.minSpeed = 48, .earlyExitRange = 20});
    //chassis.moveToPose(-non_rush_pose_3.x, non_rush_pose_3.y, non_rush_pose_3.theta + 180, 5000);
}

void scoreDisrupt() {

}

void skills () {

}

/* 4886s auton

#include "../include/main.h"
#include "stddefs.h"
#include "vex_global.h"
#include "vex_units.h"

void release_antenna(void);

void autonomous(void) {

    //Full AWP quad side
    //unravel
    intake.spinFor(10, ROT_REV, 100, VEL_PCT, false);
    lift.spinFor(350, ROT_DEG, 100, VEL_PCT, true);
    //go for the AS
    turn_pid(-87, -1, 1);
    intake.spinFor(-1, ROT_REV, 100, VEL_PCT, false);
    drive_straight(4, 30, 50);
    lift.spinFor(-200, ROT_DEG, 100, VEL_PCT);
    //go for MOGO
    lift.spinFor(-150, ROT_DEG, 20, VEL_PCT, false);
    drive_straight(-10, 50, 50);
    turn_pid(-8, -1, 1);
    drive_straight(-23, 65, 50, false);
    mogo_clamp.set(1);
    drive_straight(-2, 30, 50);
    wait(250, TIME_MSEC);
    // score the ring in intake
    intake.spinFor(10, ROT_REV, 600, VEL_RPM, false);
    // go for the stack of two
    turn_pid(-90, -1, 1);
    drive_straight(20, 50, 50, false);
    intake.spinFor(150, ROT_REV, 600, VEL_RPM, false);
    drive_straight(3, 25, 50);
    wait(250, TIME_MSEC);
    drive_straight(-2, 30, 50);
    //go for quad stack
    turn_pid(-88, -1, 1);
    drive_straight(13, 18, 50);
    // turn to pickup the second
    drive_straight(-6, 18, 50);
    wait(1000, TIME_MSEC);
    turn_pid(-38, -1, 1);
    drive_straight(9, 15, 50);
    drive_straight(-9, 18, 50);
    wait(200, TIME_MSEC);
    // prep to touch the bar
    lift.spinFor(350, ROT_DEG, 60, VEL_PCT, false);
    // go to touch the bar
    turn_pid(-70, -1, 1);
    turn_pid(20, -1, 1);
    drive_straight(15, 30, 50, false);
    mogo_clamp.set(0);
    drive_straight(5, 30, 50);

    drive_straight(-10, 30, 50);
    turn_pid(-90, -1, 1);
    drive_straight(-4, 30, 50);
    intake.spinFor(3.5, ROT_REV, 66, VEL_PCT);
    drive_straight(5, 30, 50);
    turn_pid(-142, -1, 1);
    drive_straight(-37, 30, 50);
    mogo_clamp.set(1);
    turn_pid(-175, -1, 1);
    drive_straight(18, 30, 50);
    intake.spin(DIR_FWD, 12, VLT_VLT);
    wait(700, TIME_MSEC);
    turn_pid(-45, -1, 1);



   // Skills
   // Unravel and score on AS
    intake.spinFor(8, ROT_REV, 100, VEL_PCT, false);
    lift.spinFor(300, ROT_DEG, 100, VEL_PCT, true);
    // go for MOGO
    drive_straight(11, 30, 50);
    turn_pid(90, -1, 1);
    drive_straight(-24, 60, 50);
    mogo_clamp.set(1);
    // first ring
    intake.spin(DIR_FWD, 12, VLT_VLT);
    turn_pid(-90, -1, 1);
    drive_straight(20, 50, 50);
    // second
    turn_pid(-90, -1, 1);
    drive_straight(21, 50, 50);
    // third
    turn_pid(-60, -1, 1);
    drive_straight(23, 50, 50);
    // fourth
    wait(300, TIME_MSEC);
    drive_straight(-23, 50, 50);
    turn_pid(-30, -1, 1);
    drive_straight(21, 50, 50);
    // fifth
    drive_straight(12, 15, 50);
    // put mogo in corner
    turn_pid(-110, -1, 1);
    mogo_clamp.set(0);
    drive_straight(-10, 20, 50);

    // second MOGO
    drive_straight(27, 40, 50);
    turn_pid(-155, -1, 1);
    drive_straight(-56, 40, 50);
    mogo_clamp.set(1);
     // first ring
    turn_pid(90, -1, 1);
    drive_straight(18, 40, 50);
    // second
    turn_pid(90, -1, 1);
    drive_straight(21, 40, 50);
    // third
    turn_pid(60, -1, 1);
    drive_straight(23, 40, 50);
    // fourth
    wait(300, TIME_MSEC);
    drive_straight(-23, 40, 50);
    turn_pid(30, -1, 1);
    drive_straight(21, 40, 50);
    // fifth
    drive_straight(12, 15, 50);
    // put MOGO in corner
    turn_pid(110, -1, 1);
    mogo_clamp.set(0);
    drive_straight(-10, 20, 50);
    intake.stop();

    // thrid MOGO
    drive_straight(9, 40, 50);
    turn_pid(66, -1, 1);
    drive_straight(64, 40, 50, false);
    // first ring
    intake.spinFor(DIR_FWD, 10, ROT_REV, 100, VEL_PCT, false);
    drive_straight(15, 20, 50);
    wait(1000, TIME_MSEC);
    // MOGO
    turn_pid(122, -1, 1);
    drive_straight(-56, 40, 50);
    mogo_clamp.set(1);
    lift.spinFor(-150, ROT_DEG, 100, VEL_PCT, false);
    // second
    intake.spin(DIR_FWD, 12, VLT_VLT);
    
    turn_pid(10, -1, 1);
    drive_straight(31, 40, 50);
    // third
    turn_pid(90, -1, 1);
    drive_straight(35, 40, 50);
    // fourth
    turn_pid(90, -1, 1);
    drive_straight(35, 40, 50);
    // fifth
    lift.spinFor(250, ROT_DEG, 100, VEL_PCT, false);
    turn_pid(-45, -1, 1);
    drive_straight(35, 40, 50);
    // put MOGO in corner
    turn_pid(-105, -1, 1);
    mogo_clamp.set(0);
    drive_straight(-35, 40, 50);

    drive_straight(14, 30, 50);
    turn_pid(-90, -1, 1);
    intake.spinFor(10, ROT_REV, 100, VEL_PCT, false);
    drive_straight(8, 30, 50);
    drive_straight(-20, 30, 50);

*/