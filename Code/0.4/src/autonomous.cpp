#include "setup.h" 

// ASSET(path_txt);

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

/*
chassis.swingToPoint(53, 53, 4000);
chassis.moveToPose(-60, -44, 250, 4000, {.lead = 0.6, .forwards = true, .maxSpeed = 100}, false);
chassis.swingToHeading(
    90,
    500,
    {.minSpeed = 127, .earlyExitRange = 20}
    // minSpeed 127 means the chassis will move as fast
    // as possible to make this turn
    // the earlyExitRange of 20 means that the chassis
    // will exit once it gets within 20 degrees of the
    // target angle
);
{.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
chassis.moveToPose(
    48,
    -24,
    90,
    2000,
    {.minSpeed=72, .earlyExitRange=8}
    // a minSpeed of 72 means that the chassis will slow down as
    // it approaches the target point, but it won't come to a full stop
    // an earlyExitRange of 8 means the movement will exit 8" away from
    // the target point
);
*/

void pidtune() {
    // disable all other settings other than kd and kp
    // set position to x:0, y:0, heading:0
    //chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    //chassis.swingToHeading(45, DriveSide::LEFT, 100000);
    //chassis.turnToHeading(45, 100000);
    // increase kp until the robot oscillates then add kd until it stops
    //chassis.moveToPoint(0, 16, 10000);
    chassis.moveToPose(8, 16,0, 10000);
}

/*

lemlib::Pose rush_start_pose = {-48,-60, 270};
lemlib::Pose rush_pose_1 = {-18, -60, 270};
lemlib::Pose rush_pose_2 = {0, -50, 240};
lemlib::Pose rush_pose_3 = {-18, -46, 270};
lemlib::Pose rush_pose_4 = {-10, -25, 30};


void auton_red_rush() {
    chassis.setPose(rush_start_pose);
    chassis.moveToPose(rush_pose_1.x, rush_pose_1.y, rush_pose_1.theta, 5000, {.forwards = false});
    chassis.moveToPose(rush_pose_2.x, rush_pose_2.y, rush_pose_2.theta, 5000, {.forwards = false, .maxSpeed = 64});
    //chassis.swingToPoint(rush_pose_3.x, rush_pose_3.y, DriveSide::RIGHT, 5000, {.earlyExitRange = 10});
    //chassis.moveToPoint(rush_pose_3.x, rush_pose_3.y, 5000);
    ///chassis.swingToPoint(rush_pose_4.x, rush_pose_4.y, DriveSide::RIGHT, 5000, {.minSpeed = 48, .earlyExitRange = 20});
    //chassis.moveToPose(rush_pose_4.x, rush_pose_4.y, rush_pose_4.theta, 5000);
}


void auton_blue_rush() {
    chassis.setPose(-rush_start_pose.x, rush_start_pose.y, rush_start_pose.theta + 180);
    chassis.moveToPose(-rush_pose_1.x, rush_pose_1.y, rush_pose_1.theta + 180, 5000, {.forwards = false});
    chassis.moveToPose(-rush_pose_2.x, rush_pose_2.y, rush_pose_2.theta + 180, 5000, {.forwards = false, .maxSpeed = 64});
    chassis.swingToPoint(-rush_pose_3.x, rush_pose_3.y, DriveSide::LEFT, 5000, {.earlyExitRange = 10});
    chassis.moveToPoint(-rush_pose_3.x, rush_pose_3.y, 5000);
    chassis.swingToPoint(-rush_pose_4.x, rush_pose_4.y, DriveSide::LEFT, 5000, {.minSpeed = 48, .earlyExitRange = 20});
    chassis.moveToPose(-rush_pose_4.x, rush_pose_4.y, rush_pose_4.theta + 180, 5000);
}

*/

lemlib::Pose non_rush_start_pose = {-48,36, 270};
lemlib::Pose non_rush_pose_1 = {-26, 28 , 300};
lemlib::Pose non_rush_pose_2 = {-25, 42, 45};
lemlib::Pose non_rush_pose_3 = {-6, 30, 135};

void auton_red_non_rush() {
    chassis.setPose(non_rush_start_pose);
    clamp_solenoid.toggle();
    chassis.moveToPose(non_rush_pose_1.x, non_rush_pose_1.y, non_rush_pose_1.theta, 2000, {.forwards = false, .maxSpeed = 64}, false);
    //chassis.moveToPoint(-24, 25, 5000, {.forwards = false, .maxSpeed = 24});
    pros::delay(500);
    clamp_solenoid.toggle();
    pros::delay(1000);
    chassis.moveToPose(non_rush_pose_2.x, non_rush_pose_2.y, non_rush_pose_2.theta, 5000);
    chassis.waitUntil(12);
    intake_motor.move_velocity(600);
    pros::delay(5000);
    intake_motor.move_velocity(0);
    //chassis.swingToPoint(non_rush_pose_3.x, non_rush_pose_3.y, DriveSide::RIGHT, 5000, {.minSpeed = 48, .earlyExitRange = 20});
    //chassis.moveToPose(non_rush_pose_3.x, non_rush_pose_3.y, non_rush_pose_3.theta, 5000);
}

void auton_blue_non_rush() {
    chassis.setPose(-non_rush_start_pose.x, non_rush_start_pose.y, -non_rush_start_pose.theta);
    clamp_solenoid.toggle();

    chassis.moveToPose(-non_rush_pose_1.x, non_rush_pose_1.y, -non_rush_pose_1.theta, 2000, {.forwards = false, .maxSpeed = 200}, false);
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


    chassis.moveToPose(-non_rush_pose_2.x, non_rush_pose_2.y, -non_rush_pose_2.theta, 5000);
    chassis.waitUntil(12);
    intake_motor.move_velocity(600);
    pros::delay(5000);
    intake_motor.move_velocity(0);
    //chassis.swingToPoint(-non_rush_pose_3.x, non_rush_pose_3.y, DriveSide::RIGHT, 5000, {.minSpeed = 48, .earlyExitRange = 20});
    //chassis.moveToPose(-non_rush_pose_3.x, non_rush_pose_3.y, non_rush_pose_3.theta + 180, 5000);
}

void scoreDisrupt() {
    float s = 1;
    robot::chassisSetPose(-56, 56, 90);
    intake.move(127);
    activeChassis->moveToPose(-20, 62, 90, 1000, {.minSpeed=127, .earlyExitRange=10});
    activeChassis->waitUntilDone();
    activeChassis->moveToPose(-4, 51, 135, 1000);
    pros::delay(500);
    robot::chassisGrabMogo({-14, 14, 75}, 1000);
    pros::delay(1000);
    robot::chassisGrabRing({-24, 48, 0}, 1000, {.minSpeed=70});
    pros::delay(500);
    activeChassis->turnToPoint(-70, 70, 1000);
    mogoMech.extend();
    robot::chassisMove(50, 0, 750);
};