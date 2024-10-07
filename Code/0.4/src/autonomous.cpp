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
    chassisPrintPose();
    chassis.turnToHeading(90, 100000);
    robot::chassisPrintPose();
}

void auton_red_non_rush() {
    
}

void auton_blue_non_rush() {
    
}

void scoreDisrupt() {

}

void skills () {

}