#include "autonomous.h"
#include "main.h"
#include "arm.h"
#include "setup.h" 
#include "opcontrol.h"

lemlib::Pose mirrorPose(lemlib::Pose pose) {
    return {-pose.x, pose.y, -pose.theta};
}

void stakealign() {
    while (distance_sensor.get() > 10) { dt_left.move(30); dt_right.move(30); };
    dt_left.move(0); dt_right.move(0);
}

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

// lemlib::Pose qual_neg_0 = {-52.5,-8.5, 300}; 
// lemlib::Pose qual_neg_1 = {-60, -4.5, 300};
// lemlib::Pose qual_neg_2 = {-26, -20, 300};
// lemlib::Pose qual_neg_3 = {-30, -38, 160};
// lemlib::Pose qual_neg_4 = {-42, -5, 330};
// lemlib::Pose qual_neg_5 = {-28, -2, 90};

// void qual_neg() {
//  // priority: do last
// }

//======================= quali pos 9p autons =======================

lemlib::Pose qual_pos_red_0 = {-50,-65, 80}; 
lemlib::Pose qual_pos_red_1 = {-6, -55, 80};
lemlib::Pose qual_pos_red_2 = {-32, -60, 80};
lemlib::Pose qual_pos_red_3 = {-24, -48, 0};
lemlib::Pose qual_pos_red_4 = {-24, -24, 180};
lemlib::Pose qual_pos_red_5 = {-48, 0, 0};
lemlib::Pose qual_pos_red_6 = {-60, 0, 0}; 
lemlib::Pose qual_pos_red_7 = {-26, -20, 300}; 
lemlib::Pose qual_pos_red_8 = {-26, -20, 300}; 

void qual_pos_red() {
    chassis.setPose(qual_pos_red_0.x, qual_pos_red_0.y, qual_pos_red_0.theta);

    //goal rush
    chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 1000, {}, false); //rush to mogo
    doinker_solenoid.retract();
    chassis.moveToPoint(qual_pos_red_2.x, qual_pos_red_2.y, 1000, {.forwards = false}, true); // move backwards
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL); // get ready to score
    chassis.waitUntil(10);
    doinker_solenoid.extend(); //let go mogo a bit before fully back. move back then score
    chassis.waitUntilDone();
    arm_controller.moveTo(-130,false); //score
    
    //second mogo
    intake_controller.hold(); //wait until ring is in
    //arm_controller.moveTo(Arm::position::RETRACT);
    arm_controller.moveTo(Arm::position::UP); // keep up until we fix intake
    chassis.moveToPoint(qual_pos_red_3.x, qual_pos_red_3.y, 1000, {.forwards = true}, false); //go to ring
    chassis.moveToPoint(qual_pos_red_4.x, qual_pos_red_4.y, 1000, {.forwards = false}, false); //go to mogo
    clamp_solenoid.retract(); //clamp mogo
    intake_controller.set(Intake::IntakeState::INTAKING); //intake ring inside of intake
    //chassis.turnToPoint(qual_pos_red_5.x, qual_pos_red_5.y, 1000, {}, true);
    pros::delay(300);
    intake_controller.set(Intake::IntakeState::STOPPED);

    //alicence stake
    intake_solenoid.retract();
    chassis.moveToPoint(qual_pos_red_5.x, qual_pos_red_5.y, 1000, {.forwards = true}, true); // take ring from stack
    intake_controller.set(Intake::IntakeState::INTAKING); 
    pros::delay(500);
    intake_controller.set(Intake::IntakeState::STOPPED);
    //arm_controller.moveTo(Arm::position::UP);
    chassis.moveToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {.forwards = true}, false);
    stakealign();
    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, false);

    //touch ladder
    chassis.moveToPoint(qual_pos_red_7.x, qual_pos_red_7.y, 1000, {.forwards = false}, true);//go to middle of field
    //arm_controller.moveTo(100);//move arm up along the way
    chassis.moveToPoint(qual_pos_red_8.x, qual_pos_red_8.y, 1000, {.forwards = true}, false);//go to ladder
}

lemlib::Pose qual_pos_blue_0 = mirrorPose({-50,-36, 100}); 
lemlib::Pose qual_pos_blue_1 = mirrorPose({-60, -4.5, 100});
lemlib::Pose qual_pos_blue_2 = mirrorPose(qual_pos_red_2);
lemlib::Pose qual_pos_blue_3 = mirrorPose(qual_pos_red_3);
lemlib::Pose qual_pos_blue_4 = mirrorPose(qual_pos_red_4);
lemlib::Pose qual_pos_blue_5 = mirrorPose(qual_pos_red_5);
lemlib::Pose qual_pos_blue_6 = mirrorPose(qual_pos_red_6);
lemlib::Pose qual_pos_blue_7 = mirrorPose(qual_pos_red_7);
lemlib::Pose qual_pos_blue_8 = mirrorPose(qual_pos_red_8);

void qual_pos_blue() {
    chassis.setPose(qual_pos_blue_0.x, qual_pos_blue_0.y, qual_pos_blue_0.theta);

    //goal rush
    chassis.moveToPoint(qual_pos_blue_1.x, qual_pos_blue_1.y, 1000, {}, true);
    chassis.waitUntil(10);
    doinker_solenoid.retract();
    chassis.moveToPoint(qual_pos_blue_2.x, qual_pos_blue_2.y, 1000, {.forwards = false}, true);
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL);
    chassis.waitUntil(10);
    doinker_solenoid.extend();
    chassis.waitUntilDone();
    arm_controller.moveTo(270);
    pros::delay(500);

    //second mogo
    intake_controller.hold();
    arm_controller.moveTo(Arm::position::RETRACT);
    chassis.moveToPoint(qual_pos_blue_3.x, qual_pos_blue_3.y, 1000, {.forwards = true}, false);
    chassis.moveToPoint(qual_pos_blue_4.x, qual_pos_blue_4.y, 1000, {.forwards = false}, false);
    clamp_solenoid.retract();
    intake_controller.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(qual_pos_blue_5.x, qual_pos_blue_5.y, 1000, {}, true);
    pros::delay(500);
    intake_controller.set(Intake::IntakeState::STOPPED);

    //alicence stake
    chassis.moveToPoint(qual_pos_blue_5.x, qual_pos_blue_5.y, 1000, {.forwards = true}, true);
    intake_solenoid.retract();
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(500);
    intake_controller.set(Intake::IntakeState::STOPPED);
    arm_controller.moveTo(Arm::position::UP);
    chassis.moveToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 1000, {.forwards = true}, false);
    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE);
    pros::delay(500);

    //touch ladder
    chassis.moveToPoint(qual_pos_blue_7.x, qual_pos_blue_7.y, 1000, {.forwards = false}, true);
    arm_controller.moveTo(100);
    chassis.moveToPoint(qual_pos_blue_8.x, qual_pos_blue_8.y, 1000, {.forwards = true}, false);
}


//======================= elims 10p autons =======================

lemlib::Pose elims_pos_red_0 = qual_pos_red_0;
lemlib::Pose elims_pos_red_1 = qual_pos_red_1;
lemlib::Pose elims_pos_red_2 = qual_pos_red_2;
lemlib::Pose elims_pos_red_3 = qual_pos_red_3;
lemlib::Pose elims_pos_red_4 = qual_pos_red_4;
lemlib::Pose elims_pos_red_5 = qual_pos_red_5;
lemlib::Pose elims_pos_red_6 = qual_pos_red_6;
lemlib::Pose elims_pos_red_7 = {-60,0,270};
lemlib::Pose elims_pos_red_8 = {-64,-64,15};

void elims_pos_red() {
    chassis.setPose(elims_pos_red_0.x, elims_pos_red_0.y, elims_pos_red_0.theta);

    //goal rush
    chassis.moveToPoint(elims_pos_red_1.x, elims_pos_red_1.y, 1000, {}, true);
    chassis.waitUntil(10);
    doinker_solenoid.retract();
    chassis.moveToPoint(elims_pos_red_2.x, elims_pos_red_2.y, 1000, {.forwards = false}, true);
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL);
    chassis.waitUntil(10);
    doinker_solenoid.extend();
    chassis.waitUntilDone();
    arm_controller.moveTo(270);
    pros::delay(500);
    
    //second mogo
    intake_controller.hold();
    arm_controller.moveTo(Arm::position::RETRACT);
    chassis.moveToPoint(elims_pos_red_3.x, elims_pos_red_3.y, 1000, {.forwards = true}, false);
    chassis.moveToPoint(elims_pos_red_4.x, elims_pos_red_4.y, 1000, {.forwards = false}, false);
    clamp_solenoid.retract();
    intake_controller.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(elims_pos_red_5.x, elims_pos_red_5.y, 1000, {}, true);
    pros::delay(500);
    intake_controller.set(Intake::IntakeState::STOPPED);

    //alicence stake
    chassis.moveToPoint(elims_pos_red_5.x, elims_pos_red_5.y, 1000, {.forwards = true}, true);
    intake_solenoid.retract();
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(500);
    intake_controller.set(Intake::IntakeState::STOPPED);
    arm_controller.moveTo(Arm::position::UP);
    chassis.moveToPoint(elims_pos_red_6.x, elims_pos_red_6.y, 1000, {.forwards = true}, false);
    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE);
    pros::delay(500);

    //touch ladder
    chassis.moveToPoint(elims_pos_red_7.x, elims_pos_red_7.y, 1000, {.forwards = false}, true);
     chassis.moveToPoint(elims_pos_red_8.x, elims_pos_red_8.y, 1000, {.forwards = true}, false);
}

lemlib::Pose elims_pos_blue_0 = qual_pos_blue_0;
lemlib::Pose elims_pos_blue_1 = qual_pos_blue_1;
lemlib::Pose elims_pos_blue_2 = qual_pos_blue_2;
lemlib::Pose elims_pos_blue_3 = qual_pos_blue_3;
lemlib::Pose elims_pos_blue_4 = qual_pos_blue_4;
lemlib::Pose elims_pos_blue_5 = qual_pos_blue_5;
lemlib::Pose elims_pos_blue_6 = qual_pos_blue_6;
lemlib::Pose elims_pos_blue_7 = qual_pos_blue_7;
lemlib::Pose elims_pos_blue_8 = qual_pos_blue_8;

void elims_pos_blue() {
    chassis.setPose(elims_pos_blue_0.x, elims_pos_blue_0.y, elims_pos_blue_0.theta);

    //goal rush
    chassis.moveToPoint(elims_pos_blue_1.x, elims_pos_blue_1.y, 1000, {}, true);
    chassis.waitUntil(10);
    doinker_solenoid.retract();
    chassis.moveToPoint(elims_pos_blue_2.x, elims_pos_blue_2.y, 1000, {.forwards = false}, true);
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL);
    chassis.waitUntil(10);
    doinker_solenoid.extend();
    chassis.waitUntilDone();
    arm_controller.moveTo(270);
    pros::delay(500);

    //second mogo
    intake_controller.hold();
    arm_controller.moveTo(Arm::position::RETRACT);
}

//======================= skills autons =======================

lemlib::Pose skills_0 = {-60, 0, 300}; 
lemlib::Pose skills_1 = {-46.867, -23.306, 300};
lemlib::Pose skills_2 = {-23.275, -24.193, 300};
lemlib::Pose skills_3 = {-23.859, -47.604, 160};
lemlib::Pose skills_4 = {23.834, -47.629, 330};
lemlib::Pose skills_5 = {0.126, -62.967, 90};
lemlib::Pose skills_6 = {-43.649, -47.292, 300}; 
lemlib::Pose skills_7 = {-55.425, -47.137, 300};
lemlib::Pose skills_8 = {-49.382, -55.813, 300};
lemlib::Pose skills_9 = {-65.665, -68.134, 300};

lemlib::Pose skills_10 = {-1.951, -2.26, 300};
lemlib::Pose skills_11 = {-40.253, 19.44, 300};
lemlib::Pose skills_12 = {-22.508, 27.069, 300};
lemlib::Pose skills_13 = {-20.959, 38.817, 300};
lemlib::Pose skills_14 = {-55.83, 47.264, 300};
lemlib::Pose skills_15 = {-48.847, 56.396, 300};
lemlib::Pose skills_16 = {-65.665, 65.229, 300};

lemlib::Pose skills_17 = {-0.123, 56.442, 300};
lemlib::Pose skills_18 = {21.939, 50.433, 300};
lemlib::Pose skills_19 = {0.64, 52.935, 300};
lemlib::Pose skills_20 = {22.244, 25.35, 300};
lemlib::Pose skills_21 = {40.968, 4.504, 300};
lemlib::Pose skills_22 = {25.727, -16.679, 300};
lemlib::Pose skills_23 = {56.189, -47.238, 300};
lemlib::Pose skills_24 = {45.742, -58.696, 300};
lemlib::Pose skills_25 = {46.8, -0.723, 300};
lemlib::Pose skills_26 = {46.112, 58.022, 300};
lemlib::Pose skills_27 = {65.846, 68.807, 300};

lemlib::Pose skills_28 = {59.421, 47.007, 300};
lemlib::Pose skills_29 = {42.702, 24.236, 300};
lemlib::Pose skills_30 = {58.274, 0.195, 300};
lemlib::Pose skills_31 = {66.246, -65.229, 300};
lemlib::Pose skills_32 = {11.321, -14.207, 300};
//todo: fix moving direction and thetas

void skills() {
    chassis.setPose(skills_0.x, skills_0.y, skills_0.theta);
    
    //allicen stake
    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, false);
    arm_controller.moveTo(Arm::position::RETRACT);

    //goal 1 quadrent 3 + right wall stake

    chassis.moveToPoint(skills_1.x, skills_1.y, 1000, {.forwards=false}, false);
    clamp_solenoid.retract();

    chassis.turnToHeading(60, 1000);
    intake_controller.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(skills_2.x, skills_2.y, skills_2.theta, 1000, {.forwards=true,.minSpeed=48}, false);
    chassis.moveToPoint(skills_3.x, skills_3.y, 1000, {.forwards=true}, false);

    chassis.moveToPoint(skills_4.x, skills_4.y, 1000, {.forwards=true}, false);

    chassis.moveToPose(skills_5.x, skills_5.y, skills_5.theta, 1000, {.forwards=true}, true);
    chassis.waitUntil(5);
    arm_controller.moveTo(Arm::position::INTAKE, false);
    chassis.waitUntilDone();
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL,false);
    arm_controller.moveTo(Arm::position::UP,false);
    arm_controller.moveTo(Arm::position::RETRACT);

    chassis.moveToPose(skills_6.x , skills_6.y, skills_6.theta, 1000, {.forwards=true,.minSpeed=48}, false);
    chassis.moveToPoint(skills_7.x, skills_7.y, 1000, {.forwards=true}, false);
    chassis.moveToPoint(skills_8.x, skills_8.y, 1000, {.forwards=true}, false);

    chassis.moveToPoint(skills_9.x, skills_9.y, 1000, {.forwards=false,.minSpeed=32}, false);
    clamp_solenoid.extend();

    //goal 2 quadrent 2 + left wall stake
    chassis.moveToPoint(skills_10.x, skills_10.y, 1000, {.forwards=true}, true);
    chassis.waitUntil(20);
    intake_controller.hold();

    chassis.moveToPoint(skills_11.x, skills_11.y, 1000, {.forwards=false}, false);
    clamp_solenoid.retract();

    intake_controller.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(skills_12.x, skills_12.y, skills_12.theta, 1000, {.forwards=true,.minSpeed=48}, false);
    chassis.moveToPose(skills_13.x, skills_13.y, skills_13.theta, 1000, {.forwards=true,.minSpeed=48}, false);
    chassis.moveToPose(skills_14.x, skills_14.y, skills_14.theta, 1000, {.forwards=true}, false);
    chassis.moveToPoint(skills_15.x, skills_15.y, 1000, {.forwards=true}, false);

    chassis.moveToPoint(skills_16.x, skills_16.y, 1000, {.forwards=false,.minSpeed=48}, false);
    intake_controller.set(Intake::IntakeState::STOPPED);
    clamp_solenoid.extend();

    chassis.moveToPoint(skills_17.x, skills_17.y, 1000, {.forwards=true}, true);
    chassis.waitUntil(10);
    arm_controller.moveTo(Arm::position::INTAKE);
    intake_controller.hold();
    chassis.moveToPoint(skills_18.x, skills_18.y, 1000, {.forwards=true}, true);
    chassis.waitUntil(10);
    intake_controller.set(Intake::IntakeState::INTAKING);

    chassis.moveToPoint(skills_19.x, skills_19.y, 1000, {.forwards=false}, false);
    chassis.moveToPoint(skills_20.x, skills_20.y, 1000, {.forwards=true}, false);

    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, false);
    arm_controller.moveTo(Arm::position::INTAKE, false);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(200);
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, false);
    arm_controller.moveTo(Arm::position::RETRACT);

    chassis.moveToPoint(skills_19.x, skills_19.y, 1000, {.forwards=false}, false);
    chassis.moveToPoint(skills_21.x, skills_21.y, 1000, {.forwards=true}, true);
    intake_controller.hold();
    chassis.moveToPose(skills_22.x, skills_22.y, skills_22.theta, 1000, {.forwards=false}, false);
    clamp_solenoid.retract();
    intake_controller.set(Intake::IntakeState::INTAKING);

    chassis.moveToPose(skills_23.x, skills_23.y, skills_23.theta, 1000, {.forwards=true,.minSpeed=48}, false);
    chassis.moveToPose(skills_24.x, skills_24.y, skills_24.theta, 1000, {.forwards=true}, false);
    chassis.moveToPoint(skills_25.x, skills_25.y, 1000, {.forwards=true}, false);

    intake_controller.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(skills_26.x, skills_26.y, 1000, {.forwards=true}, true);
    chassis.waitUntil(20);
    intake_controller.set(Intake::IntakeState::INTAKING);

    chassis.moveToPoint(skills_27.x, skills_27.y, 1000, {.forwards=true}, false);
    clamp_solenoid.extend();

    //goal 3 quadrent 1/4 + allience + goal4

    intake_controller.hold();
    chassis.moveToPoint(skills_28.x, skills_28.y, 1000, {.forwards=true}, false);
    chassis.moveToPose(skills_29.x, skills_29.y, skills_29.theta, 1000, {.forwards=true}, false);
    chassis.moveToPose(skills_30.x, skills_30.y, skills_30.theta, 1000, {.forwards=true}, false);

    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, false);
    arm_controller.moveTo(Arm::position::UP, false);
    arm_controller.moveTo(Arm::position::RETRACT);

    chassis.moveToPose(skills_31.x, skills_31.y, skills_31.theta, 1000, {.forwards=true}, false);

    //climb
    chassis.moveToPose(skills_32.x, skills_32.y, skills_32.theta, 1000, {.forwards=false, .minSpeed=80}, false);
    dt_left.move(127);
    dt_right.move(127);
}

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