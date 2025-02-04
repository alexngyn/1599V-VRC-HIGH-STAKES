#include "autonomous.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "setup.h" 

#define CLAMP_OFFSET -4
#define INTAKE_OFFSET 2
#define ALLIANCE_STAKE_OFFSET 15.5
#define NEUTRAL_STAKE_OFFSET 10

lemlib::Pose offsetPose(lemlib::Pose targetPose, float offset) {
    targetPose.x -= (offset * cos((-targetPose.theta + 90)*3.14159/180));
    targetPose.y -= (offset * sin((-targetPose.theta + 90)*3.14159/180));
    return {targetPose};
}

lemlib::Pose offsetPoint(lemlib::Pose targetPoint, lemlib::Pose prevPoint, float offset) {
    targetPoint.theta = atan2(targetPoint.y - prevPoint.y, targetPoint.x - prevPoint.x);
    targetPoint.x -= (offset * cos((-targetPoint.theta + 90))*3.14159/180);
    targetPoint.y -= (offset * sin((-targetPoint.theta + 90))*3.14159/180);
    return {targetPoint};
}

lemlib::Pose mirrorPose(lemlib::Pose pose) {
    return {-pose.x, pose.y, -pose.theta};
}

// void stakealign() {
//     while (distance_sensor.get() > 10) { dt_left.move(30); dt_right.move(30); };
//     dt_left.move(0); dt_right.move(0);
// }

//======================= pid tuning =======================

void pidtune() {
    // disable all other settings other than kd and kp
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    //chassis.swingToHeading(45, DriveSide::LEFT, 100000);
    //chassis.turnToHeading(45, 100000);
    // increase kp until the robot oscillates then add kd until it stops
    //chassis.moveToPoint(0, 24, 10000);
    // chassis.moveToPoint(-48, 0, 10000);
    chassis.turnToHeading(270, 100000);
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

lemlib::Pose qual_pos_red_0 = {-50.74,-58, 270}; 
lemlib::Pose qual_pos_red_1 = {-32, -58.5, 270};
lemlib::Pose qual_pos_red_2 = offsetPose({0, -47.24, 240}, -6);
lemlib::Pose qual_pos_red_3 = offsetPose({-23.622, -47.244, 120}, 2);
lemlib::Pose qual_pos_red_4 = offsetPoint({-23.622, -47.244, 0}, qual_pos_red_3, 2);
lemlib::Pose qual_pos_red_5a = offsetPose({-23.622-3, -23.622, 180}, -9);
lemlib::Pose qual_pos_red_5 = offsetPose({-23.622-3, -23.622, 180}, -2);
lemlib::Pose qual_pos_red_6 = offsetPoint({-47.244, -6, NAN}, qual_pos_red_5, 2); 
//lemlib::Pose qual_pos_red_7 = offsetPose({-70, -0, 90}, 0);
lemlib::Pose qual_pos_red_7 = offsetPose({-70, -8, 270}, 8); 

lemlib::Pose qual_pos_red_8 = {-24, -24, NAN}; 

void qual_pos_red() {
    chassis.setPose(qual_pos_red_0.x, qual_pos_red_0.y, qual_pos_red_0.theta);
    clamp_solenoid.extend();

    //goal rush
    // chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 2000, {.forwards = false}, false); //rush to mogo
    // chassis.moveToPoint(qual_pos_red_2.x, qual_pos_red_2.y, 2000, {.forwards = false}, false); //rush to mogo

    chassis.moveToPoint(qual_pos_red_1.x , qual_pos_red_1.y, 10000, {.forwards = false}, false); 
    chassis.moveToPose(qual_pos_red_2.x, qual_pos_red_2.y, qual_pos_red_2.theta, 1000, {.forwards = false}, false);
    chassis.moveToPoint(qual_pos_red_2.x, qual_pos_red_2.y, 500, {.forwards = false}, false);
    //printf("%.1f,%.1f,%.1f\n", qual_pos_red_2.x, qual_pos_red_2.y, qual_pos_red_2.theta);

    pros::delay(100);

    clamp_solenoid.retract(); //clamp mogo
    pros::delay(50);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(700);

    // chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 1000, {.forwards = true}, false);
    // chassis.moveToPoint(qual_pos_red_3.x , qual_pos_red_3.y, 1000, {.forwards = true}, false); 

    chassis.moveToPose(qual_pos_red_3.x, qual_pos_red_3.y, qual_pos_red_3.theta, 1000, {.forwards = true}, true);
    intake_controller.hold(true);

    //second mogo

    //chassis.turnToPoint(qual_pos_red_5a.x, qual_pos_red_5a.y, 1000, {.forwards=true}, false); // turn to ring
    // clamp_solenoid.retract();
    // chassis.moveToPoint(qual_pos_red_4.x, qual_pos_red_4.y, 1000, {.forwards = true}, false); // go to ring

    
    chassis.turnToPoint(qual_pos_red_5a.x, qual_pos_red_5a.y, 1000, {.forwards=true, .direction=lemlib::AngularDirection::CW_CLOCKWISE}, false); // turn to ring
    clamp_solenoid.extend();
    chassis.turnToPoint(qual_pos_red_5.x, qual_pos_red_5.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CW_CLOCKWISE}, false); // turn to mogo
    chassis.moveToPoint(qual_pos_red_5a.x, qual_pos_red_5a.y, 1000, {.forwards = false,.maxSpeed=60}, false); // go to mogo
    chassis.moveToPoint(qual_pos_red_5.x, qual_pos_red_5.y, 1000, {.forwards = false}, false); // go to mogo
    clamp_solenoid.retract();
    pros::delay(50);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(3000); // wait for allience

    //alicence stake
    intake_solenoid.extend();
    arm_controller.moveTo(Arm::position::INTAKE);
    chassis.turnToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {}, false); // turn to stake
    chassis.moveToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {.forwards = true}, true); // go to stake
    intake_solenoid.retract();
    chassis.turnToPoint(qual_pos_red_7.x, qual_pos_red_7.y, 1000, {.forwards=true}, false); // turn to stake
    chassis.moveToPose(qual_pos_red_7.x, qual_pos_red_7.y, qual_pos_red_7.theta, 1000, {.forwards = true}, false);
    intake_controller.holdldb(true);
    pros::delay(1000);

    // intake_controller.waitUntilDone();
    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    pros::delay(5000);

    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.moveToPoint(qual_pos_red_8.x, qual_pos_red_8.y, 2000, {.forwards = false, .minSpeed=48}, false); // go to ladder
    // arm_controller.moveTo(Arm::position::RETRACT, true);

    /*

    chassis.waitUntilDone();
    intake_solenoid.retract();
    intake_controller.waitUntilDone();

    chassis.moveToPose(qual_pos_red_7.x, qual_pos_red_7.y, qual_pos_red_8.theta, 1000, {.forwards = true}, false);

    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, false);

    //touch ladder

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(qual_pos_red_8.x, qual_pos_red_8.y, 1000, {.forwards = false, .minSpeed = 96}, true);//go to ladder
    arm_controller.moveTo(-250);//move arm up along the way

    */
}

lemlib::Pose qual_pos_blue_0 = mirrorPose(qual_pos_red_0); 
lemlib::Pose qual_pos_blue_1 = mirrorPose(qual_pos_red_1);
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
    // chassis.moveToPoint(qual_pos_blue_1.x, qual_pos_blue_1.y, 1000, {.forwards = false}, false); //rush to mogo
    // chassis.moveToPoint(qual_pos_blue_2.x, qual_pos_blue_2.y, 1000, {.forwards = false}, false); //rush to mogo

    chassis.moveToPoint(qual_pos_blue_3.x , qual_pos_blue_3.y, 1000, {.forwards = false}, false); 
    chassis.moveToPose(qual_pos_blue_2.x, qual_pos_blue_2.y, qual_pos_blue_2.theta, 1000, {.forwards = false}, false);
    //printf("%.1f,%.1f,%.1f\n", qual_pos_blue_2.x, qual_pos_blue_2.y, qual_pos_blue_2.theta);

    pros::delay(500);

    clamp_solenoid.extend(); //clamp mogo
    pros::delay(50);
    intake_controller.set(Intake::IntakeState::INTAKING);

    // chassis.moveToPoint(qual_pos_blue_1.x, qual_pos_blue_1.y, 1000, {.forwards = true}, false);
    // chassis.moveToPoint(qual_pos_blue_3.x , qual_pos_blue_3.y, 1000, {.forwards = true}, false); 

    chassis.moveToPose(qual_pos_blue_3.x, qual_pos_blue_3.y, qual_pos_blue_3.theta, 1000, {.forwards = true}, false);

    intake_controller.set(Intake::IntakeState::STOPPED);

    //second mogo

    chassis.turnToPoint(qual_pos_blue_4.x, qual_pos_blue_4.y, 1000, {.forwards=true}, false); // turn to ring
    clamp_solenoid.retract();
    intake_controller.hold(true);
    chassis.moveToPoint(qual_pos_blue_4.x, qual_pos_blue_4.y, 1000, {.forwards = true}, false); // go to ring

    intake_controller.waitUntilDone();
    chassis.moveToPoint(qual_pos_blue_5.x, qual_pos_blue_5.y, 1000, {.forwards = false, .minSpeed=64}, false); // go to mogo
    clamp_solenoid.extend();
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(5000);

    //alliance stake
    chassis.turnToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 1000, {}, false); // turn to stake
    chassis.moveToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 1000, {.forwards = true}, true); // go to stake
    intake_solenoid.extend();
    intake_controller.hold(true);

    pros::delay(5000);

    /*

    chassis.waitUntilDone();
    intake_solenoid.retract();
    intake_controller.waitUntilDone();

    chassis.moveToPose(qual_pos_blue_7.x, qual_pos_blue_7.y, qual_pos_blue_8.theta, 1000, {.forwards = true}, false);

    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, false);

    //touch ladder

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(qual_pos_blue_8.x, qual_pos_blue_8.y, 1000, {.forwards = false, .minSpeed = 96}, true);//go to ladder
    arm_controller.moveTo(-250);//move arm up along the way

    */
}


//======================= elims 10p autons =======================

lemlib::Pose elims_pos_red_0 = qual_pos_red_0;
lemlib::Pose elims_pos_red_1 = qual_pos_red_1;
lemlib::Pose elims_pos_red_2 = qual_pos_red_2;
lemlib::Pose elims_pos_red_3 = qual_pos_red_3;
lemlib::Pose elims_pos_red_4 = qual_pos_red_4;
lemlib::Pose elims_pos_red_5 = qual_pos_red_5;
lemlib::Pose elims_pos_red_6 = qual_pos_red_6;
lemlib::Pose elims_pos_red_7 = qual_pos_red_7;
lemlib::Pose elims_pos_red_8 = offsetPoint({-66.5,-66.5,NAN}, elims_pos_red_7, -13);

void elims_pos_red() {
    chassis.setPose(qual_pos_red_0.x, qual_pos_red_0.y, qual_pos_red_0.theta);

    //goal rush
    chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 1000, {.forwards = false}, false); //rush to mogo
    chassis.moveToPoint(qual_pos_red_2.x, qual_pos_red_2.y, 1000, {.forwards = false}, false); //rush to mogo
    clamp_solenoid.retract(); //clamp mogo

    chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 1000, {.forwards = true}, false);
    chassis.moveToPoint(qual_pos_red_3.x , qual_pos_red_3.y, 1000, {.forwards = true}, false); 

    //second mogo

    chassis.turnToPoint(qual_pos_red_4.x, qual_pos_red_4.y, 1000, {.forwards=true}, false); // turn to ring
    clamp_solenoid.extend();
    intake_controller.hold(true);
    chassis.moveToPoint(qual_pos_red_4.x, qual_pos_red_4.y, 1000, {.forwards = true}, false); // go to ring
    intake_controller.waitUntilDone();
    chassis.moveToPoint(qual_pos_red_5.x, qual_pos_red_5.y, 1000, {.forwards = false}, false); // go to mogo
    clamp_solenoid.retract();
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);

    //alicence stake
    chassis.turnToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {}, false); // turn to stake
    chassis.moveToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {.forwards = true}, true); // go to stake
    intake_solenoid.retract();
    intake_controller.hold(true);

    chassis.waitUntilDone();
    intake_solenoid.extend();
    intake_controller.waitUntilDone();

    chassis.moveToPose(qual_pos_red_7.x, qual_pos_red_7.y, qual_pos_red_8.theta, 1000, {.forwards = true}, false);

    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, false);

    //go to corner

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(qual_pos_red_8.x, qual_pos_red_8.y, 1000, {.forwards = false, .maxSpeed = 96}, true);//go to ladder
}

lemlib::Pose elims_pos_blue_0 = qual_pos_blue_0;
lemlib::Pose elims_pos_blue_1 = qual_pos_blue_1;
lemlib::Pose elims_pos_blue_2 = qual_pos_blue_2;
lemlib::Pose elims_pos_blue_3 = qual_pos_blue_3;
lemlib::Pose elims_pos_blue_4 = qual_pos_blue_4;
lemlib::Pose elims_pos_blue_5 = qual_pos_blue_5;
lemlib::Pose elims_pos_blue_6 = qual_pos_blue_6;
lemlib::Pose elims_pos_blue_7 = qual_pos_blue_7;
lemlib::Pose elims_pos_blue_8 = mirrorPose(elims_pos_red_8);

void elims_pos_blue() {
    chassis.setPose(elims_pos_blue_0.x, elims_pos_blue_0.y, elims_pos_blue_0.theta);

    //goal rush
    chassis.moveToPoint(elims_pos_blue_1.x, elims_pos_blue_1.y, 1000, {.forwards = false}, false); //rush to mogo
    chassis.moveToPoint(elims_pos_blue_2.x, elims_pos_blue_2.y, 1000, {.forwards = false}, false); //rush to mogo
    clamp_solenoid.retract(); //clamp mogo

    chassis.moveToPoint(elims_pos_blue_1.x, elims_pos_blue_1.y, 1000, {.forwards = true}, false);
    chassis.moveToPoint(elims_pos_blue_3.x , elims_pos_blue_3.y, 1000, {.forwards = true}, false); 

    //second mogo

    chassis.turnToPoint(elims_pos_blue_4.x, elims_pos_blue_4.y, 1000, {.forwards=true}, false); // turn to ring
    clamp_solenoid.extend();
    intake_controller.hold(true);
    chassis.moveToPoint(elims_pos_blue_4.x, elims_pos_blue_4.y, 1000, {.forwards = true}, false); // go to ring
    intake_controller.waitUntilDone();
    chassis.moveToPoint(elims_pos_blue_5.x, elims_pos_blue_5.y, 1000, {.forwards = false}, false); // go to mogo
    clamp_solenoid.retract();
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);

    //alicence stake
    chassis.turnToPoint(elims_pos_blue_6.x, elims_pos_blue_6.y, 1000, {}, false); // turn to stake
    chassis.moveToPoint(elims_pos_blue_6.x, elims_pos_blue_6.y, 1000, {.forwards = true}, true); // go to stake
    intake_solenoid.retract();
    intake_controller.hold(true);

    chassis.waitUntilDone();
    intake_solenoid.extend();
    intake_controller.waitUntilDone();

    chassis.moveToPose(elims_pos_blue_7.x, elims_pos_blue_7.y, elims_pos_blue_8.theta, 1000, {.forwards = true}, false);

    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, false);

    //go to corner

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(elims_pos_blue_8.x, elims_pos_blue_8.y, 1000, {.forwards = false, .maxSpeed = 96}, true);//go to ladder
}

//======================= skills autons =======================

lemlib::Pose skills_0 = offsetPose({-70, 0, 270}, 15.5); 
lemlib::Pose skills_1a = offsetPose(skills_0, 4);
lemlib::Pose skills_1 = offsetPose({-47.244+0.8, -23.622, 0}, -2);
lemlib::Pose skills_1b = {NAN,NAN, 60};
lemlib::Pose skills_2 = {-23.622, -23.622, 135};
lemlib::Pose skills_3 = {-23.622, -47.24, 200};
lemlib::Pose skills_4 = offsetPose({23.622+6, -47.244, 90},0);
lemlib::Pose skills_5 = offsetPose({5.5, -61, 180},0);
lemlib::Pose skills_5a = offsetPose({5.5, -70, 180},10);
lemlib::Pose skills_6 = offsetPose({-47.244, -52, 270},0); 
lemlib::Pose skills_7 = offsetPose({-56.055, -52, 270},0);
lemlib::Pose skills_8 = offsetPoint({-47.244, -64, 135},skills_7,4);
lemlib::Pose skills_9 = offsetPoint({-58, -67, 45},skills_8,0);

lemlib::Pose skills_10 = offsetPoint({4, -2, 45},skills_9,0);
lemlib::Pose skills_11a = {-18,15,NAN};
lemlib::Pose skills_11 = offsetPose({-47.244+4, 23.622-2, 120},-4);
lemlib::Pose skills_12 = {-23.622, 23.622, 17.5};
lemlib::Pose skills_13 = {-23.622, 47.244, 310};
lemlib::Pose skills_14 = offsetPose({-59.055, 47.244, 270},2);
lemlib::Pose skills_15 = offsetPoint({-47.244, 59.055, 45},skills_14,2);
lemlib::Pose skills_16 = offsetPoint({-65, 65, 110},skills_15,-15);

lemlib::Pose skills_17 = offsetPoint({0, 58, 110},skills_16,2);
lemlib::Pose skills_18 = offsetPoint({23.622, 47.244, 110},skills_17,2);
lemlib::Pose skills_19a = offsetPose({0, -59.055, 0},2);
lemlib::Pose skills_19 = offsetPose({0, 70, 0},10);
lemlib::Pose skills_20 = {23.622, 23.622, 140};
lemlib::Pose skills_21 = offsetPose({47.035, -0.447, 300},-4);
lemlib::Pose skills_22 = offsetPose({23.622, -23.622, 200},2);
lemlib::Pose skills_23 = offsetPose({59.055, -47.244, 90},2);
lemlib::Pose skills_24 = offsetPoint({47.244, -59.055, 0},skills_23,2);
lemlib::Pose skills_25 = {47.244, 0, 0};
lemlib::Pose skills_26 = offsetPose({47.244, 59.055, 0},2);
lemlib::Pose skills_27 = offsetPoint({65, 65, 135},skills_26,-15);

lemlib::Pose skills_28 = {59.055, 47.244, 210};
lemlib::Pose skills_29 = {42.702, 24.236, 210};
lemlib::Pose skills_30 = offsetPose({0, 59.055, 90},2);
lemlib::Pose skills_30a = offsetPose({70, 0, 90},15.5);
lemlib::Pose skills_31 = offsetPoint({59.055, -23.622, NAN},skills_30a, -4);
lemlib::Pose skills_32 = offsetPoint({65, -65, 135},skills_31,-15);
lemlib::Pose skills_33 = {16, -16, 135};
void skills() {
    chassis.setPose(skills_0.x, skills_0.y, skills_0.theta);
    
    //allicen stake
    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    pros::delay(1000);
    arm_controller.moveTo(Arm::position::RETRACT);
    clamp_solenoid.extend();

    //goal 1 quadrent 3 + right wall stake

    chassis.moveToPoint(skills_1a.x, skills_1a.y, 1000, {.forwards=false, .maxSpeed=96}, true);
    chassis.moveToPose(skills_1.x, skills_1.y, skills_1.theta, 2000, {.forwards=false, .maxSpeed=96}, false);
    //chassis.moveToPoint(skills_1.x, skills_1.y, 1000, {.forwards=false, .maxSpeed=64}, true);
    arm_controller.moveTo(Arm::position::RETRACT);

    pros::delay(100);

    clamp_solenoid.retract();

    chassis.turnToHeading(skills_1b.theta, 800);
    intake_controller.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(skills_2.x, skills_2.y, skills_2.theta, 2000, {.forwards=true, .maxSpeed=96, .minSpeed=36}, false);
    chassis.moveToPoint(skills_3.x, skills_3.y, 2000, {.forwards=true, .maxSpeed=96}, false);

    chassis.turnToHeading(skills_4.theta, 1000);
    chassis.moveToPoint(skills_4.x, skills_4.y, 2000, {.forwards=true, .maxSpeed=96}, false);

    chassis.moveToPoint(skills_5.x, skills_5.y, 2000, {.forwards=true, .maxSpeed=96}, true);
    pros::delay(500);
    arm_controller.moveTo(Arm::position::INTAKE, true);
    intake_controller.holdldb(true,1000);
    chassis.turnToPoint(4, -70, 1000,{},false);
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, false, 2000);
    //chassis.moveToPose(skills_5a.x, skills_5a.y, skills_5a.theta, 2000, {.forwards=true, .maxSpeed=96}, false);
    // arm_controller.moveTo(Arm::position::SCORE_NEUTRAL,false,3000);

    chassis.moveToPose(skills_6.x , skills_6.y, skills_6.theta, 3000, {.forwards=true,.minSpeed=48}, true);
    // pros::delay(1000);
    arm_controller.moveTo(Arm::position::RETRACT);
    chassis.moveToPoint(skills_7.x, skills_7.y, 2000, {.forwards=true, .maxSpeed=96}, false);
    chassis.moveToPoint(skills_8.x, skills_8.y, 2000, {.forwards=true, .maxSpeed=96}, false);

    chassis.moveToPoint(skills_9.x, skills_9.y, 2000, {.forwards=false, .maxSpeed=96}, false);
    clamp_solenoid.extend();
    pros::delay(100);


    //goal 2 quadrent 2 + left wall stake
    chassis.moveToPoint(skills_10.x, skills_10.y, 3000, {.forwards=true, .maxSpeed=96}, true);
    chassis.waitUntil(5);
    intake_controller.hold(true,1000);

    //chassis.turnToPoint(skills_11.x, skills_11.y, 1000, {.forwards=false, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_11a.x, skills_11a.y, 2000, {.forwards=false, .maxSpeed=96}, false);
    chassis.moveToPose(skills_11.x, skills_11.y, skills_11.theta, 2000, {.forwards=false, .maxSpeed=96}, false);
    clamp_solenoid.retract();

    intake_controller.set(Intake::IntakeState::INTAKING);
    // chassis.moveToPose(skills_12.x, skills_12.y, skills_12.theta, 1000, {.forwards=true,.minSpeed=24}, false);
    // chassis.moveToPose(skills_13.x, skills_13.y, skills_13.theta, 1000, {.forwards=true,.minSpeed=24}, false);
    // chassis.moveToPose(skills_14.x, skills_14.y, skills_14.theta, 1000, {.forwards=true, .maxSpeed=64}, false);
    // chassis.moveToPoint(skills_15.x, skills_15.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    // chassis.moveToPoint(skills_16.x, skills_16.y, 1000, {.forwards=false,.minSpeed=48}, false);
    // //intake_controller.set(Intake::IntakeState::STOPPED);
    // clamp_solenoid.retract();

    // // left wall stake
    // chassis.moveToPoint(skills_17.x, skills_17.y, 1000, {.forwards=true, .maxSpeed=64}, true);
    // //chassis.waitUntil(10);
    // arm_controller.moveTo(Arm::position::INTAKE);
    // intake_controller.holdldb(false, 1000);
    // //intake_controller.hold();
    // //chassis.moveToPoint(skills_18.x, skills_18.y, 1000, {.forwards=true}, true);
    // //chassis.waitUntil(8);
    // //intake_controller.set(Intake::IntakeState::INTAKING);
    // //intake_controller.waitUntilDone();

    // chassis.moveToPose(skills_19a.x, skills_19a.y, skills_19a.theta, 1000, {.forwards=true, .maxSpeed=64}, false);
    // chassis.moveToPoint(skills_19.x, skills_19.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    // arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, true);
    // pros::delay(3000);

    /*
    
    //next mogo

    chassis.moveToPoint(skills_20.x, skills_20.y, 1000, {.forwards=true, .maxSpeed=64}, true);
    arm_controller.moveTo(Arm::position::RETRACT);
    intake_controller.hold();
    chassis.moveToPose(skills_21.x, skills_21.y, skills_21.theta, 1000, {.forwards=false, .maxSpeed=64}, false);
    clamp_solenoid.extend();
    intake_controller.set(Intake::IntakeState::INTAKING);

    chassis.moveToPose(skills_22.x, skills_22.y, skills_22.theta, 1000, {.forwards=true,.minSpeed=24}, false);
    chassis.moveToPose(skills_23.x, skills_23.y, skills_23.theta, 1000, {.forwards=true, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_24.x, skills_24.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    chassis.moveToPoint(skills_25.x, skills_25.y, 1000, {.forwards=true,.minSpeed=48}, false);
    chassis.moveToPoint(skills_26.x, skills_26.y, 1000, {.forwards=true, .maxSpeed=64}, true);

    chassis.moveToPoint(skills_27.x, skills_27.y, 1000, {.forwards=false, .maxSpeed=64}, false);
    clamp_solenoid.retract();

    //goal 3 quadrent 1/4 + allience + goal4
    arm_controller.moveTo(Arm::position::INTAKE);
    intake_controller.holdldb(true, 1000);
    
    //intake_controller.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(skills_28.x, skills_28.y, 1000, {.forwards=true, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_29.x, skills_29.y, 1000, {.forwards=true, .maxSpeed=64}, false);
    
    chassis.moveToPose(skills_30.x, skills_30.y, skills_30.theta, 1000, {.forwards=true, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_30a.x, skills_30a.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    pros::delay(3000);

    chassis.moveToPoint(skills_30.x, skills_30.y, 1000, {.forwards=true, .maxSpeed=64}, false);
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL,true);

    //clamp_solenoid.extend();
    chassis.moveToPoint(skills_31.x, skills_31.y, 1000, {.forwards=true, .minSpeed=24}, false);
    chassis.moveToPoint(skills_32.x, skills_32.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    //climb
    chassis.moveToPose(skills_33.x, skills_33.y, skills_33.theta, 10000, {.forwards=false, .minSpeed=80}, false);
    //chassis.turnToPoint(skills_33.x, skills_33.y, 1000, {.forwards=false}, false);
    //dt_left.move(127);
    //dt_right.move(127);
    //pros::delay(20000);

    */
}
