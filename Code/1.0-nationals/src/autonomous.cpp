#include "autonomous.h"
#include "arm.h"
#include "intake.h"
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
    //chassis.turnToHeading(270, 100000);
    // chassis.moveToPoint(-48, 0, 10000);

    //chassis.turnToPoint(qual_pos_red_7.x, qual_pos_red_7.y, 1000, {.forwards=true}, true); // turn to stake
    //intake_solenoid.retract();
    //chassis.moveToPoint(qual_pos_red_7.x, qual_pos_red_7.y, 1000, {.forwards = true}, false);
    //chassis.turnToHeading(qual_pos_red_7.theta, 500);
    arm_controller.moveTo(Arm::position::INTAKE);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(3000);
   // intake_controller.holdldb(false,50000);
    //pros::delay(1000);
    intake_controller.set(Intake::IntakeState::OUTTAKE);
    pros::delay(200);
    intake_controller.set(Intake::IntakeState::STOPPED);
    pros::delay(50);

    // intake_controller.waitUntilDone();
    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    pros::delay(800);

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
lemlib::Pose qual_pos_red_3 = offsetPose({-23.622, -47.244, 120}, 0);
lemlib::Pose qual_pos_red_4 = offsetPose({-23.622-0, -23.622, 180}, -9);
lemlib::Pose qual_pos_red_5 = offsetPose({-23.622+0, -23.622, 180}, -2); //x+2
//lemlib::Pose qual_pos_red_6 = offsetPoint({-47.244, -5, NAN}, qual_pos_red_5, 0);  //51
lemlib::Pose qual_pos_red_6 = offsetPose({-72, -4, -63}, 18);  //16 -61.97 degs
lemlib::Pose qual_pos_red_6a = offsetPose({-72, -4, -63}, 28);
lemlib::Pose qual_pos_red_7 = {-49, -0, NAN};
lemlib::Pose qual_pos_red_7a = offsetPose({-70,0, 270}, 11.5); //58.5 11.5

lemlib::Pose qual_pos_red_8 = {-24, -24, NAN}; 
lemlib::Pose qual_pos_red_9 = {-14, -14, NAN}; 

void qual_pos_red() {
    chassis.setPose(qual_pos_red_0.x, qual_pos_red_0.y, qual_pos_red_0.theta);
    clamp_solenoid.extend();

    //goal rush

    chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 1000, {.forwards = false,.minSpeed=127}, false); //rush to mogo
    chassis.moveToPose(qual_pos_red_2.x, qual_pos_red_2.y, qual_pos_red_2.theta, 1000, {.forwards = false}, false); //rush to mogo
    chassis.moveToPoint(qual_pos_red_2.x, qual_pos_red_2.y, 500, {.forwards = false}, false); //rush to mogo

    pros::delay(100);

    clamp_solenoid.retract(); //clamp mogo
    pros::delay(50);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(400);

    chassis.moveToPose(qual_pos_red_3.x, qual_pos_red_3.y, qual_pos_red_3.theta, 1000, {.forwards = true}, false);
    intake_controller.hold(true);
    arm_controller.moveTo(Arm::position::INTAKE);

    //second mogo

    chassis.turnToPoint(qual_pos_red_4.x, qual_pos_red_4.y, 1000, {.forwards=true, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=64}, false); // turn to ring
    clamp_solenoid.extend();
    chassis.turnToPoint(qual_pos_red_5.x, qual_pos_red_5.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=64}, false); // turn to mogo
    chassis.moveToPoint(qual_pos_red_4.x, qual_pos_red_4.y, 1000, {.forwards = false,.maxSpeed=64}, false); // go to mogo
    chassis.moveToPoint(qual_pos_red_5.x, qual_pos_red_5.y, 1000, {.forwards = false,.maxSpeed=64}, false); // go to mogo
    clamp_solenoid.retract();
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(400); // wait for allience

    chassis.turnToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {.forwards=true}, true); // turn to stake
    chassis.moveToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 4000, {.forwards = true,.maxSpeed=64}, false);

    //pros::delay(500);

    intake_controller.set(Intake::IntakeState::OUTTAKE);
    pros::delay(200);
    intake_controller.set(Intake::IntakeState::STOPPED);
    pros::delay(50);

    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    pros::delay(600);
    chassis.moveToPoint(qual_pos_red_6a.x, qual_pos_red_6a.y, 2000, {.forwards = false}, false); // go to ladder
    intake_controller.set(Intake::IntakeState::INTAKING);
    intake_solenoid.extend();

    chassis.turnToPoint(qual_pos_red_7.x, qual_pos_red_7.y, 1000, {.forwards=true}, true); // turn to stake
    chassis.moveToPoint(qual_pos_red_7.x, qual_pos_red_7.y, 2000, {.forwards = true,.maxSpeed=48}, false); // go to ladder
    arm_controller.moveTo(Arm::position::RETRACT, true);

    pros::delay(200);
    intake_solenoid.retract();

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    dt_left.move(-64);
    dt_right.move(-64);
    pros::delay(300);
    dt_left.move(0);
    dt_right.move(0);

    pros::delay(5000);

}

lemlib::Pose qual_pos_blue_0 = mirrorPose(qual_pos_red_0); 
lemlib::Pose qual_pos_blue_1 = mirrorPose(qual_pos_red_1);
lemlib::Pose qual_pos_blue_2 = mirrorPose(qual_pos_red_2);
lemlib::Pose qual_pos_blue_3 = mirrorPose(qual_pos_red_3);
lemlib::Pose qual_pos_blue_4 = mirrorPose(qual_pos_red_4);
lemlib::Pose qual_pos_blue_5 = mirrorPose(qual_pos_red_5);
lemlib::Pose qual_pos_blue_6 = mirrorPose(qual_pos_red_6);
lemlib::Pose qual_pos_blue_6a = mirrorPose(qual_pos_red_6a);
lemlib::Pose qual_pos_blue_7 = mirrorPose(qual_pos_red_7);
lemlib::Pose qual_pos_blue_7a = mirrorPose(qual_pos_red_7a);
lemlib::Pose qual_pos_blue_8 = mirrorPose(qual_pos_red_8);
lemlib::Pose qual_pos_blue_9 = mirrorPose(qual_pos_red_9);

void qual_pos_blue() {
    chassis.setPose(qual_pos_blue_0.x, qual_pos_blue_0.y, qual_pos_blue_0.theta);
    clamp_solenoid.extend();

    //goal rush

    chassis.moveToPoint(qual_pos_blue_1.x, qual_pos_blue_1.y, 1000, {.forwards = false,.minSpeed=127}, false); //rush to mogo
    chassis.moveToPose(qual_pos_blue_2.x, qual_pos_blue_2.y, qual_pos_blue_2.theta, 1000, {.forwards = false}, false); //rush to mogo
    chassis.moveToPoint(qual_pos_blue_2.x, qual_pos_blue_2.y, 500, {.forwards = false}, false); //rush to mogo

    pros::delay(100);

    clamp_solenoid.retract(); //clamp mogo
    pros::delay(50);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(400);

    chassis.moveToPose(qual_pos_blue_3.x, qual_pos_blue_3.y, qual_pos_blue_3.theta, 1000, {.forwards = true}, true);
    intake_controller.hold(true);
    arm_controller.moveTo(Arm::position::INTAKE);

    //second mogo

    chassis.turnToPoint(qual_pos_blue_4.x, qual_pos_blue_4.y, 1000, {.forwards=true, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=64}, false); // turn to ring
    clamp_solenoid.extend();
    chassis.turnToPoint(qual_pos_blue_5.x, qual_pos_blue_5.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=64}, false); // turn to mogo
    chassis.moveToPoint(qual_pos_blue_4.x, qual_pos_blue_4.y, 1000, {.forwards = false,.maxSpeed=64}, false); // go to mogo
    chassis.moveToPoint(qual_pos_blue_5.x, qual_pos_blue_5.y, 1000, {.forwards = false,.maxSpeed=64}, false); // go to mogo
    clamp_solenoid.retract();
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(400); // wait for allience

    chassis.turnToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 1000, {.forwards=true, .maxSpeed=64}, true); // turn to stake
    chassis.moveToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 4000, {.forwards = true,.maxSpeed=64}, false);
    
    //pros::delay(500);

    intake_controller.set(Intake::IntakeState::OUTTAKE);
    pros::delay(200);
    intake_controller.set(Intake::IntakeState::STOPPED);
    pros::delay(50);
    
    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    pros::delay(600);
    chassis.moveToPoint(qual_pos_blue_6a.x, qual_pos_blue_6a.y, 2000, {.forwards = false}, false); // go to ladder
    intake_controller.set(Intake::IntakeState::INTAKING);
    intake_solenoid.extend();

    chassis.turnToPoint(qual_pos_blue_7.x, qual_pos_blue_7.y, 1000, {.forwards=true}, true); // turn to stake
    chassis.moveToPoint(qual_pos_blue_7.x, qual_pos_blue_7.y, 2000, {.forwards = true,.maxSpeed=48}, false); // go to ladder
    arm_controller.moveTo(Arm::position::RETRACT, true);

    pros::delay(200);
    intake_solenoid.retract();

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    dt_left.move(-64);
    dt_right.move(-64);
    pros::delay(300);
    dt_left.move(0);
    dt_right.move(0);

    pros::delay(5000);

    //alliance stake

//     intake_solenoid.extend();
//     arm_controller.moveTo(Arm::position::INTAKE);
//     chassis.turnToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 1000, {}, false); // turn to stake
//     chassis.moveToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 1000, {.forwards = true, .maxSpeed=48}, false); // go to stake
//     //chassis.turnToPoint(qual_pos_blue_7.x, qual_pos_blue_7.y, 1000, {.forwards=true}, true); // turn to stake
//     //intake_solenoid.retract();
//    // chassis.moveToPoint(qual_pos_blue_7.x, qual_pos_blue_7.y, 1000, {.forwards = true}, false);
//    //intake_controller.holdldb(true);
//     //pros::delay(1000);
//     intake_solenoid.retract();
//     chassis.turnToHeading(qual_pos_blue_7.theta, 500);
//     //chassis.moveToPoint(qual_pos_blue_7.x, qual_pos_blue_7.y, 1000, {.forwards = true}, false);
    
//     chassis.moveToPose(qual_pos_blue_7a.x, qual_pos_blue_7a.y, qual_pos_blue_7a.theta, 1000, {.forwards = true}, false);
//     //chassis.turnToHeading(qual_pos_blue_7.theta, 1000);


//     // // intake_controller.waitUntilDone();
//     intake_solenoid.retract();
//     arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
//     pros::delay(8000);

    //chassis.moveToPoint(qual_pos_blue_8.x, qual_pos_blue_8.y, 2000, {.forwards = false}, false); // go to ladder

    //chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    //chassis.moveToPoint(qual_pos_blue_9.x, qual_pos_blue_9.y, 2000, {.forwards = false}, true); // go to ladder
    //arm_controller.moveTo(-290);//move arm up along the way
    // arm_controller.moveTo(Arm::position::RETRACT, true);
}

//======================= skills autons =======================

lemlib::Pose skills_0 = {-62.994, 0, 90}; 
lemlib::Pose skills_1a = {-47.244, 0, 90};
lemlib::Pose skills_1 = offsetPose({-47.244, -23.622, 0}, -2);
lemlib::Pose skills_1b = {NAN,NAN, 60};
lemlib::Pose skills_2 = {-23.622, -23.622, 135};
lemlib::Pose skills_3 = {-23.622, -47.24-4, 200};
lemlib::Pose skills_4 = offsetPose({23.622+0, -47.244-4, 90},0);
lemlib::Pose skills_5 = offsetPose({1, -62, 180},0);
lemlib::Pose skills_5a = offsetPose({1, -57, 180},0);
lemlib::Pose skills_6 = offsetPose({-47.244, -47.244-4, 270},0); 
lemlib::Pose skills_7 = offsetPose({-56.055-3, -47.244-4, 270},0);
lemlib::Pose skills_8 = offsetPoint({-47.244, -59.055+0, 135},skills_7,4);
lemlib::Pose skills_9 = offsetPoint({-58, -67, 45},skills_8,2);

lemlib::Pose skills_10 = offsetPoint({-47-2, -47, 45},skills_9,0);
//lemlib::Pose skills_10 = offsetPoint({0, 0, 45},skills_9,0);
//lemlib::Pose skills_11a = {-38,23.5,NAN};
lemlib::Pose skills_11 = offsetPose({-47.244-2, 23.622+2, 180},0);
lemlib::Pose skills_11a = offsetPose({NAN,NAN,120},0);
lemlib::Pose skills_12 = {-23.622+0, 23.622, 17.5};
lemlib::Pose skills_13 = {-23.622+0, 47.244, 310};
lemlib::Pose skills_14 = offsetPose({-59.055-8, 47.244, 270},2);
lemlib::Pose skills_15 = offsetPoint({-47.244, 59.055, 45},skills_14,2);
lemlib::Pose skills_16 = offsetPoint({-65, 65, 110},skills_15,-20);

lemlib::Pose skills_17 = offsetPoint({0, 42, 110},skills_16,0);
//lemlib::Pose skills_18 = offsetPoint({23.622, 47.244, 110},skills_17,2);
//lemlib::Pose skills_19a = offsetPose({0, 59.055, 0},2);
//lemlib::Pose skills_19 = offsetPose({0, 70, 0},10);
lemlib::Pose skills_20 = {23.622, 23.622, 140};

lemlib::Pose skills_21 = offsetPose({47.035, -0.447, 300},-4);
// lemlib::Pose skills_22 = offsetPose({23.622, -23.622, 200},2);
// lemlib::Pose skills_23 = offsetPose({59.055, -47.244, 90},2);
// lemlib::Pose skills_24 = offsetPoint({47.244, -59.055, 0},skills_23,2);
// lemlib::Pose skills_25 = {47.244, 0, 0};
lemlib::Pose skills_26 = offsetPose({47.244, 59.055, 0},2);
lemlib::Pose skills_27 = offsetPoint({65, 65, 135},skills_26,-15);

lemlib::Pose skills_28 = {59.055, 47.244, 210};
lemlib::Pose skills_29 = {42.702, 24.236, 210};
lemlib::Pose skills_30 = offsetPose({0, 59.055, 90},2);
lemlib::Pose skills_30a = offsetPose({70, 0, 90},15.5);
lemlib::Pose skills_31 = offsetPoint({59.055, -23.622, NAN},skills_30a, -4);
lemlib::Pose skills_32 = offsetPoint({65, -65, 135},skills_31,-20);
lemlib::Pose skills_33 = {16, -16, 135};
lemlib::Pose skills_34 = {14, 14, NAN};
void skills() {
    sideColor = color::red;
    chassis.setPose(skills_0.x, skills_0.y, skills_0.theta);
    
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(500);
    //allicen stake
    //arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    //pros::delay(1000);
    //arm_controller.moveTo(Arm::position::RETRACT);
    clamp_solenoid.extend();

    //goal 1 quadrent 3 + right wall stake

    chassis.moveToPoint(skills_1a.x, skills_1a.y, 2000, {.forwards=true, .maxSpeed=64}, true);
    chassis.moveToPose(skills_1.x, skills_1.y, skills_1.theta, 2000, {.forwards=false, .maxSpeed=64}, false);
    // //chassis.moveToPoint(skills_1.x, skills_1.y, 1000, {.forwards=false, .maxSpeed=64}, true);
    // arm_controller.moveTo(Arm::position::RETRACT);

    pros::delay(100);

    intake_controller.set(Intake::IntakeState::OUTTAKE);

    clamp_solenoid.retract();

    chassis.turnToHeading(skills_1b.theta, 800);
    intake_controller.set(Intake::IntakeState::INTAKING);
    // intake_controller.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(skills_2.x, skills_2.y, skills_2.theta, 2000, {.forwards=true, .maxSpeed=64, .minSpeed=36}, false);
    chassis.moveToPoint(skills_3.x, skills_3.y, 2000, {.forwards=true, .maxSpeed=64}, false);

    chassis.turnToHeading(skills_4.theta, 1000);
    chassis.moveToPoint(skills_4.x, skills_4.y, 2000, {.forwards=true, .maxSpeed=64}, false);

    chassis.moveToPoint(skills_5.x, skills_5.y, 2000, {.forwards=true, .maxSpeed=64}, true);
    pros::delay(1000);
    //arm_controller.moveTo(Arm::position::INTAKE, true);
    //intake_controller.holdldb(true,1000);
    //chassis.turnToPoint(1, -70, 1000,{},false);
    chassis.moveToPoint(1, -66, 1000, {.forwards=false, .maxSpeed=64}, false);
    //arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, true, 1000);
    //pros::delay(1000);
    chassis.moveToPoint(skills_5a.x, skills_5a.y, 2000, {.forwards=false, .maxSpeed=64}, true);
    //pros::delay(100);
    //arm_controller.moveTo(Arm::position::RETRACT);
    // arm_controller.moveTo(Arm::position::SCORE_NEUTRAL,false,3000);
    intake_controller.set(Intake::IntakeState::INTAKING);

    chassis.moveToPose(skills_6.x , skills_6.y, skills_6.theta, 3000, {.forwards=true,.minSpeed=48}, true);
    // pros::delay(1000);
    chassis.moveToPoint(skills_7.x, skills_7.y, 2000, {.forwards=true, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_8.x, skills_8.y, 2000, {.forwards=true, .maxSpeed=64}, false);

    chassis.moveToPoint(skills_9.x, skills_9.y, 2000, {.forwards=false, .maxSpeed=64}, false);
    clamp_solenoid.extend();
    intake_controller.set(Intake::IntakeState::OUTTAKE);
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::STOPPED);


    // //goal 2 quadrent 2 + left wall stake
    chassis.moveToPoint(skills_10.x, skills_10.y, 3000, {.forwards=true, .maxSpeed=64}, true);
    // chassis.waitUntil(5);
    // intake_controller.hold(true,1000);

    // chassis.turnToPoint(skills_11.x, skills_11.y, 1000, {.forwards=false, .maxSpeed=64}, false);
    // chassis.moveToPoint(skills_11a.x, skills_11a.y, 2000, {.forwards=false, .maxSpeed=64}, false);
    chassis.moveToPose(skills_11.x, skills_11.y, skills_11.theta, 3000, {.forwards=false, .maxSpeed=64}, false);
    clamp_solenoid.retract();
    pros::delay(200);

    chassis.turnToHeading(skills_11a.theta, 1000);

    intake_controller.set(Intake::IntakeState::INTAKING);
    //chassis.moveToPose(skills_12.x, skills_12.y, skills_12.theta, 2000, {.forwards=true, .maxSpeed=64}, false);
    chassis.moveToPose(skills_12.x, skills_12.y, skills_12.theta, 2000, {.forwards=true, .maxSpeed=64,.minSpeed=0}, false);
    chassis.moveToPose(skills_13.x, skills_13.y, skills_13.theta, 2000, {.forwards=true, .maxSpeed=64,.minSpeed=0}, false);
    chassis.moveToPose(skills_14.x, skills_14.y, skills_14.theta, 2000, {.forwards=true, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_15.x, skills_15.y, 2000, {.forwards=true, .maxSpeed=64}, false);

    chassis.moveToPoint(skills_16.x, skills_16.y, 2000, {.forwards=false,.minSpeed=48}, false);
    intake_controller.set(Intake::IntakeState::OUTTAKE);
    clamp_solenoid.extend();
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);

    // left wall stake
    chassis.moveToPoint(skills_17.x, skills_17.y, 3000, {.forwards=true, .maxSpeed=64}, true);
    //chassis.waitUntil(10);
    //arm_controller.moveTo(Arm::position::INTAKE);
    //intake_controller.holdldb(true, 1000);
    //intake_controller.hold();
    // chassis.moveToPoint(skills_18.x, skills_18.y, 1000, {.forwards=true}, true);
    //chassis.waitUntil(8);
    //intake_controller.set(Intake::IntakeState::INTAKING);
    //intake_controller.waitUntilDone();

    //chassis.moveToPose(skills_19a.x, skills_19a.y, skills_19a.theta, 2000, {.forwards=true, .maxSpeed=64}, false);
    // chassis.moveToPoint(skills_19.x, skills_19.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    //arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, true);
    //pros::delay(1000);

    
    
    //next mogo

    chassis.moveToPoint(skills_20.x, skills_20.y, 2000, {.forwards=true, .maxSpeed=64}, true);
    // arm_controller.moveTo(Arm::position::RETRACT);
    pros::delay(1000);
    intake_controller.hold();
    chassis.moveToPose(skills_21.x, skills_21.y, skills_21.theta, 1000, {.forwards=false, .maxSpeed=64}, false);
    clamp_solenoid.retract();
    intake_controller.set(Intake::IntakeState::INTAKING);

    // chassis.moveToPose(skills_22.x, skills_22.y, skills_22.theta, 1000, {.forwards=true,.minSpeed=24}, false);
    // chassis.moveToPose(skills_23.x, skills_23.y, skills_23.theta, 1000, {.forwards=true, .maxSpeed=64}, false);
    // chassis.moveToPoint(skills_24.x, skills_24.y, 2000, {.forwards=true, .maxSpeed=64}, false);

    // chassis.moveToPoint(skills_25.x, skills_25.y, 2000, {.forwards=true,.minSpeed=48}, false);
    chassis.moveToPoint(skills_26.x, skills_26.y, 2000, {.forwards=true, .maxSpeed=64}, true);

    chassis.moveToPoint(skills_27.x, skills_27.y, 2000, {.forwards=false, .maxSpeed=64}, false);
    clamp_solenoid.extend();
    chassis.moveToPoint(skills_34.x, skills_34.y, 2000, {.forwards=true, .maxSpeed=64}, false);



    //goal 3 quadrent 1/4 + allience + goal4
    // arm_controller.moveTo(Arm::position::INTAKE);
    // intake_controller.holdldb(true, 1000);
    
    // //intake_controller.set(Intake::IntakeState::INTAKING);
    // chassis.moveToPoint(skills_28.x, skills_28.y, 1000, {.forwards=true, .maxSpeed=64}, false);
    // chassis.moveToPoint(skills_29.x, skills_29.y, 1000, {.forwards=true, .maxSpeed=64}, false);
    
    // chassis.moveToPose(skills_30.x, skills_30.y, skills_30.theta, 1000, {.forwards=true, .maxSpeed=64}, false);
    // chassis.moveToPoint(skills_30a.x, skills_30a.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    // arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    // pros::delay(3000);

    // chassis.moveToPoint(skills_30.x, skills_30.y, 1000, {.forwards=true, .maxSpeed=64}, false);
    // arm_controller.moveTo(Arm::position::SCORE_NEUTRAL,true);

    // //clamp_solenoid.extend();
    // chassis.moveToPoint(skills_31.x, skills_31.y, 1000, {.forwards=true, .minSpeed=24}, false);
    // chassis.moveToPoint(skills_32.x, skills_32.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    // //climb
    // chassis.moveToPose(skills_33.x, skills_33.y, skills_33.theta, 10000, {.forwards=false, .minSpeed=80}, false);
    // //chassis.turnToPoint(skills_33.x, skills_33.y, 1000, {.forwards=false}, false);
    // //dt_left.move(127);
    // //dt_right.move(127);
    // //pros::delay(20000);


}
