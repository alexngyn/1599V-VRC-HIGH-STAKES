#include "autonomous.h"
#include "arm.h"
#include "intake.h"
#include "lemlib/pose.hpp"
#include "setup.h" 

#define CLAMP_OFFSET -4
#define INTAKE_OFFSET 2
#define ALLIANCE_STAKE_OFFSET 15.5
#define NEUTRAL_STAKE_OFFSET 10

lemlib::Pose offsetPose(lemlib::Pose targetPose, float offset) {
    if (offset != 0) {
        targetPose.x -= offset * sin(lemlib::degToRad(targetPose.theta));
        targetPose.y -= offset * cos(lemlib::degToRad(targetPose.theta));
    }
    return {targetPose};
}

lemlib::Pose offsetPoint(lemlib::Pose targetPoint, lemlib::Pose prevPoint, float offset) {
    if (offset != 0) {
        targetPoint.theta = atan2(targetPoint.x - prevPoint.x, targetPoint.y - prevPoint.y);
        targetPoint.x -= offset * sin(lemlib::degToRad(targetPoint.theta));
        targetPoint.y -= offset * cos(lemlib::degToRad(targetPoint.theta));
    }
    return {targetPoint};
}

lemlib::Pose mirrorPose(lemlib::Pose pose) {
    return {-pose.x, pose.y, -pose.theta};
}

void moveStraight(float length, int timeout, lemlib::MoveToPointParams params) {
    if (chassis.isInMotion()) chassis.waitUntilDone();
    params.forwards = length > 0;
    lemlib::Pose pose = chassis.getPose();
    chassis.moveToPoint(pose.x + length * sin(lemlib::degToRad(pose.theta)),
                        pose.y + length * cos(lemlib::degToRad(pose.theta)), timeout, params);
}

//======================= pid tuning =======================

void pidtune() {
    // disable all other settings other than kd and kp
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    lemlib::Pose pose = offsetPoint({0, 48, NAN},{0,0,0},8);
    std::printf("%.f %.f %.f\n", pose.x, pose.y, pose.theta);
    // turn to face heading 90 with a very long timeout
    //chassis.swingToHeading(45, DriveSide::LEFT, 100000);
    //chassis.turnToHeading(45, 100000);
    // increase kp until the robot oscillates then add kd until it stops
    //chassis.moveToPoint(0, 24, 10000);
    // chassis.moveToPoint(-48, 0, 10000);
    //chassis.turnToHeading(270, 100000);
    // chassis.moveToPoint(-48, 0, 10000);

    //chassis.turnToPoint(elims_rush_pos_red_7.x, elims_rush_pos_red_7.y, 1000, {.forwards=true}, true); // turn to stake
    //intake_solenoid.retract();
    //chassis.moveToPoint(elims_rush_pos_red_7.x, elims_rush_pos_red_7.y, 1000, {.forwards = true}, false);
    //chassis.turnToHeading(elims_rush_pos_red_7.theta, 500);
    pros::delay(2000);
    std::printf("%.1f %.1f %.1f\n", pose.x, pose.y, pose.theta);
    pros::delay(50000);

    //chassisPrintPose();
    //chassis.turnToHeading(95, 100000);
    //robot::chassisPrintPose();
}

//======================= qualis pos 6p safe rush auton =======================

lemlib::Pose qual_safe_pos_blue_0 = {64,15,142}; 
lemlib::Pose qual_safe_pos_blue_1 = offsetPose(qual_safe_pos_blue_0, 8);
lemlib::Pose qual_safe_pos_blue_2 ={23.6,23.6, 60};
lemlib::Pose qual_safe_pos_blue_3 = {23.6, 47.25, NAN};
lemlib::Pose qual_safe_pos_blue_4 = {47, 0, NAN};

void qual_safe_pos_blue() {
    chassis.setPose(qual_safe_pos_blue_0.x, qual_safe_pos_blue_0.y, qual_safe_pos_blue_0.theta);
    clamp_solenoid.extend();

    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE);
    pros::delay(1000);

    chassis.moveToPoint(qual_safe_pos_blue_1.x, qual_safe_pos_blue_1.y, 3000, {.forwards = false}, false); //rush to mogo

    chassis.moveToPose(qual_safe_pos_blue_2.x, qual_safe_pos_blue_2.y, qual_safe_pos_blue_2.theta, 30000, {.forwards = false}, false); //rush to mogo
    clamp_solenoid.retract(); //clamp mogo

    intake_controller.set(Intake::IntakeState::INTAKING);

    chassis.moveToPoint(qual_safe_pos_blue_3.x, qual_safe_pos_blue_3.y, 3000, {.forwards = true, .maxSpeed=64}, false); //rush to mogo

    pros::delay(300);

    intake_solenoid.extend();

    chassis.moveToPoint(qual_safe_pos_blue_4.x, qual_safe_pos_blue_4.y, 8000, {.forwards = true, .maxSpeed=64}, false); //rush to mogo

    pros::delay(200);

    intake_solenoid.retract();

    pros::delay(20000);
}

//======================= qualis pos 9p goal rush AWP auton =======================

lemlib::Pose qual_rush_pos_red_0 = {-50.74,-58, 270}; 
lemlib::Pose qual_rush_pos_red_1 = {-32, -58.5, 270};
lemlib::Pose qual_rush_pos_red_2 = offsetPose({0, -47.24, 240}, -6);
lemlib::Pose qual_rush_pos_red_3 = offsetPose({-23.622, -47.244, 120}, 2);
lemlib::Pose qual_rush_pos_red_4 = offsetPose({-23.622-0, -23.622, 180}, -9);
lemlib::Pose qual_rush_pos_red_5 = offsetPose({-23.622+0, -23.622, 180}, -2);
lemlib::Pose qual_rush_pos_red_6 = offsetPoint({-47.244-1, -2, NAN}, qual_rush_pos_red_5, 0);  //51
lemlib::Pose qual_rush_pos_red_7 = offsetPose({-70, -2, 270}, 0);
lemlib::Pose qual_rush_pos_red_7a = offsetPose({-70, -2, 270}, 17); //58.5 11.5 //7

lemlib::Pose qual_rush_pos_red_8 = {-24, -24, NAN}; 
lemlib::Pose qual_rush_pos_red_9 = {-14, -14, NAN}; 

lemlib::Pose qual_rush_pos_red_10 = {-24, -47, 90}; 

void qual_rush_pos_red() {
    chassis.setPose(qual_rush_pos_red_0.x, qual_rush_pos_red_0.y, qual_rush_pos_red_0.theta);
    clamp_solenoid.extend();

    //goal rush

    chassis.moveToPoint(qual_rush_pos_red_1.x , qual_rush_pos_red_1.y, 10000, {.forwards = false, .minSpeed=127}, false); 
    chassis.moveToPose(qual_rush_pos_red_2.x, qual_rush_pos_red_2.y, qual_rush_pos_red_2.theta, 1000, {.forwards = false}, false);
    chassis.moveToPoint(qual_rush_pos_red_2.x, qual_rush_pos_red_2.y, 500, {.forwards = false}, false);

    pros::delay(100);

    clamp_solenoid.retract(); //clamp mogo
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(700);

    chassis.moveToPose(qual_rush_pos_red_3.x, qual_rush_pos_red_3.y, qual_rush_pos_red_3.theta, 1000, {.forwards = true}, true);
    intake_controller.hold(true);

    //second mogo
    
    chassis.turnToPoint(qual_rush_pos_red_4.x, qual_rush_pos_red_4.y, 1000, {.forwards=true, .direction=lemlib::AngularDirection::CW_CLOCKWISE,.maxSpeed=96}, false); // turn to ring
    clamp_solenoid.extend();
    chassis.turnToPoint(qual_rush_pos_red_5.x, qual_rush_pos_red_5.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CW_CLOCKWISE,.maxSpeed=96}, false); // turn to mogo
    chassis.moveToPoint(qual_rush_pos_red_4.x, qual_rush_pos_red_4.y, 1000, {.forwards = false,.maxSpeed=96}, false); // go to mogo
    chassis.moveToPoint(qual_rush_pos_red_5.x, qual_rush_pos_red_5.y, 1000, {.forwards = false}, false); // go to mogo
    clamp_solenoid.retract();
    pros::delay(50);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(500); // wait for allience

    //alicence stake
    //
    intake_solenoid.extend();

    chassis.turnToPoint(qual_rush_pos_red_6.x, qual_rush_pos_red_6.y, 1000, {}, false); // turn to stake
    chassis.moveToPoint(qual_rush_pos_red_6.x, qual_rush_pos_red_6.y, 1000, {.forwards = true,.maxSpeed=96}, true); // go to stake
    //chassis.turnToPoint(qual_rush_pos_red_7.x, qual_rush_pos_red_7.y, 1000, {.forwards=true,.maxSpeed=64}, true); // turn to stake
    chassis.turnToHeading(qual_rush_pos_red_7.theta, 500);
    intake_solenoid.retract();
    
    chassis.moveToPose(qual_rush_pos_red_7a.x, qual_rush_pos_red_7a.y,qual_rush_pos_red_7a.theta, 1000, {.forwards = true,.maxSpeed=96}, false);

    pros::delay(800);
    //
    // chassis.moveToPoint(qual_rush_pos_red_8.x, qual_rush_pos_red_8.y, 2000, {.forwards = false}, false); // go to ladder

    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.moveToPoint(qual_rush_pos_red_9.x, qual_rush_pos_red_9.y, 2000, {.forwards = false}, true); // go to ladder
    // arm_controller.moveTo(-290);//move arm up along the way
    //
    chassis.moveToPoint(qual_rush_pos_red_10.x, qual_rush_pos_red_10.y, 2000, {.forwards = false}, false); // go to ladder
    chassis.turnToHeading(qual_rush_pos_red_10.theta, 1000);
}

lemlib::Pose qual_rush_pos_blue_0 = mirrorPose(qual_rush_pos_red_0); 
lemlib::Pose qual_rush_pos_blue_1 = mirrorPose(qual_rush_pos_red_1);
lemlib::Pose qual_rush_pos_blue_2 = mirrorPose(qual_rush_pos_red_2);
lemlib::Pose qual_rush_pos_blue_3 = mirrorPose(qual_rush_pos_red_3);
lemlib::Pose qual_rush_pos_blue_4 = mirrorPose(qual_rush_pos_red_4);
lemlib::Pose qual_rush_pos_blue_5 = mirrorPose(qual_rush_pos_red_5);
lemlib::Pose qual_rush_pos_blue_6 = mirrorPose(qual_rush_pos_red_6);
lemlib::Pose qual_rush_pos_blue_7 = mirrorPose(qual_rush_pos_red_7);
lemlib::Pose qual_rush_pos_blue_7a = mirrorPose(qual_rush_pos_red_7a);
lemlib::Pose qual_rush_pos_blue_8 = mirrorPose(qual_rush_pos_red_8);
lemlib::Pose qual_rush_pos_blue_9 = mirrorPose(qual_rush_pos_red_9);
lemlib::Pose qual_rush_pos_blue_10 = mirrorPose(qual_rush_pos_red_10);

void qual_rush_pos_blue() {
    chassis.setPose(qual_rush_pos_blue_0.x, qual_rush_pos_blue_0.y, qual_rush_pos_blue_0.theta);
    clamp_solenoid.extend();

    //goal rush

    chassis.moveToPoint(qual_rush_pos_blue_1.x, qual_rush_pos_blue_1.y, 1000, {.forwards = false, .minSpeed=127}, false); //rush to mogo
    chassis.moveToPose(qual_rush_pos_blue_2.x, qual_rush_pos_blue_2.y, qual_rush_pos_blue_2.theta, 1000, {.forwards = false}, false); //rush to mogo
    chassis.moveToPoint(qual_rush_pos_blue_2.x, qual_rush_pos_blue_2.y, 500, {.forwards = false}, false); //rush to mogo

    pros::delay(100);

    clamp_solenoid.retract(); //clamp mogo
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(700);

    chassis.moveToPose(qual_rush_pos_blue_3.x, qual_rush_pos_blue_3.y, qual_rush_pos_blue_3.theta, 1000, {.forwards = true}, true);
    intake_controller.hold(true);

    //second mogo

    chassis.turnToPoint(qual_rush_pos_blue_4.x, qual_rush_pos_blue_4.y, 1000, {.forwards=true, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=96}, false); // turn to ring
    clamp_solenoid.extend();
    chassis.turnToPoint(qual_rush_pos_blue_5.x, qual_rush_pos_blue_5.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=96}, false); // turn to mogo
    chassis.moveToPoint(qual_rush_pos_blue_4.x, qual_rush_pos_blue_4.y, 1000, {.forwards = false,.maxSpeed=96}, false); // go to mogo
    chassis.moveToPoint(qual_rush_pos_blue_5.x, qual_rush_pos_blue_5.y, 1000, {.forwards = false}, false); // go to mogo
    clamp_solenoid.retract();
    pros::delay(50);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(500); // wait for allience

    //alliance stake

    intake_solenoid.extend();

    chassis.turnToPoint(qual_rush_pos_blue_6.x, qual_rush_pos_blue_6.y, 1000, {}, false); // turn to stake
    chassis.moveToPoint(qual_rush_pos_blue_6.x, qual_rush_pos_blue_6.y, 1000, {.forwards = true,.maxSpeed=96}, true); // go to stake
    
    chassis.turnToHeading(qual_rush_pos_blue_7.theta, 500);
    intake_solenoid.retract();

    chassis.moveToPose(qual_rush_pos_blue_7a.x, qual_rush_pos_blue_7a.y,qual_rush_pos_blue_7a.theta, 1000, {.forwards = true,.maxSpeed=96}, false);

    pros::delay(800);

    // chassis.moveToPoint(qual_rush_pos_blue_8.x, qual_rush_pos_blue_8.y, 2000, {.forwards = false}, false); // go to ladder

    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.moveToPoint(qual_rush_pos_blue_9.x, qual_rush_pos_blue_9.y, 2000, {.forwards = false}, true); // go to ladder
    // arm_controller.moveTo(-290);//move arm up along the way

    chassis.moveToPoint(qual_rush_pos_blue_10.x, qual_rush_pos_blue_10.y, 2000, {.forwards = false}, false); // go to ladder
    chassis.turnToHeading(qual_rush_pos_blue_10.theta, 1000);
}

//======================= elims pos 9p goal rush auton =======================

lemlib::Pose elims_rush_pos_red_0 = {-50.74,-58, 270}; 
lemlib::Pose elims_rush_pos_red_1 = {-32, -58.5, 270};
lemlib::Pose elims_rush_pos_red_2 = offsetPose({0, -47.24, 240}, -6);
lemlib::Pose elims_rush_pos_red_3 = offsetPose({-23.622, -47.244, 120}, 0);
lemlib::Pose elims_rush_pos_red_4 = offsetPose({-23.622-0, -23.622, 180}, -9);
lemlib::Pose elims_rush_pos_red_5 = offsetPose({-23.622+0, -23.622, 180}, -7); //x+2
//lemlib::Pose elims_rush_pos_red_6 = offsetPoint({-47.244, -5, NAN}, elims_rush_pos_red_5, 0);  //51
lemlib::Pose elims_rush_pos_red_6 = offsetPose({-72, -7, -63}, 12);  //16 -61.97 degs
lemlib::Pose elims_rush_pos_red_6a = offsetPose({-72, -7, -63}, 32);
lemlib::Pose elims_rush_pos_red_7 = {-54, -0, NAN};
lemlib::Pose elims_rush_pos_red_7a = offsetPose({-70,0, 270}, 13.5); //58.5 11.5


void elims_rush_pos_red() {
    chassis.setPose(elims_rush_pos_red_0.x, elims_rush_pos_red_0.y, elims_rush_pos_red_0.theta);
    clamp_solenoid.extend();

    //goal rush

    chassis.moveToPoint(elims_rush_pos_red_1.x, elims_rush_pos_red_1.y, 1000, {.forwards = false,.minSpeed=127}, false); //rush to mogo
    chassis.moveToPose(elims_rush_pos_red_2.x, elims_rush_pos_red_2.y, elims_rush_pos_red_2.theta, 1000, {.forwards = false}, false); //rush to mogo
    chassis.moveToPoint(elims_rush_pos_red_2.x, elims_rush_pos_red_2.y, 500, {.forwards = false}, false); //rush to mogo

    pros::delay(100);

    clamp_solenoid.retract(); //clamp mogo
    pros::delay(50);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(400);

    chassis.moveToPose(elims_rush_pos_red_3.x, elims_rush_pos_red_3.y, elims_rush_pos_red_3.theta, 1000, {.forwards = true}, false);
    intake_controller.hold(true);
    arm_controller.moveTo(Arm::position::INTAKE);

    //second mogo

    chassis.turnToPoint(elims_rush_pos_red_4.x, elims_rush_pos_red_4.y, 1000, {.forwards=true, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=64}, false); // turn to ring
    clamp_solenoid.extend();
    chassis.turnToPoint(elims_rush_pos_red_5.x, elims_rush_pos_red_5.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=64}, false); // turn to mogo
    chassis.moveToPoint(elims_rush_pos_red_4.x, elims_rush_pos_red_4.y, 1000, {.forwards = false,.maxSpeed=64}, false); // go to mogo
    chassis.moveToPoint(elims_rush_pos_red_5.x, elims_rush_pos_red_5.y, 1000, {.forwards = false,.maxSpeed=64}, false); // go to mogo
    clamp_solenoid.retract();
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(200); // wait for allience

    chassis.turnToPoint(elims_rush_pos_red_6.x, elims_rush_pos_red_6.y, 1000, {.forwards=true}, true); // turn to stake
    chassis.moveToPoint(elims_rush_pos_red_6.x, elims_rush_pos_red_6.y, 4000, {.forwards = true,.maxSpeed=64}, false);

    //pros::delay(500);

    intake_controller.set(Intake::IntakeState::OUTTAKE);
    pros::delay(200);
    intake_controller.set(Intake::IntakeState::STOPPED);
    pros::delay(50);

    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    pros::delay(600);
    chassis.moveToPoint(elims_rush_pos_red_6a.x, elims_rush_pos_red_6a.y, 2000, {.forwards = false}, false); // go to ladder
    intake_controller.set(Intake::IntakeState::INTAKING);
    intake_solenoid.extend();

    chassis.turnToPoint(elims_rush_pos_red_7.x, elims_rush_pos_red_7.y, 1000, {.forwards=true}, true); // turn to stake
    chassis.moveToPoint(elims_rush_pos_red_7.x, elims_rush_pos_red_7.y, 2000, {.forwards = true,.maxSpeed=48}, false); // go to ladder
    arm_controller.moveTo(Arm::position::RETRACT, true);

    pros::delay(200);
    intake_solenoid.retract();

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    dt_left.move(-48);
    dt_right.move(-48);
    pros::delay(150);
    dt_left.move(0);
    dt_right.move(0);

    pros::delay(5000);

}

lemlib::Pose elims_rush_pos_blue_0 = mirrorPose(elims_rush_pos_red_0); 
lemlib::Pose elims_rush_pos_blue_1 = mirrorPose(elims_rush_pos_red_1);
lemlib::Pose elims_rush_pos_blue_2 = mirrorPose(elims_rush_pos_red_2);
lemlib::Pose elims_rush_pos_blue_3 = mirrorPose(elims_rush_pos_red_3);
lemlib::Pose elims_rush_pos_blue_4 = mirrorPose(elims_rush_pos_red_4);
lemlib::Pose elims_rush_pos_blue_5 = mirrorPose(elims_rush_pos_red_5);
lemlib::Pose elims_rush_pos_blue_6 = mirrorPose(elims_rush_pos_red_6);
lemlib::Pose elims_rush_pos_blue_6a = mirrorPose(elims_rush_pos_red_6a);
lemlib::Pose elims_rush_pos_blue_7 = mirrorPose(elims_rush_pos_red_7);
lemlib::Pose elims_rush_pos_blue_7a = mirrorPose(elims_rush_pos_red_7a);

void elims_rush_pos_blue() {
    chassis.setPose(elims_rush_pos_blue_0.x, elims_rush_pos_blue_0.y, elims_rush_pos_blue_0.theta);
    clamp_solenoid.extend();

    //goal rush

    chassis.moveToPoint(elims_rush_pos_blue_1.x, elims_rush_pos_blue_1.y, 1000, {.forwards = false,.minSpeed=127}, false); //rush to mogo
    chassis.moveToPose(elims_rush_pos_blue_2.x, elims_rush_pos_blue_2.y, elims_rush_pos_blue_2.theta, 1000, {.forwards = false}, false); //rush to mogo
    chassis.moveToPoint(elims_rush_pos_blue_2.x, elims_rush_pos_blue_2.y, 500, {.forwards = false}, false); //rush to mogo

    pros::delay(100);

    clamp_solenoid.retract(); //clamp mogo
    pros::delay(50);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(400);

    chassis.moveToPose(elims_rush_pos_blue_3.x, elims_rush_pos_blue_3.y, elims_rush_pos_blue_3.theta, 1000, {.forwards = true}, true);
    intake_controller.hold(true);
    arm_controller.moveTo(Arm::position::INTAKE);

    //second mogo

    chassis.turnToPoint(elims_rush_pos_blue_4.x, elims_rush_pos_blue_4.y, 1000, {.forwards=true, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=64}, false); // turn to ring
    clamp_solenoid.extend();
    chassis.turnToPoint(elims_rush_pos_blue_5.x, elims_rush_pos_blue_5.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=64}, false); // turn to mogo
    chassis.moveToPoint(elims_rush_pos_blue_4.x, elims_rush_pos_blue_4.y, 1000, {.forwards = false,.maxSpeed=64}, false); // go to mogo
    chassis.moveToPoint(elims_rush_pos_blue_5.x, elims_rush_pos_blue_5.y, 1000, {.forwards = false,.maxSpeed=64}, false); // go to mogo
    clamp_solenoid.retract();
    intake_controller.set(Intake::IntakeState::OUTTAKE);
    pros::delay(100);
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(200); // wait for allience

    chassis.turnToPoint(elims_rush_pos_blue_6.x, elims_rush_pos_blue_6.y, 1000, {.forwards=true, .maxSpeed=64}, true); // turn to stake
    chassis.moveToPoint(elims_rush_pos_blue_6.x, elims_rush_pos_blue_6.y, 4000, {.forwards = true,.maxSpeed=64}, false);
    
    //pros::delay(500);

    intake_controller.set(Intake::IntakeState::OUTTAKE);
    pros::delay(200);
    intake_controller.set(Intake::IntakeState::STOPPED);
    pros::delay(50);
    
    arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    pros::delay(600);
    chassis.moveToPoint(elims_rush_pos_blue_6a.x, elims_rush_pos_blue_6a.y, 2000, {.forwards = false}, false); // go to ladder
    intake_controller.set(Intake::IntakeState::INTAKING);
    intake_solenoid.extend();

    chassis.turnToPoint(elims_rush_pos_blue_7.x, elims_rush_pos_blue_7.y, 1000, {.forwards=true}, true); // turn to stake
    chassis.moveToPoint(elims_rush_pos_blue_7.x, elims_rush_pos_blue_7.y, 2000, {.forwards = true,.maxSpeed=48}, false); // go to ladder

    pros::delay(200);
    intake_solenoid.retract();

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    dt_left.move(-48);
    dt_right.move(-48);
    pros::delay(150);
    dt_left.move(0);
    dt_right.move(0);

    pros::delay(5000);
}

//======================= skills autons =======================

lemlib::Pose skills_0 = {-62.994, 0, 90}; 
lemlib::Pose skills_1a = {-47.244, 0, 90};
lemlib::Pose skills_1 = offsetPose({-47.244, -23.622, 0}, -2);
lemlib::Pose skills_1b = {NAN,NAN, 60};
lemlib::Pose skills_2 = {-23.622, -23.622, 135};
lemlib::Pose skills_3 = {-23.622, -47.24-4, 200};
lemlib::Pose skills_4 = offsetPose({23.622+0, -47.244-4, 90},0);
lemlib::Pose skills_5 = offsetPose({-0, -62, 180},0);//1
lemlib::Pose skills_5a = offsetPose({-0, -70, 180},0);//1
lemlib::Pose skills_5b = offsetPose({-0, -65, 180},0);//1
lemlib::Pose skills_5c = offsetPose({-0, -57, 180},0);//1
lemlib::Pose skills_6 = offsetPose({-47.244, -47.244-6, 270},0); 
lemlib::Pose skills_7 = offsetPose({-56.055-6, -47.244-6, 270},0);
lemlib::Pose skills_8 = offsetPoint({-47.244, -59.055+0, 135},skills_7,4);
lemlib::Pose skills_9 = offsetPoint({-61, -63, 45},skills_8,2);

lemlib::Pose skills_10 = offsetPoint({-47-3, -47, 45},skills_9,0);
//lemlib::Pose skills_10 = offsetPoint({0, 0, 45},skills_9,0);
//lemlib::Pose skills_11a = {-38,23.5,NAN};
lemlib::Pose skills_11 = offsetPose({-47.244-3, 23.622-2, 180},0);
lemlib::Pose skills_11a = offsetPose({NAN,NAN,120},0);
lemlib::Pose skills_12 = {-23.622-0, 23.622-2, 17.5};
lemlib::Pose skills_13 = {-23.622+0, 47.244-4, 310};
lemlib::Pose skills_14 = offsetPose({-59.055-3, 47.244-5, 270},0);
lemlib::Pose skills_15 = offsetPoint({-47.244-0, 59.055-0, 45},skills_14,0);
lemlib::Pose skills_16 = offsetPoint({-59, 60, 135},skills_15,2);
lemlib::Pose skills_16a = {50, 50, 135};

lemlib::Pose skills_17 = offsetPoint({-5, 55, 180},skills_16,0);
lemlib::Pose skills_17a = offsetPoint({-5, 70, 180},skills_17,0);
lemlib::Pose skills_17b = offsetPoint({-5, 60, 180},skills_17,0);
lemlib::Pose skills_17c = offsetPoint({-5, 53, 180},skills_17,0);
// lemlib::Pose skills_18 = offsetPoint({23.622, 47.244, 110},skills_17,2);
//lemlib::Pose  skills_19a = offsetPose({0, 59.055, 0},2);
//lemlib::Pose skills_19 = offsetPose({0, 70, 0},10);
lemlib::Pose skills_20 = {23.622, 23.622, 140};


lemlib::Pose skills_21 = offsetPose({47.035, 0, 300},8);
lemlib::Pose skills_22 = offsetPose({23.622, -23.622, 200},2);
lemlib::Pose skills_23 = offsetPose({30, -47.244-3, 90},0);
lemlib::Pose skills_23a = offsetPose({42, -47.244-3, 90},0);
lemlib::Pose skills_24 = offsetPoint({42, -59.055-2, 90},skills_23,0);
lemlib::Pose skills_25 = {46, 8, 0};
lemlib::Pose skills_23b = offsetPose({30, -47.244-3, 90},0);
lemlib::Pose skills_26 = offsetPose({23.622, 47.244-2, 330},0);
lemlib::Pose skills_26a = offsetPose({47.244, 59.055, 330},-4);
lemlib::Pose skills_27 = offsetPoint({49, 64, 225},skills_26,0);

lemlib::Pose skills_28 = {59.055, 47.244, 210};
lemlib::Pose skills_29 = {38, 14, 215};
lemlib::Pose skills_30 = offsetPose({70-15, -10, 90},0);
lemlib::Pose skills_30a = offsetPose({70-10, -10, 90},0);
lemlib::Pose skills_30b = offsetPose({70-20, -10, 90},0);

lemlib::Pose skills_31 = offsetPoint({59.055, -23.622, NAN},skills_30b, -4);
lemlib::Pose skills_32 = offsetPoint({64, -55, 135},skills_31,0);
lemlib::Pose skills_33 = {15, -21, 135}; //18,-18

void skills() {
    sideColor = color::red;
    intake_controller.setState(Intake::SortState::OFF);

    chassis.setPose(skills_0.x, skills_0.y, skills_0.theta);
    
    intake_controller.set(Intake::IntakeState::INTAKING);
    pros::delay(500); 

    clamp_solenoid.extend();

    // goal 1 quadrent 3 + right wall stake

    chassis.moveToPoint(skills_1a.x, skills_1a.y, 2000, {.forwards=true, .maxSpeed=96}, true);
    intake_controller.set(Intake::IntakeState::OUTTAKE);
    chassis.moveToPose(skills_1.x, skills_1.y, skills_1.theta, 2000, {.forwards=false, .maxSpeed=96}, false);

    //pros::delay(100);

    clamp_solenoid.retract();

    chassis.turnToHeading(skills_1b.theta, 800);
    intake_controller.set(Intake::IntakeState::INTAKING);
    // intake_controller.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(skills_2.x, skills_2.y, skills_2.theta, 2000, {.forwards=true, .maxSpeed=96, .minSpeed=36}, false);
    chassis.moveToPoint(skills_3.x, skills_3.y, 2000, {.forwards=true, .maxSpeed=96}, false);

    

    chassis.turnToHeading(skills_4.theta, 1000);
    chassis.moveToPoint(skills_4.x, skills_4.y, 2000, {.forwards=true, .maxSpeed=96}, false);

    chassis.moveToPoint(skills_5.x, skills_5.y, 2000, {.forwards=true, .maxSpeed=96}, true);
    pros::delay(1000);


    arm_controller.moveTo(Arm::position::INTAKE, true);
    pros::delay(1500);
    //intake_controller.holdldb(true,1000);
    chassis.turnToPoint(skills_5a.x, skills_5a.y, 1000, {.forwards=true}, false);
    // arm_controller.moveTo(Arm::position::INTAKE, true);
    chassis.moveToPoint(skills_5b.x, skills_5b.y, 1000, {.forwards=true, .maxSpeed=96}, false);
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, true, 1000);
    intake_controller.set(Intake::IntakeState::STOPPED);
    pros::delay(600);
    
    chassis.moveToPoint(skills_5c.x, skills_5c.y, 2000, {.forwards=false, .maxSpeed=96}, true);
    pros::delay(100);
    arm_controller.moveTo(Arm::position::RETRACT);

    intake_controller.set(Intake::IntakeState::INTAKING);

    chassis.moveToPose(skills_6.x , skills_6.y, skills_6.theta, 3000, {.forwards=true, .maxSpeed=96,.minSpeed=48}, true);
    // pros::delay(1000);
    chassis.moveToPoint(skills_7.x, skills_7.y, 2000, {.forwards=true, .maxSpeed=96}, false);
    chassis.moveToPoint(skills_8.x, skills_8.y, 2000, {.forwards=true, .maxSpeed=96}, false);

    chassis.moveToPoint(skills_9.x, skills_9.y, 2000, {.forwards=false, .maxSpeed=96}, false);
    clamp_solenoid.extend();
    intake_controller.set(Intake::IntakeState::OUTTAKE);
    pros::delay(300);
    intake_controller.set(Intake::IntakeState::STOPPED);

    

    // //goal 2 quadrent 2 + left wall stake
    chassis.moveToPoint(skills_10.x, skills_10.y, 3000, {.forwards=true, .maxSpeed=96}, true);
    // chassis.waitUntil(5);
    // intake_controller.hold(true,1000);

    // chassis.moveToPoint(skills_11.x, skills_11.y, 3000, {.forwards=true, .maxSpeed=96, .minSpeed=48, .earlyExitRange=24}, true);
    chassis.moveToPose(skills_11.x, skills_11.y, skills_11.theta, 3000, {.forwards=false, .maxSpeed=64}, false);
    clamp_solenoid.retract();
    pros::delay(200);

    chassis.turnToHeading(skills_11a.theta, 1000);

    intake_controller.set(Intake::IntakeState::INTAKING);
    //chassis.moveToPose(skills_12.x, skills_12.y, skills_12.theta, 2000, {.forwards=true, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_12.x, skills_12.y, 2000, {.forwards=true, .maxSpeed=96,.minSpeed=0}, false);
    chassis.moveToPoint(skills_13.x, skills_13.y, 2000, {.forwards=true, .maxSpeed=96,.minSpeed=0}, false);
    chassis.moveToPoint(skills_14.x, skills_14.y, 2000, {.forwards=true, .maxSpeed=96}, false);
    chassis.moveToPoint(skills_15.x, skills_15.y, 2000, {.forwards=true, .maxSpeed=96}, false);

    chassis.moveToPoint(skills_16.x, skills_16.y, 2000, {.forwards=false,.minSpeed=48}, false);
    intake_controller.set(Intake::IntakeState::OUTTAKE);
    clamp_solenoid.extend();
    pros::delay(300);

    chassis.moveToPoint(skills_16a.x, skills_16a.y, 100, {.forwards=true,.maxSpeed=96}, false);
    intake_controller.set(Intake::IntakeState::INTAKING);
    

    // left wall stake
    chassis.moveToPoint(skills_17.x, skills_17.y, 3000, {.forwards=true, .maxSpeed=96}, true);
    chassis.waitUntil(10);
    arm_controller.moveTo(Arm::position::INTAKE);
    chassis.turnToPoint(skills_17a.x, skills_17a.y, 1000, {.forwards=true}, false);
    chassis.moveToPoint(skills_17b.x, skills_17b.y, 1000, {.forwards=true, .maxSpeed=96}, false);
    arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, true);
    intake_controller.set(Intake::IntakeState::STOPPED);
    pros::delay(1000);
    

    chassis.moveToPoint(skills_17c.x, skills_17c.y, 1000, {.forwards=false, .maxSpeed=96}, false);

    //chassis.moveToPoint(skills_18.x, skills_18.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    //intake_controller.set(Intake::IntakeState::INTAKING);
    //intake_controller.holdldb(true, 1000);
    //intake_controller.hold();
    // chassis.moveToPoint(skills_18.x, skills_18.y, 1000, {.forwards=true}, true);
    //chassis.waitUntil(8);
    //intake_controller.set(Intake::IntakeState::INTAKING);
    //intake_controller.waitUntilDone();

    //chassis.moveToPose(skills_19a.x, skills_19a.y, skills_19a.theta, 2000, {.forwards=true, .maxSpeed=64}, false);
    // chassis.moveToPoint(skills_19.x, skills_19.y, 1000, {.forwards=true, .maxSpeed=64}, false);

    //arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, true);
    // pros::delay(1000);

    
    
    //next mogo

    chassis.moveToPoint(skills_20.x, skills_20.y, 2000, {.forwards=true, .maxSpeed=96}, true);
    arm_controller.moveTo(Arm::position::RETRACT);
    pros::delay(300);
    intake_controller.hold();
    chassis.moveToPose(skills_21.x, skills_21.y, skills_21.theta, 3000, {.forwards=false, .maxSpeed=96}, false);
    clamp_solenoid.retract();
    intake_controller.set(Intake::IntakeState::INTAKING);

    /*

    chassis.moveToPoint(skills_22.x, skills_22.y, 3000, {.forwards=true,.maxSpeed=96}, false);

    chassis.moveToPose(skills_23.x, skills_23.y, skills_23.theta, 2000, {.forwards=true, .maxSpeed=96}, false);
    chassis.moveToPoint(skills_23a.x, skills_23a.y, 2000, {.forwards=true, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_23.x, skills_23.y, 2000, {.forwards=false, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_24.x, skills_24.y, 2000, {.forwards=true, .maxSpeed=64}, false);
    chassis.moveToPoint(skills_23b.x, skills_23.y, 2000, {.forwards=false, .maxSpeed=64}, false);

    chassis.moveToPoint(skills_25.x, skills_25.y, 3000, {.forwards=true,.maxSpeed=96}, false);
    chassis.moveToPoint(skills_26.x, skills_26.y, 2000, {.forwards=true, .maxSpeed=64}, false);

    // doinker_solenoid.toggle();

    chassis.moveToPoint(skills_26a.x, skills_26a.y, 2000, {.forwards=true, .maxSpeed=96}, false);
    intake_controller.hold();
    
    chassis.turnToPoint(skills_27.x, skills_27.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    chassis.moveToPoint(skills_27.x, skills_27.y, 2000, {.forwards=false, .maxSpeed=96}, false);
    clamp_solenoid.extend();
    // doinker_solenoid.toggle();

    //goal 3 quadrent 1/4 + allience + goal4

    // intake_controller.hold();

    // arm_controller.moveTo(Arm::position::INTAKE);
    // intake_controller.holdldb(true, 1000);
    
    // //intake_controller.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(skills_28.x, skills_28.y, 1000, {.forwards=true, .maxSpeed=96}, false);
    chassis.moveToPoint(skills_29.x, skills_29.y, 1000, {.forwards=true, .maxSpeed=96}, false);
    
    // chassis.moveToPose(skills_30.x, skills_30.y, skills_30.theta, 2000, {.forwards=false, .maxSpeed=96}, false);
    // chassis.moveToPoint(skills_30a.x, skills_30a.y, 1000, {.forwards=false, .maxSpeed=96}, false);
    // intake_controller.set(Intake::IntakeState::INTAKING);
    // pros::delay(1000);

    // chassis.moveToPoint(skills_30b.x, skills_30b.y, 1000, {.forwards=true, .maxSpeed=96}, false);

    // arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
    // pros::delay(3000);

    // chassis.moveToPoint(skills_30.x, skills_30.y, 1000, {.forwards=true, .maxSpeed=64}, false);
    // arm_controller.moveTo(Arm::position::SCORE_NEUTRAL,true);

    clamp_solenoid.extend();
    chassis.moveToPoint(skills_31.x, skills_31.y, 1000, {.forwards=true, .maxSpeed=96}, false);
    chassis.moveToPoint(skills_32.x, skills_32.y, 1000, {.forwards=true, .maxSpeed=127}, false);

    // // //climb
    // arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, true);
    // chassis.moveToPose(skills_33.x, skills_33.y, skills_33.theta, 10000, {.forwards=false, .maxSpeed=127}, false);
    // // //chassis.turnToPoint(skills_33.x, skills_33.y, 1000, {.forwards=false}, false);
    // dt_left.move(127);
    // dt_right.move(127);
    // pros::delay(500);
    // dt_left.move(0);
    // dt_right.move(0);
    */
}
