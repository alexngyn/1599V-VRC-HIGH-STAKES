// #include "autonomous.h"
// #include "lemlib/chassis/chassis.hpp"
// #include "pros/motors.h"
// #include "pros/rtos.hpp"
// #include "setup.h" 
// #include <cmath>

// #define CLAMP_OFFSET -6 // -9
// #define INTAKE_OFFSET 4 //6
// #define ALLIANCE_STAKE_OFFSET 16
// #define NEUTRAL_STAKE_OFFSET 10

// lemlib::Pose offsetPose(lemlib::Pose targetPose, float offset) {
//     targetPose.x -= (offset * cos((-targetPose.theta + 90)*3.14159/180));
//     targetPose.y -= (offset * sin((-targetPose.theta + 90)*3.14159/180));
//     return {targetPose};
// }

// lemlib::Pose offsetPoint(lemlib::Pose targetPoint, lemlib::Pose prevPoint, float offset) {
//     targetPoint.theta = atan2(targetPoint.y - prevPoint.y, targetPoint.x - prevPoint.x);
//     targetPoint.x -= (offset * cos((-targetPoint.theta + 90))*3.14159/180);
//     targetPoint.y -= (offset * sin((-targetPoint.theta + 90))*3.14159/180);
//     return {targetPoint};
// }

// lemlib::Pose mirrorPose(lemlib::Pose pose) {
//     return {-pose.x, pose.y, -pose.theta};
// }

// // void stakealign() {
// //     while (distance_sensor.get() > 10) { dt_left.move(30); dt_right.move(30); };
// //     dt_left.move(0); dt_right.move(0);
// // }

// //======================= pid tuning =======================

// void pidtune() {
//     // disable all other settings other than kd and kp
//     // set position to x:0, y:0, heading:0
//     chassis.setPose(0, 0, 0);
//     // turn to face heading 90 with a very long timeout
//     //chassis.swingToHeading(45, DriveSide::LEFT, 100000);
//     //chassis.turnToHeading(45, 100000);
//     // increase kp until the robot oscillates then add kd until it stops
//     //chassis.moveToPoint(0, 24, 10000);
//     // chassis.moveToPoint(-48, 0, 10000);
//     chassis.turnToHeading(270, 100000);
//     // chassis.moveToPoint(-48, 0, 10000);

//     pros::delay(5000);

//     //chassisPrintPose();
//     //chassis.turnToHeading(95, 100000);
//     //robot::chassisPrintPose();
// }

// //======================= quali neg 10p auton =======================

// // lemlib::Pose qual_neg_0 = {-52.5,-8.5, 300}; 
// // lemlib::Pose qual_neg_1 = {-60, -4.5, 300};
// // lemlib::Pose qual_neg_2 = {-26, -20, 300};
// // lemlib::Pose qual_neg_3 = {-30, -38, 160};
// // lemlib::Pose qual_neg_4 = {-42, -5, 330};
// // lemlib::Pose qual_neg_5 = {-28, -2, 90};

// // void qual_neg() {
// //  // priority: do last
// // }

// //======================= quali pos 9p autons =======================

// lemlib::Pose qual_pos_red_0 = {-50.74,-58, 270}; 
// lemlib::Pose qual_pos_red_1 = {-32, -58.5, 270};
// lemlib::Pose qual_pos_red_2 = offsetPose({0, -47.24, 240}, -7);
// lemlib::Pose qual_pos_red_3 = offsetPose({-23.622, -47.244, 120}, -2);
// lemlib::Pose qual_pos_red_4 = offsetPoint({-23.622, -47.244, 0}, qual_pos_red_3, 2);
// lemlib::Pose qual_pos_red_5a = offsetPose({-23.622, -23.622, 180}, -9);
// lemlib::Pose qual_pos_red_5 = offsetPose({-23.622, -23.622, 180}, -4);
// lemlib::Pose qual_pos_red_6 = offsetPoint({-46, 0, NAN}, qual_pos_red_5, 4); 
// lemlib::Pose qual_pos_red_6a = offsetPose(qual_pos_red_6, -6);
// lemlib::Pose qual_pos_red_6b = offsetPose(qual_pos_red_6, -3);
// lemlib::Pose qual_pos_red_7 = offsetPose({-70, qual_pos_red_6.y, 270}, 10); 
// //lemlib::Pose qual_pos_red_8 = {-48, 0, 270}; 
// lemlib::Pose qual_pos_red_8 = {-24, -24, NAN}; 

// void qual_pos_red() {
//     chassis.setPose(qual_pos_red_0.x, qual_pos_red_0.y, qual_pos_red_0.theta); // setup position

//     clamp_solenoid.extend();

//     //goal rush
//     // chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 2000, {.forwards = false}, false); //rush to mogo
//     // chassis.moveToPoint(qual_pos_red_2.x, qual_pos_red_2.y, 2000, {.forwards = false}, false); //rush to mogo

//     chassis.moveToPoint(qual_pos_red_1.x , qual_pos_red_1.y, 10000, {.forwards = false}, false);  // rush mogo
//     chassis.moveToPose(qual_pos_red_2.x, qual_pos_red_2.y, qual_pos_red_2.theta, 1000, {.forwards = false}, false);  // rush mogo
//     chassis.moveToPoint(qual_pos_red_2.x, qual_pos_red_2.y, 500, {.forwards = false}, false);  // rush mogo
//     //printf("%.1f,%.1f,%.1f\n", qual_pos_red_2.x, qual_pos_red_2.y, qual_pos_red_2.theta);

//     pros::delay(500);

//     clamp_solenoid.retract(); //clamp mogo
//     pros::delay(50);
//     intake_controller.set(Intake::IntakeState::INTAKING);
//     pros::delay(500);
//     intake_controller.hold(true);

//     // chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 1000, {.forwards = true}, false);
//     // chassis.moveToPoint(qual_pos_red_3.x , qual_pos_red_3.y, 1000, {.forwards = true}, false); 

//     chassis.moveToPose(qual_pos_red_3.x, qual_pos_red_3.y, qual_pos_red_3.theta, 1000, {.forwards = true}, false); // take first ring

//     //second mogo

//     chassis.turnToPoint(qual_pos_red_5a.x, qual_pos_red_5a.y, 1000, {.forwards=true}, false); // turn to relse mogo
//     clamp_solenoid.extend();
//     // chassis.moveToPoint(qual_pos_red_4.x, qual_pos_red_4.y, 1000, {.forwards = true}, false); // go to ring
    
//     chassis.turnToPoint(qual_pos_red_5a.x, qual_pos_red_5a.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CW_CLOCKWISE}, false); // turn 2nd mogo
//     chassis.turnToHeading(qual_pos_red_5a.theta, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE}, false); // turn 2nd mogo
//     chassis.moveToPose(qual_pos_red_5a.x, qual_pos_red_5a.y, qual_pos_red_5a.theta, 1000, {.forwards = false}, false); // go to mogo
//     chassis.moveToPoint(qual_pos_red_5.x, qual_pos_red_5.y, 1000, {.forwards = false}, false); // go to mogo
//     clamp_solenoid.retract();
//     pros::delay(100);
//     intake_controller.set(Intake::IntakeState::INTAKING);
//     pros::delay(200);

//     //alicence stake
//     chassis.turnToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {}, false); // turn to stack
//     chassis.moveToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {.forwards = true}, true); // go to stack
//     //arm_controller.moveTo(Arm::position::INTAKE);
//     intake_solenoid.extend();
//     intake_controller.holdldb(true);
//     chassis.waitUntilDone();
//     intake_solenoid.retract();
//     //chassis.moveToPoint(qual_pos_red_6a.x, qual_pos_red_6a.y, 1000, {.forwards = false}, false); // go to stack
//     //chassis.moveToPoint(qual_pos_red_6b.x, qual_pos_red_6b.y, 1000, {.forwards = true}, false); // go to stack
//     intake_controller.waitUntilDone();
//     chassis.moveToPose(qual_pos_red_7.x, qual_pos_red_7.y, qual_pos_red_7.theta, 1000, {.forwards = true}, false); // go to stake

//     arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);

//     pros::delay(800);

//     arm_controller.moveTo(Arm::position::RETRACT, true);

//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
//     chassis.moveToPoint(qual_pos_red_8.x, qual_pos_red_8.y, 1000, {.forwards = true}, false); // go to ladder
// }

// lemlib::Pose qual_pos_blue_0 = mirrorPose(qual_pos_red_0); 
// lemlib::Pose qual_pos_blue_1 = mirrorPose(qual_pos_red_1);
// lemlib::Pose qual_pos_blue_2 = mirrorPose(qual_pos_red_2);
// lemlib::Pose qual_pos_blue_3 = mirrorPose(qual_pos_red_3);
// lemlib::Pose qual_pos_blue_4 = mirrorPose(qual_pos_red_4);
// lemlib::Pose qual_pos_blue_5 = mirrorPose(qual_pos_red_5);
// lemlib::Pose qual_pos_blue_5a = mirrorPose(qual_pos_red_5a);
// lemlib::Pose qual_pos_blue_6 = mirrorPose(qual_pos_red_6);
// lemlib::Pose qual_pos_blue_6a = mirrorPose(qual_pos_red_6a);
// lemlib::Pose qual_pos_blue_6b = mirrorPose(qual_pos_red_6b);
// lemlib::Pose qual_pos_blue_7 = mirrorPose(qual_pos_red_7);
// lemlib::Pose qual_pos_blue_8 = mirrorPose(qual_pos_red_8);

// void qual_pos_blue() {
//     chassis.setPose(qual_pos_blue_0.x, qual_pos_blue_0.y, qual_pos_blue_0.theta); // setup position

//     //goal rush
//     chassis.moveToPoint(qual_pos_blue_1.x , qual_pos_blue_1.y, 10000, {.forwards = false}, false);  // rush mogo
//     chassis.moveToPose(qual_pos_blue_2.x, qual_pos_blue_2.y, qual_pos_blue_2.theta, 1000, {.forwards = false}, false);  // rush mogo
//     chassis.moveToPoint(qual_pos_blue_2.x, qual_pos_blue_2.y, 500, {.forwards = false}, false);  // rush mogo
    
//     pros::delay(500);
    
//     clamp_solenoid.retract(); //clamp mogo
//     pros::delay(50);
//     intake_controller.set(Intake::IntakeState::INTAKING);
//     pros::delay(1000);
//     intake_controller.hold(true);
    
//     chassis.moveToPose(qual_pos_blue_3.x, qual_pos_blue_3.y, qual_pos_blue_3.theta, 1000, {.forwards = true}, false); // take first ring
    
//     //second mogo
//     chassis.turnToPoint(qual_pos_blue_5a.x, qual_pos_blue_5a.y, 1000, {.forwards=true}, false); // turn to release mogo
//     clamp_solenoid.extend();
    
//     chassis.turnToPoint(qual_pos_blue_5a.x, qual_pos_blue_5a.y, 1000, {.forwards=false, .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false); // turn 2nd mogo
//     chassis.moveToPoint(qual_pos_blue_5a.x, qual_pos_blue_5a.y, 1000, {.forwards = false}, false); // go to mogo
//     chassis.moveToPoint(qual_pos_blue_5.x, qual_pos_blue_5.y, 1000, {.forwards = false}, false); // go to mogo
//     clamp_solenoid.retract();
//     pros::delay(100);
//     intake_controller.set(Intake::IntakeState::INTAKING);
//     pros::delay(500);
    
//     //alliance stake
//     chassis.turnToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 1000, {}, false); // turn to stack
//     chassis.moveToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 1000, {.forwards = true}, true); // go to stack
//     //arm_controller.moveTo(Arm::position::INTAKE);
//     intake_solenoid.extend();
//     chassis.waitUntilDone();
//     intake_solenoid.retract();
//     //chassis.moveToPoint(qual_pos_blue_6a.x, qual_pos_blue_6a.y, 1000, {.forwards = false}, false); // go to stack
//     //chassis.moveToPoint(qual_pos_blue_6b.x, qual_pos_blue_6b.y, 1000, {.forwards = true}, false); // go to stack
//     intake_controller.waitUntilDone();
//     //chassis.moveToPose(qual_pos_blue_7.x, qual_pos_blue_7.y, qual_pos_blue_7.theta, 1000, {.forwards = true}, false); // go to stake
    
//     //arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, false);
    
//     // chassis.moveToPoint(qual_pos_blue_6.x, qual_pos_blue_6.y, 1000, {.forwards = false}, false); // go away from stake
    
//     // chassis.moveToPoint(qual_pos_blue_8a.x, qual_pos_blue_8a.y, 1000, {.forwards = true}, false); // go to corner
    
//     // doinker_solenoid.extend();
    
//     // chassis.moveToPose(qual_pos_blue_8.x, qual_pos_blue_8.y, qual_pos_blue_8.theta, 1000, {.forwards = true}, false); // go to corner
    
//     // chassis.turnToHeading(qual_pos_blue_8.theta + 180, 1000);
    
//     // //touch ladder
//     // intake_controller.set(Intake::IntakeState::INTAKING);
//     // doinker_solenoid.retract();
//     // arm_controller.moveTo(-160, true);
//     // chassis.moveToPoint(qual_pos_blue_9.x, qual_pos_blue_9.y, 1000, {.forwards = true, .minSpeed = 64}, true); // go to ladder
// }



// //======================= elims 10p autons =======================

// lemlib::Pose elims_pos_red_0 = qual_pos_red_0;
// lemlib::Pose elims_pos_red_1 = qual_pos_red_1;
// lemlib::Pose elims_pos_red_2 = qual_pos_red_2;
// lemlib::Pose elims_pos_red_3 = qual_pos_red_3;
// lemlib::Pose elims_pos_red_4 = qual_pos_red_4;
// lemlib::Pose elims_pos_red_5 = qual_pos_red_5;
// lemlib::Pose elims_pos_red_6 = qual_pos_red_6;
// lemlib::Pose elims_pos_red_7 = qual_pos_red_7;
// lemlib::Pose elims_pos_red_8 = offsetPoint({-66.5,-66.5,NAN}, elims_pos_red_7, -13);

// void elims_pos_red() {
//     chassis.setPose(qual_pos_red_0.x, qual_pos_red_0.y, qual_pos_red_0.theta);

//     //goal rush
//     chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 1000, {.forwards = false}, false); //rush to mogo
//     chassis.moveToPoint(qual_pos_red_2.x, qual_pos_red_2.y, 1000, {.forwards = false}, false); //rush to mogo
//     clamp_solenoid.extend(); //clamp mogo

//     chassis.moveToPoint(qual_pos_red_1.x, qual_pos_red_1.y, 1000, {.forwards = true}, false);
//     chassis.moveToPoint(qual_pos_red_3.x , qual_pos_red_3.y, 1000, {.forwards = true}, false); 

//     //second mogo

//     chassis.turnToPoint(qual_pos_red_4.x, qual_pos_red_4.y, 1000, {.forwards=true}, false); // turn to ring
//     clamp_solenoid.retract();
//     intake_controller.hold(true);
//     chassis.moveToPoint(qual_pos_red_4.x, qual_pos_red_4.y, 1000, {.forwards = true}, false); // go to ring
//     intake_controller.waitUntilDone();
//     chassis.moveToPoint(qual_pos_red_5.x, qual_pos_red_5.y, 1000, {.forwards = false}, false); // go to mogo
//     clamp_solenoid.extend();
//     pros::delay(100);
//     intake_controller.set(Intake::IntakeState::INTAKING);

//     //alicence stake
//     chassis.turnToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {}, false); // turn to stake
//     chassis.moveToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {.forwards = true}, true); // go to stake
//     intake_solenoid.retract();
//     intake_controller.holdldb(true);

//     chassis.waitUntilDone();
//     intake_solenoid.extend();
//     intake_controller.waitUntilDone();

//     chassis.moveToPose(qual_pos_red_7.x, qual_pos_red_7.y, qual_pos_red_8.theta, 1000, {.forwards = true}, false);

//     arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, false);

//     //go to corner

//     chassis.moveToPoint(qual_pos_red_6.x, qual_pos_red_6.y, 1000, {.forwards = false}, true); // go to stake
//     chassis.turnToPoint(qual_pos_red_8.x, qual_pos_red_8.y, 1000, {.forwards=true}, false); // turn to corner

//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
//     chassis.moveToPoint(qual_pos_red_8.x, qual_pos_red_8.y, 1000, {.forwards = false, .maxSpeed = 96}, true);//go to ladder





//         chassis.moveToPoint(qual_pos_red_8.x, qual_pos_red_8.y, 1000, {.forwards = true}, false); // go to corner

//     // doinker_solenoid.extend();

//     // chassis.moveToPose(qual_pos_red_8.x, qual_pos_red_8.y, qual_pos_red_8.theta, 1000, {.forwards = true}, false); // go to corner

//     // chassis.turnToHeading(qual_pos_red_8.theta + 180, 1000);

//     // //touch ladder
//     // intake_controller.set(Intake::IntakeState::INTAKING);
//     // doinker_solenoid.retract();
//     // arm_controller.moveTo(-160,true);
//     // chassis.moveToPoint(qual_pos_red_9.x, qual_pos_red_9.y, 1000, {.forwards = true, .minSpeed = 64}, true);//go to ladder
// }

// lemlib::Pose elims_pos_blue_0 = qual_pos_blue_0;
// lemlib::Pose elims_pos_blue_1 = qual_pos_blue_1;
// lemlib::Pose elims_pos_blue_2 = qual_pos_blue_2;
// lemlib::Pose elims_pos_blue_3 = qual_pos_blue_3;
// lemlib::Pose elims_pos_blue_4 = qual_pos_blue_4;
// lemlib::Pose elims_pos_blue_5 = qual_pos_blue_5;
// lemlib::Pose elims_pos_blue_6 = qual_pos_blue_6;
// lemlib::Pose elims_pos_blue_7 = qual_pos_blue_7;
// lemlib::Pose elims_pos_blue_8 = mirrorPose(elims_pos_red_8);

// void elims_pos_blue() {
//     chassis.setPose(elims_pos_blue_0.x, elims_pos_blue_0.y, elims_pos_blue_0.theta);

//     //goal rush
//     chassis.moveToPoint(elims_pos_blue_1.x, elims_pos_blue_1.y, 1000, {.forwards = false}, false); //rush to mogo
//     chassis.moveToPoint(elims_pos_blue_2.x, elims_pos_blue_2.y, 1000, {.forwards = false}, false); //rush to mogo
//     clamp_solenoid.extend(); //clamp mogo

//     chassis.moveToPoint(elims_pos_blue_1.x, elims_pos_blue_1.y, 1000, {.forwards = true}, false);
//     chassis.moveToPoint(elims_pos_blue_3.x , elims_pos_blue_3.y, 1000, {.forwards = true}, false); 

//     //second mogo

//     chassis.turnToPoint(elims_pos_blue_4.x, elims_pos_blue_4.y, 1000, {.forwards=true}, false); // turn to ring
//     clamp_solenoid.retract();
//     intake_controller.hold(true);
//     chassis.moveToPoint(elims_pos_blue_4.x, elims_pos_blue_4.y, 1000, {.forwards = true}, false); // go to ring
//     intake_controller.waitUntilDone();
//     chassis.moveToPoint(elims_pos_blue_5.x, elims_pos_blue_5.y, 1000, {.forwards = false}, false); // go to mogo
//     clamp_solenoid.extend();
//     pros::delay(100);
//     intake_controller.set(Intake::IntakeState::INTAKING);

//     //alicence stake
//     chassis.turnToPoint(elims_pos_blue_6.x, elims_pos_blue_6.y, 1000, {}, false); // turn to stake
//     chassis.moveToPoint(elims_pos_blue_6.x, elims_pos_blue_6.y, 1000, {.forwards = true}, true); // go to stake
//     intake_solenoid.retract();
//     intake_controller.hold(true);

//     chassis.waitUntilDone();
//     intake_solenoid.extend();
//     intake_controller.waitUntilDone();

//     chassis.moveToPose(elims_pos_blue_7.x, elims_pos_blue_7.y, elims_pos_blue_8.theta, 1000, {.forwards = true}, false);

//     arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, false);

//     //go to corner

//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
//     chassis.moveToPoint(elims_pos_blue_8.x, elims_pos_blue_8.y, 1000, {.forwards = false, .maxSpeed = 96}, true);//go to ladder
// }

// //======================= skills autons =======================



// lemlib::Pose skills_0 = offsetPose({-70, 0, 270}, ALLIANCE_STAKE_OFFSET); 
// lemlib::Pose skills_1 = offsetPose({-47.244, -23.622, 180}, CLAMP_OFFSET);
// lemlib::Pose skills_1a = {NAN,NAN, 60};
// lemlib::Pose skills_2 = {-23.622, -23.622, 135};
// lemlib::Pose skills_3 = {-23.622, -47.244, 90};
// lemlib::Pose skills_4 = offsetPose({23.622, -47.244, 90},INTAKE_OFFSET);
// lemlib::Pose skills_5 = offsetPose({0, -59.055, 180},INTAKE_OFFSET);
// lemlib::Pose skills_5a = offsetPose({0, -70, 180},NEUTRAL_STAKE_OFFSET);
// lemlib::Pose skills_6 = offsetPose({-47.244, -47.244, 270},INTAKE_OFFSET); 
// lemlib::Pose skills_7 = offsetPose({-59.055, -47.244, 270},INTAKE_OFFSET);
// lemlib::Pose skills_8 = offsetPoint({-47.244, -59.055, 135},skills_7,INTAKE_OFFSET);
// lemlib::Pose skills_9 = offsetPoint({-65, -65, 45},skills_8,-10);

// lemlib::Pose skills_10 = offsetPoint({0, 0, 45},skills_9,INTAKE_OFFSET);
// lemlib::Pose skills_11 = offsetPose({-47.244, 23.622, 120},CLAMP_OFFSET);
// lemlib::Pose skills_12 = {-23.622, 23.622, 17.5};
// lemlib::Pose skills_13 = {-23.622, 47.244, 310};
// lemlib::Pose skills_14 = offsetPose({-59.055, 47.244, 270},INTAKE_OFFSET);
// lemlib::Pose skills_15 = offsetPoint({-47.244, 59.055, 45},skills_14,INTAKE_OFFSET);
// lemlib::Pose skills_16 = offsetPoint({-655, 65, 110},skills_15,-10);

// lemlib::Pose skills_17 = offsetPoint({0, 58, 110},skills_16,INTAKE_OFFSET);
// lemlib::Pose skills_18 = offsetPoint({23.622, 47.244, 110},skills_17,INTAKE_OFFSET);
// lemlib::Pose skills_19a = offsetPose({0, -59.055, 0},INTAKE_OFFSET);
// lemlib::Pose skills_19 = offsetPose({0, 70, 0},NEUTRAL_STAKE_OFFSET);
// lemlib::Pose skills_20 = {23.622, 23.622, 140};
// lemlib::Pose skills_21 = offsetPose({47.035, -0.447, 300},CLAMP_OFFSET);
// lemlib::Pose skills_22 = offsetPose({23.622, -23.622, 200},INTAKE_OFFSET);
// lemlib::Pose skills_23 = offsetPose({59.055, -47.244, 90},INTAKE_OFFSET);
// lemlib::Pose skills_24 = offsetPoint({47.244, -59.055, 0},skills_23,INTAKE_OFFSET);
// lemlib::Pose skills_25 = {47.244, 0, 0};
// lemlib::Pose skills_26 = offsetPose({47.244, 59.055, 0},INTAKE_OFFSET);
// lemlib::Pose skills_27 = offsetPoint({65, 65, 135},skills_26,-10);

// lemlib::Pose skills_28 = {59.055, 47.244, 210};
// lemlib::Pose skills_29 = {42.702, 24.236, 210};
// lemlib::Pose skills_30 = offsetPose({0, 59.055, 90},INTAKE_OFFSET);
// lemlib::Pose skills_30a = offsetPose({70, 0, 90},ALLIANCE_STAKE_OFFSET);
// lemlib::Pose skills_31 = offsetPoint({59.055, -23.622, NAN},skills_30a, CLAMP_OFFSET);
// lemlib::Pose skills_32 = offsetPoint({65, -65, 135},skills_31,-10);
// lemlib::Pose skills_33 = {16, -16, 135};

// void skills() {
//     chassis.setPose(skills_0.x, skills_0.y, skills_0.theta);
    
//     //allicen stake
//     arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
//     pros::delay(800);
//     //arm_controller.moveTo(Arm::position::RETRACT);

//     //goal 1 quadrent 3 + right wall stake

//     chassis.moveToPose(skills_1.x, skills_1.y, skills_1.theta, 1000, {.forwards=false}, true);
//     clamp_solenoid.extend();
//     arm_controller.moveTo(Arm::position::RETRACT);

//     chassis.turnToHeading(skills_1a.theta, 1000);
//     intake_controller.set(Intake::IntakeState::INTAKING);
//     chassis.moveToPose(skills_2.x, skills_2.y, skills_2.theta, 1000, {.forwards=true,.minSpeed=24}, false);
//     chassis.moveToPoint(skills_3.x, skills_3.y, 1000, {.forwards=true}, false);

//     chassis.moveToPoint(skills_4.x, skills_4.y, 1000, {.forwards=true}, false);

//     chassis.moveToPose(skills_5.x, skills_5.y, skills_5.theta, 1000, {.forwards=true}, true);
//     pros::delay(500);
//     arm_controller.moveTo(Arm::position::INTAKE, false);
//     intake_controller.holdldb(false);
//     chassis.moveToPoint(skills_5a.x, skills_5a.y, 1000, {.forwards=true}, false);
//     arm_controller.moveTo(Arm::position::SCORE_NEUTRAL,false);

//     chassis.moveToPose(skills_6.x , skills_6.y, skills_6.theta, 1000, {.forwards=true,.minSpeed=48}, true);
//     pros::delay(1000);
//     arm_controller.moveTo(Arm::position::RETRACT);
//     chassis.moveToPoint(skills_7.x, skills_7.y, 1000, {.forwards=true}, false);
//     chassis.moveToPoint(skills_8.x, skills_8.y, 1000, {.forwards=true}, false);

//     chassis.moveToPoint(skills_9.x, skills_9.y, 1000, {.forwards=false}, false);
//     clamp_solenoid.retract();

//     //goal 2 quadrent 2 + left wall stake
//     chassis.moveToPoint(skills_10.x, skills_10.y, 1000, {.forwards=true}, true);
//     chassis.waitUntil(20);
//     intake_controller.hold();

//     chassis.moveToPoint(skills_11.x, skills_11.y, 1000, {.forwards=false}, false);
//     clamp_solenoid.extend();

//     intake_controller.set(Intake::IntakeState::INTAKING);
//     chassis.moveToPose(skills_12.x, skills_12.y, skills_12.theta, 1000, {.forwards=true,.minSpeed=24}, false);
//     chassis.moveToPose(skills_13.x, skills_13.y, skills_13.theta, 1000, {.forwards=true,.minSpeed=24}, false);
//     chassis.moveToPose(skills_14.x, skills_14.y, skills_14.theta, 1000, {.forwards=true}, false);
//     chassis.moveToPoint(skills_15.x, skills_15.y, 1000, {.forwards=true}, false);

//     chassis.moveToPoint(skills_16.x, skills_16.y, 1000, {.forwards=false,.minSpeed=48}, false);
//     //intake_controller.set(Intake::IntakeState::STOPPED);
//     clamp_solenoid.retract();

//     // left wall stake
//     chassis.moveToPoint(skills_17.x, skills_17.y, 1000, {.forwards=true}, true);
//     //chassis.waitUntil(10);
//     arm_controller.moveTo(Arm::position::INTAKE);
//     intake_controller.holdldb(false);
//     //intake_controller.hold();
//     //chassis.moveToPoint(skills_18.x, skills_18.y, 1000, {.forwards=true}, true);
//     //chassis.waitUntil(8);
//     //intake_controller.set(Intake::IntakeState::INTAKING);
//     //intake_controller.waitUntilDone();

//     chassis.moveToPose(skills_19a.x, skills_19a.y, skills_19a.theta, 1000, {.forwards=true}, false);
//     chassis.moveToPoint(skills_19.x, skills_19.y, 1000, {.forwards=true}, false);

//     arm_controller.moveTo(Arm::position::SCORE_NEUTRAL, true);
//     pros::delay(800);
    
//     //next mogo

//     chassis.moveToPoint(skills_20.x, skills_20.y, 1000, {.forwards=true}, true);
//     arm_controller.moveTo(Arm::position::RETRACT);
//     intake_controller.hold();
//     chassis.moveToPose(skills_21.x, skills_21.y, skills_21.theta, 1000, {.forwards=false}, false);
//     clamp_solenoid.extend();
//     intake_controller.set(Intake::IntakeState::INTAKING);

//     chassis.moveToPose(skills_22.x, skills_22.y, skills_22.theta, 1000, {.forwards=true,.minSpeed=24}, false);
//     chassis.moveToPose(skills_23.x, skills_23.y, skills_23.theta, 1000, {.forwards=true}, false);
//     chassis.moveToPoint(skills_24.x, skills_24.y, 1000, {.forwards=true}, false);

//     chassis.moveToPoint(skills_25.x, skills_25.y, 1000, {.forwards=true,.minSpeed=48}, false);
//     chassis.moveToPoint(skills_26.x, skills_26.y, 1000, {.forwards=true}, true);

//     chassis.moveToPoint(skills_27.x, skills_27.y, 1000, {.forwards=false}, false);
//     clamp_solenoid.retract();

//     //goal 3 quadrent 1/4 + allience + goal4
//     arm_controller.moveTo(Arm::position::INTAKE);
//     intake_controller.holdldb();
    
//     //intake_controller.set(Intake::IntakeState::INTAKING);
//     chassis.moveToPoint(skills_28.x, skills_28.y, 1000, {.forwards=true}, false);
//     chassis.moveToPoint(skills_29.x, skills_29.y, 1000, {.forwards=true}, false);
    
//     chassis.moveToPose(skills_30.x, skills_30.y, skills_30.theta, 1000, {.forwards=true}, false);
//     chassis.moveToPoint(skills_30a.x, skills_30a.y, 1000, {.forwards=true}, false);

//     arm_controller.moveTo(Arm::position::SCORE_ALLIANCE, true);
//     pros::delay(800);

//     chassis.moveToPoint(skills_30.x, skills_30.y, 1000, {.forwards=true}, false);
//     arm_controller.moveTo(Arm::position::SCORE_NEUTRAL,true);

//     //clamp_solenoid.extend();
//     chassis.moveToPoint(skills_31.x, skills_31.y, 1000, {.forwards=true, .minSpeed=24}, false);
//     chassis.moveToPoint(skills_32.x, skills_32.y, 1000, {.forwards=true}, false);

//     //climb
//     chassis.moveToPose(skills_33.x, skills_33.y, skills_33.theta, 10000, {.forwards=false, .minSpeed=80}, false);
//     //chassis.turnToPoint(skills_33.x, skills_33.y, 1000, {.forwards=false}, false);
//     //dt_left.move(127);
//     //dt_right.move(127);
//     //pros::delay(20000);
// }

// /*======================= old autons =======================

// double meter_to_in (double meter) { return meter * 39.37008; }
// void autonIntake() { colorSortVision(); pros::delay(50); }

// lemlib::Pose awp_0 = {-52.5,-8.5, 300};
// lemlib::Pose awp_1 = {-60, -4.5, 300};
// lemlib::Pose awp_2 = {-26, -20, 300};
// lemlib::Pose awp_3 = {-30, -38, 160};
// lemlib::Pose awp_4 = {-42, -5, 330};
// lemlib::Pose awp_5 = {-28, -2, 90};

// void soloAWP_right_pos(){ // red
//     pros::Task autonIntakeThread(autonIntake);
//     autonIntakeThread.suspend();

//     chassis.setPose(awp_0.x, awp_0.y, awp_0.theta);
//     arm_controller.moveToAngle(90);
//     while (!arm_controller.isInPosition()) {pros::delay(10);};

//     chassis.moveToPoint(awp_1.x, awp_1.y, 1000, {}, false);
//     chassis.waitUntilDone();

//     arm_controller.moveToAngle(16);
//     pros::delay(500);
//     //while (!arm_controller.isInPosition()) {pros::delay(10);};

//     //set position, open clamp
//     clamp_solenoid.toggle();
//     chassis.moveToPoint(awp_0.x, awp_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
//     //move to mogo
//     chassis.moveToPose(awp_2.x, awp_2.y, awp_2.theta, 2000, {.forwards = false, .maxSpeed = 100}, false);

//     // // //grab mogo
//     pros::delay(50);
//     clamp_solenoid.toggle();
    
//     pros::delay(200);

//     autonIntakeThread.resume();

//     chassis.moveToPose(awp_3.x, awp_3.y, awp_3.theta, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     // pros::delay(200); //changed 1000 -> 800

//     arm_controller.moveToAngle(18);
//     intake_solenoid.toggle();

//     // //move to donut pile
//     chassis.moveToPose(awp_4.x, awp_4.y, awp_4.theta, 3000, {.forwards = true, .maxSpeed = 64}, false);

//     // //eat second donut
//     pros::delay(500);

//     // //move to ladder
//     chassis.moveToPose(awp_5.x, awp_5.y, awp_5.theta, 1000, {.forwards = true, .maxSpeed = 127}, true);
//     //chassis.waitUntil(2);
//     intake_solenoid.toggle();
//     pros::delay(10000);
// }

// void soloAWP_left_pos(){ // blue
//     pros::Task autonIntakeThread(autonIntake);
//     autonIntakeThread.suspend();

//     chassis.setPose(awp_0.x, -awp_0.y, -awp_0.theta - 180);
//     arm_controller.moveToAngle(90);
//     while (!arm_controller.isInPosition()) {pros::delay(10);};

//     chassis.moveToPoint(awp_1.x, -awp_1.y, 1000);
//     chassis.waitUntilDone();

//     arm_controller.moveToAngle(16);
//     pros::delay(500);

//     //set position, open clamp
//     clamp_solenoid.toggle();
//     chassis.moveToPoint(awp_0.x, -awp_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
//     //move to mogo
//     chassis.moveToPose(awp_2.x, -awp_2.y, -awp_2.theta - 180, 2000, {.forwards = false, .maxSpeed = 100}, false);

//     //grab mogo
//     pros::delay(50);
//     clamp_solenoid.toggle();

//     pros::delay(200);
    
//     autonIntakeThread.resume();
//     //pros::delay(200);

//     chassis.moveToPose(awp_3.x, -awp_3.y, -awp_3.theta-180, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     //pros::delay(200); //changed 1000 -> 800

//     arm_controller.moveToAngle(18);
//     intake_solenoid.toggle();
//     //autonIntakeThread.resume();

//     //move to donut pile
//     chassis.moveToPose(awp_4.x, -awp_4.y, -awp_4.theta - 180, 3000, {.forwards = true, .maxSpeed = 100}, false);

//     //eat second donut
//     pros::delay(500);

//     //move to ladder
//     chassis.moveToPose(awp_5.x, -awp_5.y, -awp_5.theta - 180, 5000, {.forwards = true, .maxSpeed = 127}, false);
//     chassis.waitUntil(5);
//     intake_solenoid.toggle();
//     pros::delay(10000);
// }

// lemlib::Pose awp_neg_0 = {awp_0.x, -awp_0.y, -awp_0.theta - 180};
// lemlib::Pose awp_neg_1 = {awp_1.x, -awp_1.y, -awp_1.theta - 180};
// lemlib::Pose awp_neg_2 = {awp_2.x, -awp_2.y, -awp_2.theta - 180};
// lemlib::Pose awp_neg_3 = {-7, 35, 210};
// lemlib::Pose awp_neg_4 = {-76, 60, 0};
// lemlib::Pose awp_neg_e = {-10,30, 30};
// lemlib::Pose awp_neg_5 = {-25, 45, 210};
// lemlib::Pose awp_neg_6 = {awp_4.x, -awp_4.y, -awp_4.theta - 180};
// lemlib::Pose awp_neg_7 = {awp_5.x, -awp_5.y, -awp_5.theta - 180};

// void soloAWP_left_neg(){ // red
//     pros::Task autonIntakeThread(autonIntake);
//     autonIntakeThread.suspend();

//     chassis.setPose(awp_neg_0.x, awp_neg_0.y, awp_neg_0.theta);
//     arm_controller.moveToAngle(90);
//     while (!arm_controller.isInPosition()) {pros::delay(10);};

//     chassis.moveToPoint(awp_neg_1.x, awp_neg_1.y, 1000);
//     chassis.waitUntilDone();

//     arm_controller.moveToAngle(16);
//     pros::delay(500);

//     //set position, open clamp
//     clamp_solenoid.toggle();
//     chassis.moveToPoint(awp_neg_0.x, awp_neg_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
//     //move to mogo
//     chassis.moveToPose(awp_neg_2.x, awp_neg_2.y, awp_neg_2.theta, 2000, {.forwards = false, .maxSpeed = 100}, false);

//     //grab mogo
//     pros::delay(50);
//     clamp_solenoid.toggle();
    
//     pros::delay(200);
//     autonIntakeThread.resume();

//     chassis.turnToHeading(-awp_neg_2.theta-180, 1000, {}, false);

//     autonIntakeThread.resume();

//     chassis.moveToPose(awp_neg_3.x, awp_neg_3.y, awp_neg_3.theta, 1000, {.forwards = true, .maxSpeed = 50, .minSpeed = 20, .earlyExitRange = 3}, false);
//     //chassis.moveToPoint(awp_neg_3.x, awp_neg_3.y, 2000, {.forwards = true, .maxSpeed = 50, .minSpeed = 20, .earlyExitRange = 3}, false);
//     chassis.moveToPoint(awp_neg_4.x, awp_neg_4.y, 1000, {.forwards = true, .maxSpeed = 50}, false);
//     chassis.moveToPoint(awp_neg_e.x, awp_neg_e.y, 1000, {.forwards = false, .maxSpeed = 80}, false);
//     chassis.moveToPoint(awp_neg_5.x, awp_neg_5.y, 2000, {.forwards = true, .maxSpeed = 127}, false);

//     intake_solenoid.toggle();

//     //move to donut pile
//     chassis.moveToPose(awp_neg_6.x, awp_neg_6.y, awp_neg_6.theta, 3000, {.forwards = true, .maxSpeed = 127}, false);

//     //eat second donut
//     pros::delay(500);

//     //move to ladder
//     chassis.moveToPose(awp_neg_7.x, awp_neg_7.y, awp_neg_7.theta, 1000, {.forwards = true, .maxSpeed = 127}, true);
//     chassis.waitUntil(5);
//     intake_solenoid.toggle();
// }

// void soloAWP_right_neg() { // blue
//     pros::Task autonIntakeThread(autonIntake);
//     autonIntakeThread.suspend();

//     chassis.setPose(awp_neg_0.x, -awp_neg_0.y, -awp_neg_0.theta - 180);
//     arm_controller.moveToAngle(90);
//     while (!arm_controller.isInPosition()) {pros::delay(10);};
    
//     chassis.moveToPoint(awp_neg_1.x, -awp_neg_1.y, 1000);
//     chassis.waitUntilDone();

//     arm_controller.moveToAngle(16);
//     pros::delay(500);

//     //set position, open clamp
//     clamp_solenoid.toggle();
//     chassis.moveToPoint(awp_neg_0.x, -awp_neg_0.y, 1000, {.forwards = false, .minSpeed = 60});

//     //move to mogo
//     chassis.moveToPose(awp_neg_2.x, -awp_neg_2.y, -awp_neg_2.theta - 180, 2000, {.forwards = false, .maxSpeed = 125}, false);

//     //grab mogo
//     pros::delay(50);
//     clamp_solenoid.toggle();

//     pros::delay(200);
//     autonIntakeThread.resume();

//     chassis.swingToHeading(-awp_neg_2.theta - 180, DriveSide::LEFT, 1000, {.minSpeed = 60});

//     chassis.moveToPoint(awp_neg_3.x, -awp_neg_3.y, 2000, {.forwards = true, .maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 3}, false);
//     chassis.moveToPoint(awp_neg_4.x, -awp_neg_4.y, 1000, {.forwards = true, .maxSpeed = 80}, false);
//     chassis.moveToPoint(awp_neg_e.x, -awp_neg_e.y, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 20}, false);
//     chassis.moveToPose(awp_neg_5.x, -awp_neg_5.y, -awp_neg_5.theta - 180, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 20}, false);

//     intake_solenoid.toggle();

//     //move to donut pile
//     chassis.moveToPose(awp_neg_6.x, -awp_neg_6.y, -awp_neg_6.theta - 180, 3000, {.forwards = true, .maxSpeed = 127}, false);

//     //eat second donut
//     pros::delay(500);

//     //move to ladder
//     chassis.moveToPose(awp_neg_7.x, -awp_neg_7.y, -awp_neg_7.theta - 180, 1000, {.forwards = true, .maxSpeed = 127}, true);
//     chassis.waitUntil(5);
//     intake_solenoid.toggle();

// }

// //======================= elims =======================

// lemlib::Pose mid_mogo = {-6, -47.5, 90};

// //middle donuts
// lemlib::Pose elims_1 = {-6.718, -42.645, 0};
// lemlib::Pose elims_2 = {-7.159, 52.352, 0};
// lemlib::Pose elims_3 = {-22.631, 47.184, 240};

// void elims_right () { //red
//     pros::Task autonIntakeThread(autonIntake);
//     autonIntakeThread.suspend();

//     chassis.setPose(awp_0.x, awp_0.y, awp_0.theta);
//     arm_controller.moveToAngle(90);
//     while (!arm_controller.isInPosition()) {pros::delay(10);};

//     chassis.moveToPoint(awp_1.x, awp_1.y, 1000, {}, false);
//     chassis.waitUntilDone();

//     arm_controller.moveToAngle(16);
//     pros::delay(500);
//     //while (!arm_controller.isInPosition()) {pros::delay(10);};

//     //set position, open clamp
//     clamp_solenoid.toggle();
//     chassis.moveToPoint(awp_0.x, awp_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
//     //move to mogo
//     chassis.moveToPose(awp_2.x, awp_2.y, awp_2.theta, 2000, {.forwards = false, .maxSpeed = 100}, false);

//     // // //grab mogo
//     pros::delay(50);
//     clamp_solenoid.toggle();
    
//     pros::delay(200);

//     autonIntakeThread.resume();

//     chassis.moveToPose(awp_3.x, awp_3.y, awp_3.theta, 2000, {.forwards = true, .maxSpeed = 100}, false);

//     arm_controller.moveToAngle(18);
//     intake_solenoid.toggle();

//     // //move to donut pile
//     chassis.moveToPose(awp_4.x, awp_4.y, awp_4.theta, 3000, {.forwards = true, .maxSpeed = 64}, false);

//     // //eat second donut
//     pros::delay(250);
//     autonIntakeThread.suspend();

//     //turn 180, drop mogo, turn back
//     chassis.turnToHeading(awp_4.theta + 180, 3000,{.maxSpeed = 64});
//     chassis.turnToHeading(awp_4.theta, 3000, {.maxSpeed = 64});
//     clamp_solenoid.toggle();
//     // //move to 3rd mogo
//     chassis.moveToPose(mid_mogo.x, mid_mogo.y, mid_mogo.theta, 1000, {.forwards = false, .maxSpeed = 127}, true);
//     // detectdonutthread.suspend();
//     intake_solenoid.toggle();
//     pros::delay(500);
//     autonIntakeThread.resume();
// }

// void elims_left() { // blue
//     pros::Task autonIntakeThread(autonIntake);
//     autonIntakeThread.suspend();

//     chassis.setPose(awp_0.x, awp_0.y, awp_0.theta);
//     arm_controller.moveToAngle(90);
//     while (!arm_controller.isInPosition()) {pros::delay(10);};

//     chassis.moveToPoint(awp_1.x, awp_1.y, 1000, {}, false);
//     chassis.waitUntilDone();

//     arm_controller.moveToAngle(16);
//     pros::delay(500);
//     //while (!arm_controller.isInPosition()) {pros::delay(10);};

//     //set position, open clamp
//     clamp_solenoid.toggle();
//     chassis.moveToPoint(awp_0.x, awp_0.y, 1000, {.forwards = false, .minSpeed = 60});
    
//     //move to mogo
//     chassis.moveToPose(awp_2.x, awp_2.y, awp_2.theta, 2000, {.forwards = false, .maxSpeed = 100}, false);

//     // // //grab mogo
//     pros::delay(50);
//     clamp_solenoid.toggle();
    
//     pros::delay(200);

//     autonIntakeThread.resume();

//     chassis.moveToPose(awp_3.x, awp_3.y, awp_3.theta, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     // pros::delay(200); //changed 1000 -> 800

//     arm_controller.moveToAngle(18);
//     intake_solenoid.toggle();

//     // //move to donut pile
//     chassis.moveToPose(awp_4.x, awp_4.y, awp_4.theta, 3000, {.forwards = true, .maxSpeed = 64}, false);

//     // //eat second donut
//     pros::delay(500);

//     //turn 180, drop mogo, turn back
//     chassis.turnToHeading(awp_4.theta + 180, 3000);
//     clamp_solenoid.toggle();
//     chassis.turnToHeading(awp_4.theta, 3000);

//     // //move to 3rd mogo
//     chassis.moveToPose(mid_mogo.x, mid_mogo.y, mid_mogo.theta, 1000, {.forwards = false, .maxSpeed = 127}, true);
//     intake_solenoid.toggle();
//     pros::delay(500);
// }


// //======================= skills =======================

// void skills () {
//     pros::Task autonIntakeThread(autonIntake);
//     autonIntakeThread.suspend();
    
//     //setup
//     clamp_solenoid.toggle();
//     dt_left.move_relative(-0.35, 150);
//     dt_right.move_relative(-0.35, 150);
//     pros::delay(2000);
//     chassis.setPose(-55, 0 ,270);

//     //alliance stake
//     arm_controller.moveToAngle(90);
//     pros::delay(500);
//     dt_left.move_relative(1, 150);
//     dt_right.move_relative(1, 150);
//     pros::delay(2000);
//     arm_controller.moveToAngle(16);
//     pros::delay(500);
//     dt_left.move(0);
//     dt_right.move(0);
//     dt_left.move_relative(-1, 300);
//     dt_right.move_relative(-1, 300);

//     chassis.moveToPoint(-53, -20, 2000, {.forwards = false, .maxSpeed = 75}, false);
//     pros::delay(1000);
//     clamp_solenoid.toggle();
//     pros::delay(1000);

//     chassis.turnToPoint(-65, -46, 2000);
//     autonIntakeThread.resume();
//     chassis.moveToPoint(-65, -46, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-46, -46, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-25.3, -25, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-46, -46, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-24.5, -47.2, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-46, -46, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-47, -59, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     chassis.turnToPoint(-64, -64, 2000, {.forwards = true});
//     chassis.moveToPoint(-64, -64, 2000, {.forwards = false, .maxSpeed = 75}, true);
//     pros::delay(2000);
//     autonIntakeThread.suspend();
//     clamp_solenoid.toggle();
//     pros::delay(500);
//     chassis.moveToPoint(-55, -55, 2000, {.forwards = false, .maxSpeed = 100}, true);

//     chassis.moveToPoint(-47, 17, 3000, {.forwards = false, .maxSpeed = 100}, false);
//     chassis.waitUntilDone();
//     clamp_solenoid.toggle();
//     pros::delay(500);
//     autonIntakeThread.resume();
//     chassis.turnToPoint(-26, 20, 2000, {.forwards = true, .maxSpeed = 127}, false);
//     chassis.moveToPoint(-26, 20, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     chassis.moveToPoint(-22, 45, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     chassis.turnToPoint(-44, 47, 2000, {.forwards = true});
//     chassis.moveToPoint(-44, 47, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     chassis.moveToPoint(-60, 47, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     chassis.turnToPoint(-48, 57, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     chassis.moveToPoint(-48, 57, 2000, {.forwards = true, .maxSpeed = 100}, false);
//     chassis.turnToPoint(-66, 66, 2000, {.forwards = false, .maxSpeed = 127}, false);
//     chassis.waitUntilDone();
//     clamp_solenoid.toggle();
//     pros::delay(500);
//     chassis.moveToPoint(-44, 44, 2000, {.forwards = true, .maxSpeed = 127}, false);
//     arm_controller.moveToAngle(150);
//     chassis.moveToPoint(-10, 10, 10000, {.forwards = true, .maxSpeed = 100}, false);
//     chassis.waitUntilDone();
//     arm_controller.moveToAngle(10);
// }
// */