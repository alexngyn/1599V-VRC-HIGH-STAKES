#pragma once

#include "api.h" // IWYU pragma: keep
#include "arm.h" 
#include "led.h" // IWYU pragma: keep
#include "intake.h"

enum protocol { bluetooth, vexnet}; 
enum color { red, blue, unknown };

std::string colorToString(color color);

#define CONTROLLER_MODE vexnet
extern color sideColor; // what side we on

//controllers
extern pros::Controller master;
extern pros::Controller partner;

// drivetrain
#define RIGHT_MOTOR_PORTS { 1, 2, 3 }
#define LEFT_MOTOR_PORTS { -16, -17, -18 }
//#define DT_MOTOR_PORTS { 1, 2, 3, -16, -17, -18 }

//#define HORIZONTAL_ENCODER_PORT 10
#define VERTICAL_ENCODER_PORT -15
//extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;

#define INTAKE_SOLENOID_PORT 'D'
extern pros::adi::Pneumatics intake_solenoid;

extern pros::MotorGroup dt_left;
extern pros::MotorGroup dt_right;
extern lemlib::Chassis chassis;

#define INERTIAL_SENSOR_PORT 14
extern pros::IMU inertial_sensor;

//mogo clamp
#define CLAMP_SOLENOID_PORT 'H'
extern pros::adi::Pneumatics clamp_solenoid;

//doinker
#define DOINKER_PORT 'G'
extern pros::adi::Pneumatics doinker_solenoid;

//Auton Selection
#define INDICATOR_PORT 'A'
extern pros::adi::DigitalOut indicator;
#define SELECTION_PORT 'B'
extern pros::adi::AnalogIn selector;

//LEDs
#define LED_1_PORT 'E'
#define LED_2_PORT 'F'
#define LED_1_LENGTH 55
#define LED_2_LENGTH 55

//intake
#define INTAKE_PORT -11
extern pros::Motor intake_motor;

//arm
#define ARM_PORTS { -20, -19 }
extern pros::MotorGroup arm_motors;
#define ARM_ROTATIONAL_SENSOR_PORT 21
extern pros::Rotation arm_rotational_sensor;
#define OPTICAL_SENSOR_PORT 6
extern pros::Optical optical_sensor;
#define DISTANCE_SENSOR_PORT 7
extern pros::Distance distance_sensor;

// #define VISION_SENSOR_PORT 19
// extern pros::Vision vision_sensor;

extern Arm arm_controller;
extern pros::Rotation arm_rotational_sensor;
extern Intake intake_controller;

// extern pros::vision_signature_s_t REDSIG;
// extern pros::vision_signature_s_t BLUESIG;