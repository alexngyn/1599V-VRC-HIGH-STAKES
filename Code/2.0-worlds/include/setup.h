#pragma once

#include "api.h" // IWYU pragma: keep
#include "arm.h" 
#include "led.h" // IWYU pragma: keep
#include "intake.h"

enum protocol { bluetooth, vexnet }; 
enum color { red, blue, unknown };

std::string colorToString(color color);

#define CONTROLLER_MODE vexnet
extern color sideColor; // what side we on

//controllers
extern pros::Controller master;
extern pros::Controller partner;

// drivetrain
#define RIGHT_MOTOR_PORTS { 3, 4, 5 }
#define LEFT_MOTOR_PORTS { -8, -9, -10 }

//#define HORIZONTAL_ENCODER_PORT 10
#define VERTICAL_ENCODER_PORT -6
//extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;

#define INTAKE_SOLENOID_PORT 'C'
extern pros::adi::Pneumatics intake_solenoid;

extern pros::MotorGroup dt_left;
extern pros::MotorGroup dt_right;
extern lemlib::Chassis chassis;

#define INERTIAL_SENSOR_PORT 18
extern pros::IMU inertial_sensor;

//mogo clamp
#define CLAMP_SOLENOID_PORT 'D'
extern pros::adi::Pneumatics clamp_solenoid;

//doinker
#define DOINKER_PORT 'E'
extern pros::adi::Pneumatics doinker_solenoid;

//Auton Selection
#define INDICATOR_PORT 'A'
extern pros::adi::DigitalOut indicator;
#define SELECTION_PORT 'B'
extern pros::adi::AnalogIn selector;

//LEDs
#define LED_1_PORT 'F'
#define LED_2_PORT 'G'
#define LED_1_LENGTH 51
#define LED_2_LENGTH 51

//intake
#define INTAKE_PORT -7
extern pros::Motor intake_motor;

//arm
#define ARM_PORTS { 12,-14 }
extern pros::MotorGroup arm_motors;
#define ARM_ROTATIONAL_SENSOR_PORT -16
extern pros::Rotation arm_rotational_sensor;
#define OPTICAL_SENSOR_PORT 15 
extern pros::Optical optical_sensor;
#define DISTANCE_SENSOR_PORT 17
extern pros::Optical distance_sensor;

extern Arm arm_controller;
extern pros::Rotation arm_rotational_sensor;
extern Intake intake_controller;

// extern rd::Console console;

// extern rd::Image planet;
// extern rd::Image goof;