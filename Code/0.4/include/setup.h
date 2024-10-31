#pragma once

#include "api.h" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "arm.h" 
#include "pros/optical.hpp"

enum protocol { bluetooth, vexnet}; 
enum color { red, blue };
enum side { left, right };

#define CONTROLLER_MODE vexnet
#define sideColor red // what side we on
#define autonSide right

#define RIGHT_MOTOR_PORTS { 1, 2, 3 }
#define LEFT_MOTOR_PORTS { -16, -17, -18 }
#define DT_MOTOR_PORTS { 1, 2, 3, -16, -17, -18 }

#define CLAMP_SOLENOID_PORT 'H'

//#define INDICATOR_G_PORT 'B'

#define LED_1_PORT 'A'
//#define LED_2_PORT 'H'
#define LED_1_LENGTH 64
//#define LED_2_LENGTH 64

#define INTAKE_PORT -11

#define ARM_PORT -20
#define ARM_ROTATIONAL_SENSOR_PORT 21

#define INERTIAL_SENSOR_PORT 14

#define OPTICAL_SENSOR_PORT 6

extern pros::Controller master;
extern pros::Controller partner;

extern pros::MotorGroup dt_left;
extern pros::MotorGroup dt_right;

extern pros::adi::Pneumatics clamp_solenoid;

extern pros::adi::DigitalOut indicator_g;

extern pros::Motor intake_motor;

extern pros::Motor arm_motor;

extern pros::IMU inertial_sensor;

extern lemlib::Chassis chassis;

extern Arm arm_controller;

extern pros::Rotation arm_rotational_sensor;

extern pros::Optical optical_sensor;