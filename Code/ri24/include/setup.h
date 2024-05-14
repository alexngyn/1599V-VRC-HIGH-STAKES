#pragma once

#include "api.h"
#include "pros/motors.hpp"

enum protocol { bluetooth, vexnet}; 

#define CONTROLLER_MODE bluetooth

#define DT_WHEEL_LF_PORT 1
#define DT_WHEEL_LB_PORT 2
#define DT_WHEEL_RF_PORT 9
#define DT_WHEEL_RB_PORT 10

#define ARM_L_PORT 3
#define ARM_R_PORT 8

#define INTAKE_PORT 7

extern pros::Controller master;

extern pros::Motor dt_motor_lf;
extern pros::Motor dt_motor_lb;

extern pros::Motor dt_motor_rf;
extern pros::Motor dt_motor_rb;

extern pros::Motor_Group dt_left;
extern pros::Motor_Group dt_right;

extern pros::Motor arm_motor_left;
extern pros::Motor arm_motor_right;
extern pros::Motor_Group arm;

extern pros::Motor intake_motor;